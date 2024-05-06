
#include "draw.hpp"
#include "camera.hpp"
#include "gl_render_triangles.hpp"
#include "gl_render_lines.hpp"


Camera g_camera;
DebugDraw g_debug_draw;
b3Vec3r g_light_color = {1.0f, 1.0f, 1.0f};
b3Vec3r g_light_position = {20.0f, 20.0f, 20.0f};


struct GLRenderPoints
{
    void create() {
        const char* vs = \
			"#version 330\n"
            "uniform mat4 projection_matrix;\n"
            "uniform mat4 view_matrix;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "layout(location = 2) in float v_size;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projection_matrix * view_matrix * vec4(v_position, 1.0f);\n"
            "   gl_PointSize = v_size;\n"
            "}\n";

        const char* fs = \
			"#version 330\n"
            "in vec4 f_color;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "	color = f_color;\n"
            "}\n";

        m_program_id = create_shader_program(vs, fs);
        m_projection_uniform = glGetUniformLocation(m_program_id, "projection_matrix");
        m_view_matrix = glGetUniformLocation(m_program_id, "view_matrix");

        m_vertex_attribute = 0;
        m_color_attribute = 1;
        m_size_attribute = 2;

        // Generate
        glGenVertexArrays(1, &m_vao_id);
        glGenBuffers(3, m_vbo_ids);

        glBindVertexArray(m_vao_id);
        glEnableVertexAttribArray(m_vertex_attribute);
        glEnableVertexAttribArray(m_color_attribute);
        glEnableVertexAttribArray(m_size_attribute);

        // vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glVertexAttribPointer(m_vertex_attribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glVertexAttribPointer(m_color_attribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[2]);
        glVertexAttribPointer(m_size_attribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_sizes), m_sizes, GL_DYNAMIC_DRAW);

        check_gl_error();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    void destroy()
    {
        if (m_vao_id)
        {
            glDeleteVertexArrays(1, &m_vao_id);
            glDeleteBuffers(3, m_vbo_ids);
            m_vao_id = 0;
        }

        if (m_program_id)
        {
            glDeleteProgram(m_program_id);
            m_program_id = 0;
        }
    }

    void vertex(const b3Vec3r& v, const b3Color& c, float size)
    {
        if (m_count == e_maxVertices)
            flush();

        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        m_sizes[m_count] = size;
        ++m_count;
    }

    void flush()
    {
        if (m_count == 0)
            return;

        glUseProgram(m_program_id);

        float proj[16] = { 0.0f };
        float view[16] = { 0.0f };
        g_camera.build_projection_matrix(proj);
        g_camera.build_view_matrix(view);
        glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, proj);
        glUniformMatrix4fv(m_view_matrix, 1, GL_FALSE, view);
        glBindVertexArray(m_vao_id);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3r), m_vertices);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float), m_sizes);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, m_count);
        glDisable(GL_PROGRAM_POINT_SIZE);

        check_gl_error();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 512 };
    b3Vec3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];
    float m_sizes[e_maxVertices];

    int32 m_count;

    GLuint m_vao_id;
    GLuint m_vbo_ids[3];
    GLuint m_program_id;
    GLint m_projection_uniform;
    GLint m_view_matrix;
    GLint m_vertex_attribute;
    GLint m_color_attribute;
    GLint m_size_attribute;
};


void DebugDraw::create()
{
    m_points = new GLRenderPoints;
    m_points->create();

    m_lines = new GLRenderLines;
    m_lines->create();

    m_triangles = new gl_render_triangles;
    m_triangles->create();
}


void DebugDraw::flush()
{
    m_points->flush();
    m_lines->flush();
    m_triangles->flush();
}


void DebugDraw::destroy()
{
    m_points->destroy();
    delete m_points;
    m_points = nullptr;

    m_lines->destroy();
    delete m_lines;
    m_lines = nullptr;

    m_triangles->destroy();
    delete m_triangles;
    m_triangles = nullptr;
}


void DebugDraw::draw_box(const b3CubeShape* cube, const b3Transformr& xf, const b3Color& color)
{
    b3Vec3r v[8], n[6];
    for (int32 i = 0; i < 8; ++i)
        v[i] = xf.transform(cube->m_vertices[i]);

    for (int32 i = 0; i < 6; ++i)
        n[i] = xf.rotate(cube->m_normals[i]);

    const b3FaceIndex* faces = cube->m_faces;
    const b3EdgeIndex* edges = cube->m_edges;
    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        for (int32 i = 0; i < 12; i++) {
            m_lines->vertex(v[edges[i].v1], color);
            m_lines->vertex(v[edges[i].v2], color);
        }
    } else {
        for (int32 i = 0; i < 6; i++) {
            m_triangles->vertex(v[faces[i].e1], n[i], color);
            m_triangles->vertex(v[faces[i].e2], n[i], color);
            m_triangles->vertex(v[faces[i].e3], n[i], color);
            m_triangles->vertex(v[faces[i].e3], n[i], color);
            m_triangles->vertex(v[faces[i].e4], n[i], color);
            m_triangles->vertex(v[faces[i].e1], n[i], color);
        }
    }
}


void DebugDraw::draw_plane(const b3PlaneShape* plane, const b3Transformr &xf, const b3Color &color)
{
    b3Vec3r lf = xf.position();
    b3Vec3r d_w = {plane->m_half_width, 0.0f, 0.0f};
    b3Vec3r d_l = {0.0f, plane->m_half_length, 0.0f};

    d_w = xf.transform(d_w);
    d_l = xf.transform(d_l);

    lf = lf - d_w - d_l;

    d_w = 2.0 * d_w / real(m_plane_segment);
    d_l = 2.0 * d_l / real(m_plane_segment);

    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        for (int32 i = 0; i < m_plane_segment; i++) {
            for (int32 j  = 0; j < m_plane_segment; j++) {

                m_lines->vertex(lf + i * d_w + j * d_l, color);
                m_lines->vertex(lf + (i + 1) * d_w + j * d_l, color);

                m_lines->vertex(lf + (i + 1) * d_w + j * d_l, color);
                m_lines->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, color);

                m_lines->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, color);
                m_lines->vertex(lf + i * d_w + (j + 1) * d_l, color);

                m_lines->vertex(lf + i * d_w + (j + 1) * d_l, color);
                m_lines->vertex(lf + i * d_w + j * d_l, color);
            }

        }
    } else {
        for (int32 i = 0; i < m_plane_segment; i++) {
            for (int j = 0; j < m_plane_segment; j++) {
                m_triangles->vertex(lf + i * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + i * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + i * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
            }
        }
    }
}


void DebugDraw::draw_sphere(const b3SphereShape* sphere, const b3Transformr &xf, const b3Color &color)
{
    // vertex position
    float x, y, z, xy;
    float radius = sphere->get_radius();
    // vertex normal
    float nx, ny, nz, lengthInv = 1.0f / radius;

    const int32 sector_count = 20;
    const int32 stack_count = 20;

    b3Vec3r vertices[(sector_count + 1) * (stack_count + 1)];
    b3Vec3r normals[(sector_count + 1) * (stack_count + 1)];

    float sector_step = 2.0f * b3_pi / sector_count;
    float stack_step = b3_pi / stack_count;
    float sector_angle, stack_angle;

    int32 index_v = 0;
    int32 index_n = 0;
    for (int32 i = 0; i <= stack_count; ++i) {
        stack_angle = b3_pi / 2.0f - i * stack_step;
        xy = radius * cosf(stack_angle);

        z = radius * sinf(stack_angle);

        for (int32 j = 0; j <= sector_count; ++j) {
            sector_angle = j * sector_step;

            x = xy * cosf(sector_angle);
            y = xy * sinf(sector_angle);
            vertices[index_v++] = b3Vec3r(x, y, z);

            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;
            normals[index_n++] = b3Vec3r(nx, ny, nz);

        }
    }


    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        int32 k1, k2;
        for (int32 i = 0; i < stack_count; ++i) {
            k1 = i * (sector_count + 1);
            k2 = k1 + sector_count + 1;

            for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
                if (i != 0) {
                    m_lines->vertex(xf.transform(vertices[k1]), color);
                    m_lines->vertex(xf.transform(vertices[k2]), color);
                    m_lines->vertex(xf.transform(vertices[k1 + 1]), color);
                }

                if (i != (stack_count - 1)) {
                    m_lines->vertex(xf.transform(vertices[k1 + 1]), color);
                    m_lines->vertex(xf.transform(vertices[k2]), color);
                    m_lines->vertex(xf.transform(vertices[k2 + 1]), color);
                }
            }
        }

    } else {
        int32 k1, k2;
        for (int32 i = 0; i < stack_count; ++i) {
            k1 = i * (sector_count + 1);
            k2 = k1 + sector_count + 1;

            for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
                if (i != 0) {
                    m_triangles->vertex(xf.transform(vertices[k1]), xf.transform(normals[k1]), color);
                    m_triangles->vertex(xf.transform(vertices[k2]), xf.transform(normals[k2]), color);
                    m_triangles->vertex(xf.transform(vertices[k1 + 1]), xf.transform(normals[k1 + 1]), color);
                }

                if (i != (stack_count - 1)) {
                    m_triangles->vertex(xf.transform(vertices[k1 + 1]), xf.transform(normals[k1 + 1]), color);
                    m_triangles->vertex(xf.transform(vertices[k2]), xf.transform(normals[k2]), color);
                    m_triangles->vertex(xf.transform(vertices[k2 + 1]), xf.transform(normals[k2 + 1]), color);
                }
            }
        }
    }
}


void DebugDraw::draw_segment(const b3Vec3r &p1, const b3Vec3r &p2, const b3Color &color)
{
    m_lines->vertex(p1, color);
    m_lines->vertex(p2, color);
}


