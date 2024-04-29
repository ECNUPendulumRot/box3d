
#include "draw.hpp"
#include "camera.hpp"
#include "gl_render_triangles.hpp"


Camera g_camera;
DebugDraw g_debug_draw;
float g_light_color[3] = {1.0f, 1.0f, 1.0f};
float g_light_position[3] = {20.0f, 100.0f, 20.0f};


struct GLRenderPoints
{
    void create()
    {
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

    void vertex(const b3Vector3r& v, const b3Color& c, float size)
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
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vector3r), m_vertices);

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
    b3Vector3r m_vertices[e_maxVertices];
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


struct GLRenderLines
{
    void create()
    {
        const char* vs = \
			"#version 330\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
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
        m_projection_uniform = glGetUniformLocation(m_program_id, "projectionMatrix");
        m_light_uniform = glGetUniformLocation(m_program_id, "light_color");
        m_vertex_attribute = 0;
        m_color_attribute = 1;

        // Generate
        glGenVertexArrays(1, &m_vao_id);
        glGenBuffers(2, m_vbo_ids);

        glBindVertexArray(m_vao_id);
        glEnableVertexAttribArray(m_vertex_attribute);
        glEnableVertexAttribArray(m_color_attribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glVertexAttribPointer(m_vertex_attribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glVertexAttribPointer(m_color_attribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

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
            glDeleteBuffers(2, m_vbo_ids);
            m_vao_id = 0;
        }

        if (m_program_id)
        {
            glDeleteProgram(m_program_id);
            m_program_id = 0;
        }
    }

    void vertex(const b3Vector3r& v, const b3Color& c)
    {
        if (m_count == e_maxVertices)
            flush();

        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        ++m_count;
    }

    void flush()
    {
        if (m_count == 0)
            return;

        glUseProgram(m_program_id);

        float proj[16] = { 0.0f };
        g_camera.build_projection_matrix(proj);

        glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, proj);
        glUniform3fv(m_light_uniform, 1, g_light_color);

        glBindVertexArray(m_vao_id);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vector3r), m_vertices);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);

        glEnable(GL_LINE_SMOOTH);
        glDrawArrays(GL_LINES, 0, m_count);
        glDisable(GL_LINE_SMOOTH);
        check_gl_error();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 2 * 512 };
    b3Vector3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];

    int32 m_count;

    GLuint m_vao_id;
    GLuint m_vbo_ids[2];
    GLuint m_program_id;
    GLint m_projection_uniform;
    GLint m_light_uniform;
    GLint m_vertex_attribute;
    GLint m_color_attribute;
};


void DebugDraw::draw_box(const b3EdgeIndex* edge_index, const b3FaceIndex* face_index,
                         const b3Vector3r* n, const b3Vector3r* v, const b3Color& color)
{
    for (int32 i = 0; i < 6; i++) {
        m_triangles->vertex({v[face_index[i].e1].y, v[face_index[i].e1].z, v[face_index[i].e1].x},
                            {n[i].y, n[i].z, n[i].x}, color);
        m_triangles->vertex({v[face_index[i].e2].y, v[face_index[i].e2].z, v[face_index[i].e2].x},
                            {n[i].y, n[i].z, n[i].x}, color);
        m_triangles->vertex({v[face_index[i].e3].y, v[face_index[i].e3].z, v[face_index[i].e3].x},
                            {n[i].y, n[i].z, n[i].x}, color);
        m_triangles->vertex({v[face_index[i].e3].y, v[face_index[i].e3].z, v[face_index[i].e3].x},
                            {n[i].y, n[i].z, n[i].x}, color);
        m_triangles->vertex({v[face_index[i].e4].y, v[face_index[i].e4].z, v[face_index[i].e4].x},
                            {n[i].y, n[i].z, n[i].x}, color);
        m_triangles->vertex({v[face_index[i].e1].y, v[face_index[i].e1].z, v[face_index[i].e1].x},
                            {n[i].y, n[i].z, n[i].x}, color);
    }
}


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


