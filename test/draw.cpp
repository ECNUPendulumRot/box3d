
#include "draw.hpp"

Camera g_camera;
DebugDraw g_debug_draw;
static float g_light_color[3] = {0.8f, 0.8f, 0.8f};
static float g_light_position[3] = {0.0f, 10.0f, 0.0f};

#define BUFFER_OFFSET(x)  ((const void*) (x))


static void s_check_gl_error()
{
    GLenum errCode = glGetError();
    if (errCode != GL_NO_ERROR)
    {
        fprintf(stderr, "OpenGL error = %d\n", errCode);
        assert(false);
    }
}


static void s_print_log(GLuint object)
{
    GLint log_length = 0;
    if (glIsShader(object))
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else if (glIsProgram(object))
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else
    {
        fprintf(stderr, "printlog: Not a shader or a program\n");
        return;
    }

    char* log = (char*)malloc(log_length);

    if (glIsShader(object))
        glGetShaderInfoLog(object, log_length, NULL, log);
    else if (glIsProgram(object))
        glGetProgramInfoLog(object, log_length, NULL, log);

    fprintf(stderr, "%s", log);
    free(log);
}


static GLuint s_create_shader_from_string(const char* source, GLenum type)
{
    GLuint res = glCreateShader(type);
    const char* sources[] = { source };
    glShaderSource(res, 1, sources, NULL);
    glCompileShader(res);
    GLint compile_ok = GL_FALSE;
    glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
    if (compile_ok == GL_FALSE)
    {
        fprintf(stderr, "Error compiling shader of type %d!\n", type);
        s_print_log(res);
        glDeleteShader(res);
        return 0;
    }

    return res;
}


static GLuint s_create_shader_program(const char* vs, const char* fs)
{
    GLuint vsId = s_create_shader_from_string(vs, GL_VERTEX_SHADER);
    GLuint fsId = s_create_shader_from_string(fs, GL_FRAGMENT_SHADER);
    assert(vsId != 0 && fsId != 0);

    GLuint programId = glCreateProgram();
    glAttachShader(programId, vsId);
    glAttachShader(programId, fsId);
    glBindFragDataLocation(programId, 0, "color");
    glLinkProgram(programId);

    glDeleteShader(vsId);
    glDeleteShader(fsId);

    GLint status = GL_FALSE;
    glGetProgramiv(programId, GL_LINK_STATUS, &status);
    assert(status != GL_FALSE);

    return programId;
}


void Camera::build_projection_matrix(float* m) const
{
    float w = float(m_width);
    float h = float(m_height);
    float ratio = w / h;

    float sd_h = 1.0f;
    float far = 100.0f;
    float p11 = 1.0f / (ratio * sd_h * m_zoom);
    float p22 = 1.0f / (sd_h * m_zoom);
    float p33 = -(far + 1.0f / m_zoom) / (far - 1.0f / m_zoom);
    float p34 = -2.0f * far / (far * m_zoom - 1.0f);

    b3Vector3r u = {0, 1, 0};
    auto f = (m_position - m_lookat).normalized();
    auto s = u.cross(f).normalized();
    u = f.cross(s).normalized();

    m[0] = p11 * s.x();
    m[1] = p22 * u.x();
    m[2] = p33 * f.x();
    m[3] = -f.x();

    m[4] = p11 * s.y();
    m[5] = p22 * u.y();
    m[6] = p33 * f.y();
    m[7] = -f.y();

    m[8] = p11 * s.z();
    m[9] = p22 * u.z();
    m[10] = p33 * f.z();
    m[11] = -f.z();

    m[12] = -p11 * s.dot(m_position);
    m[13] = -p22 * u.dot(m_position);
    m[14] = -p33 * f.dot(m_position) + p34;
    m[15] = f.dot(m_position);

}


struct GLRenderPoints
{
    void create()
    {
        const char* vs = \
			"#version 330\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "layout(location = 2) in float v_size;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
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

        m_program_id = s_create_shader_program(vs, fs);
        m_projection_uniform = glGetUniformLocation(m_program_id, "projectionMatrix");
        m_view_matrix = glGetUniformLocation(m_program_id, "viewMatrix");
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

        s_check_gl_error();

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
        g_camera.build_projection_matrix(proj);

        glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, proj);

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

        s_check_gl_error();

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

        m_program_id = s_create_shader_program(vs, fs);
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

        s_check_gl_error();

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
        s_check_gl_error();

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


struct GLRenderTriangles
{
    void create()
    {
        const char* vs = \
			"#version 330\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "layout(location = 2) in vec3 v_normal;\n"
            "out vec3 f_color;\n"
            "out vec3 f_normal;\n"
            "out vec3 f_frag_pos;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color.rgb;\n"
            "	f_normal = v_normal;\n"
            "	f_frag_pos = v_position;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
            "}\n";

        const char* fs = \
			"#version 330\n"
            "uniform vec3 light_color;\n"
            "uniform vec3 light_pos;\n"
            "in vec3 f_color;\n"
            "in vec3 f_normal;\n"
            "in vec3 f_frag_pos;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "   float ambient_factor = 0.3f;\n"
            "   vec3 ambient = ambient_factor * light_color;\n"
            "   \n"
            "   vec3 light_dir = normalize(light_pos - f_frag_pos);\n"
            "   float diff = max(dot(f_normal, light_dir), 0.0);\n"
            "   vec3 diffuse = diff * light_color;\n"
            "   \n"
            "	color = vec4((ambient) * f_color, 1.0);\n"
            "}\n";

        m_program_id = s_create_shader_program(vs, fs);
        m_projection_uniform = glGetUniformLocation(m_program_id, "projectionMatrix");
        m_color_uniform = glGetUniformLocation(m_program_id, "light_color");
        m_light_pos_uniform = glGetUniformLocation(m_program_id, "light_pos");

        m_vertex_attribute = 0;
        m_color_attribute = 1;
        m_normal_attribute = 2;

        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(3, m_vbo_ids);

        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertex_attribute);
        glEnableVertexAttribArray(m_color_attribute);
        glEnableVertexAttribArray(m_normal_attribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glVertexAttribPointer(m_vertex_attribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glVertexAttribPointer(m_color_attribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[2]);
        glVertexAttribPointer(m_normal_attribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glBufferData(GL_ARRAY_BUFFER, sizeof(m_normals), m_normals, GL_DYNAMIC_DRAW);

        s_check_gl_error();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    void destroy()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(3, m_vbo_ids);
            m_vaoId = 0;
        }

        if (m_program_id)
        {
            glDeleteProgram(m_program_id);
            m_program_id = 0;
        }
    }

    void vertex(const b3Vector3r& v, const b3Vector3r& n, const b3Color& c)
    {
        if (m_count == e_maxVertices)
            flush();

        m_vertices[m_count] = v;
        m_colors[m_count] = c;
        m_normals[m_count] = n;
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
        glUniform3fv(m_color_uniform, 1, g_light_color);
        glUniform3fv(m_light_pos_uniform, 1, g_light_position);

        glBindVertexArray(m_vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vector3r), m_vertices);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vector3r), m_normals);

        glEnable(GL_POLYGON_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, m_count);
        glDisable(GL_BLEND);
        glDisable(GL_POLYGON_SMOOTH);

        s_check_gl_error();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 512 };
    b3Vector3r m_vertices[3 * e_maxVertices];
    b3Color m_colors[4 * e_maxVertices];
    b3Vector3r m_normals[3 * e_maxVertices];
    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vbo_ids[3];
    GLuint m_program_id;
    GLint m_projection_uniform;
    GLint m_color_uniform;
    GLint m_light_pos_uniform;
    GLint m_vertex_attribute;
    GLint m_color_attribute;
    GLint m_normal_attribute;
};



void DebugDraw::draw_box(const b3EdgeIndex* edge_index, const b3FaceIndex* face_index,
                         const b3Vector3r* n, const b3Vector3r* v, const b3Color& color)
{
    for (int32 i = 0; i < 6; i++) {
        m_triangles->vertex({v[face_index[i].e1].y(), v[face_index[i].e1].x(), v[face_index[i].e1].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
        m_triangles->vertex({v[face_index[i].e2].y(), v[face_index[i].e2].x(), v[face_index[i].e2].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
        m_triangles->vertex({v[face_index[i].e3].y(), v[face_index[i].e3].x(), v[face_index[i].e3].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
        m_triangles->vertex({v[face_index[i].e3].y(), v[face_index[i].e3].x(), v[face_index[i].e3].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
        m_triangles->vertex({v[face_index[i].e4].y(), v[face_index[i].e4].x(), v[face_index[i].e4].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
        m_triangles->vertex({v[face_index[i].e1].y(), v[face_index[i].e1].x(), v[face_index[i].e1].z()},
                            {n[i].y(), n[i].z(), n[i].x()}, color);
    }

//    for (int32 i = 0; i < 12; i++) {
//        m_lines->vertex({v[edge_index[i].v1].y(), v[edge_index[i].v1].x(), v[edge_index[i].v1].z()}, b3Color::black);
//        m_lines->vertex({v[edge_index[i].v2].y(), v[edge_index[i].v2].x(), v[edge_index[i].v2].z()}, b3Color::black);
//    }
}


void DebugDraw::create()
{
    m_points = new GLRenderPoints;
    m_points->create();

    m_lines = new GLRenderLines;
    m_lines->create();

    m_triangles = new GLRenderTriangles;
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
    m_triangles = NULL;
}


