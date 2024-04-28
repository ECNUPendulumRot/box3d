
#include "draw.hpp"

Camera g_camera;
DebugDraw g_debug_draw;


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


void Camera::build_projection_matrix(float* m)
{
    float w = float(m_width);
    float h = float(m_height);
    float ratio = w / h;

    float sd_h = 1.0f;

    float e_x = ratio * sd_h * m_zoom;
    float e_y = sd_h * m_zoom;

    float r = e_x;
    float t = e_y;

//    b3Vector3r d = (m_position - m_lookat).normalized();
//    b3Vector3r u(0, 1, 0);
//    b3Vector3r s = u.cross(d).normalized();
//    u = d.cross(s);

b3Vector3r u = {0, 1, 0};
    auto f = (m_lookat - m_position).normalized();
    auto s = u.cross(f).normalized();
    u = s.cross(f).normalized();

    m[0] = s.x();
    m[1] = s.y();
    m[2] = s.z();
    m[4] = u.x();
    m[5] = u.y();
    m[6] = u.z();
    m[8] =-f.x();
    m[9] =-f.y();
    m[10] =-f.z();
    m[3] =-s.dot(m_position);
    m[7] =-u.dot(m_position);
    m[11] = f.dot(m_position);


//    m[0] = s.x() / r;
//    m[1] = u.x() / t;
//    m[2] = -d.x();
//    m[3] = -d.x();
//
//    m[4] = s.y() / r;
//    m[5] = u.y() / t;
//    m[6] = -d.y();
//    m[7] = -d.y();
//
//    m[8] = -s.z() / r;
//    m[9] = -u.z() / t;
//    m[10] = -d.z();
//    m[11] = -d.z();
//
//    m[12] = -s.dot(m_position) / r;
//    m[13] = -u.dot(m_position) / t;
//    m[14] = d.dot(m_position) + 2.0f / m_zoom;
//    m[15] = d.dot(m_position);
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



void DebugDraw::draw_box(const b3Vector3r& centroid, const b3Vector3r& hf, const b3Color& color)
{
    b3Vector3r c_gl(centroid.y(), centroid.z(), centroid.x());
    b3Vector3r hf_gl(hf.y(), hf.z(), hf.x());
    m_points->vertex(c_gl + b3Vector3r(hf_gl.x(), hf_gl.y(), hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(hf_gl.x(), -hf_gl.y(), hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(-hf_gl.x(), -hf_gl.y(), hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(-hf_gl.x(), hf_gl.y(), hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(hf_gl.x(), hf_gl.y(), -hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(hf_gl.x(), -hf_gl.y(), -hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(-hf_gl.x(), -hf_gl.y(), -hf_gl.z()), color, 5.0f);
    m_points->vertex(c_gl + b3Vector3r(-hf_gl.x(), hf_gl.y(), -hf_gl.z()), color, 5.0f);
}


void DebugDraw::create()
{
    m_points = new GLRenderPoints;
    m_points->create();

}

void DebugDraw::flush()
{
    m_points->flush();
}


