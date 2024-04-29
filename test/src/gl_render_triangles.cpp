
#include "gl_render_triangles.hpp"

#include "camera.hpp"

void gl_render_triangles::create() {

    const char* vs = \
			"#version 330\n"
            "uniform mat4 projection_mat;\n"
            "uniform mat4 view_mat;\n"
            "uniform mat4 model_mat;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "layout(location = 2) in vec3 v_normal;\n"
            "out vec3 f_color;\n"
            "out vec3 f_normal;\n"
            "out vec3 f_frag_pos;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color.rgb;\n"
            "	f_normal = vec3(model_mat * vec4(v_normal, 1));\n"
            "	f_frag_pos = vec3(model_mat * vec4(v_position, 1));\n"
            "	gl_Position = projection_mat * view_mat * model_mat * vec4(v_position, 1.0f);\n"
            "}\n";

    const char* fs = \
			"#version 330\n"
            "uniform vec3 light_color;\n"
            "uniform vec3 light_pos;\n"
            "uniform vec3 view_pos;\n"
            "uniform float const_attenuation;\n"
            "uniform float linear_attenuation;\n"
            "uniform float quadratic_attenuation;\n"
            "in vec3 f_color;\n"
            "in vec3 f_normal;\n"
            "in vec3 f_frag_pos;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "   float ambient_factor = 0.1f;\n"
            "   vec3 ambient = ambient_factor * light_color;\n"
            "   \n"
            "   vec3 norm = normalize(f_normal);\n"
            "   float d = length(light_pos - f_frag_pos);\n"
            "   vec3 light_dir = normalize(light_pos - f_frag_pos);\n"
            "   float attenuation = 1.0 / (const_attenuation + linear_attenuation * d + quadratic_attenuation * d * d);\n"
            "   float diff = max(dot(norm, light_dir), 0.0);\n"
            "   vec3 diffuse = diff * light_color;\n"
            "   \n"
            "	color = vec4((ambient + diffuse * attenuation) * f_color, 1.0);\n"
            "}\n";

    m_program_id = create_shader_program(vs, fs);
    m_projection_uniform = glGetUniformLocation(m_program_id, "projection_mat");
    m_view_uniform = glGetUniformLocation(m_program_id, "view_mat");
    m_model_uniform = glGetUniformLocation(m_program_id, "model_mat");
    m_color_uniform = glGetUniformLocation(m_program_id, "light_color");
    m_light_pos_uniform = glGetUniformLocation(m_program_id, "light_pos");
    m_view_pos_uniform = glGetUniformLocation(m_program_id, "view_pos");

    m_const_attenuation_uniform = glGetUniformLocation(m_program_id, "const_attenuation");
    m_linear_attenuation_uniform = glGetUniformLocation(m_program_id, "linear_attenuation");
    m_quadratic_attenuation_uniform = glGetUniformLocation(m_program_id, "quadratic_attenuation");

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

    check_gl_error();

    // Cleanup
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_count = 0;
}

void gl_render_triangles::destroy() {

    if (m_vaoId) {
        glDeleteVertexArrays(1, &m_vaoId);
        glDeleteBuffers(3, m_vbo_ids);
        m_vaoId = 0;
    }

    if (m_program_id) {
        glDeleteProgram(m_program_id);
        m_program_id = 0;
    }
}

void gl_render_triangles::vertex(const b3Vec3r &v, const b3Vec3r &n, const b3Color &c) {

    if (m_count == e_maxVertices)
        flush();

    m_vertices[m_count] = v;
    m_colors[m_count] = c;
    m_normals[m_count] = n;
    ++m_count;
}

void gl_render_triangles::flush() {

    if (m_count == 0)
        return;

    glUseProgram(m_program_id);

    float proj[16] = { 0.0f };
    g_camera.build_projection_matrix(proj);
    float view[16] = { 0.0f };
    g_camera.build_view_matrix(view);
    float model[16] = { 0.0f };
    g_camera.build_model_matrix(model);

    float view_pos[3] = {g_camera.m_position.x, g_camera.m_position.y, g_camera.m_position.z};
    glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, proj);
    glUniformMatrix4fv(m_view_uniform, 1, GL_FALSE, view);
    glUniformMatrix4fv(m_model_uniform, 1, GL_FALSE, model);
    glUniform3fv(m_color_uniform, 1, g_light_color.data());
    glUniform3fv(m_light_pos_uniform, 1, g_light_position.data());
    glUniform3fv(m_view_pos_uniform, 1, view_pos);
    glUniform1f(m_const_attenuation_uniform, m_const_attenuation);
    glUniform1f(m_linear_attenuation_uniform, m_linear_attenuation);
    glUniform1f(m_quadratic_attenuation_uniform, m_quadratic_attenuation);

    glBindVertexArray(m_vaoId);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3r), m_vertices);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[2]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3r), m_normals);

    glEnable(GL_POLYGON_SMOOTH);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_DEPTH_TEST);
    glDrawArrays(GL_TRIANGLES, 0, m_count);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_POLYGON_SMOOTH);

    check_gl_error();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);

    m_count = 0;
}
