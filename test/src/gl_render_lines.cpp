// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "gl_render_lines.hpp"

#include "camera.hpp"

/**
 * @brief Creates and initializes the GLRenderLines object.
 */
void GLRenderLines::create()
{
    const char* vs = \
			"#version 330\n"
            "uniform mat4 projection_mat;\n"
            "uniform mat4 view_mat;\n"
            "uniform mat4 model_mat;\n"
            "layout(location = 0) in vec3 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projection_mat * view_mat * model_mat * vec4(v_position, 1.0f);\n"
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
    m_projection_uniform = glGetUniformLocation(m_program_id, "projection_mat");
    m_view_uniform = glGetUniformLocation(m_program_id, "view_mat");
    m_model_uniform = glGetUniformLocation(m_program_id, "model_mat");
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

/**
 * @brief Destroys the GLRenderLines object and releases resources.
 */
void GLRenderLines::destroy()
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

/**
 * @brief Adds a vertex and color to the render list.
 * 
 * @param v The position of the vertex.
 * @param c The color of the vertex.
 */
void GLRenderLines::vertex(const b3Vec3f &v, const b3Color &c)
{
    if (m_count == e_maxVertices)
        flush();

    m_vertices[m_count] = v;
    m_colors[m_count] = c;
    ++m_count;
}

/**
 * @brief Flushes the render list and draws the lines.
 */
void GLRenderLines::flush()
{
    if (m_count == 0)
        return;

    glUseProgram(m_program_id);

    float proj[16] = { 0.0f };
    g_camera.build_projection_matrix(proj);
    float view[16] = { 0.0f };
    g_camera.build_view_matrix(view);
    const float* model = g_camera.m_model_matrix;


    glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, proj);
    glUniformMatrix4fv(m_view_uniform, 1, GL_FALSE, view);
    glUniformMatrix4fv(m_model_uniform, 1, GL_FALSE, model);

    glBindVertexArray(m_vao_id);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_ids[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3r), m_vertices);

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
