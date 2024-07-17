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

#ifndef BOX3D_GL_RENDER_POINTS_HPP
#define BOX3D_GL_RENDER_POINTS_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"


struct Camera;


struct GLRenderPoints
{
    void create();

    void destroy();

    void vertex(const b3Vec3r& v, const b3Color& c, float size);

    void flush();

    enum { e_maxVertices = 1 << 15 };
    b3Vec3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];
    float m_sizes[e_maxVertices];

    int32 m_count;

    GLuint m_vao_id;
    GLuint m_vbo_ids[3];
    GLuint m_program_id;

    GLint m_projection_uniform;
    GLint m_view_uniform;
    GLint m_model_uniform;

    GLint m_vertex_attribute;
    GLint m_color_attribute;
    GLint m_size_attribute;
};


extern Camera g_camera;
extern b3Vec3f g_light_color;
extern b3Vec3f g_light_position;


#endif //BOX3D_GL_RENDER_POINTS_HPP
