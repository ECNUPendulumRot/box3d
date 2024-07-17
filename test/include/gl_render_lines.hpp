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

#ifndef BOX3D_GL_RENDER_LINES_HPP
#define BOX3D_GL_RENDER_LINES_HPP



#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"

/**
 * @brief Forward declaration of the Camera class.
 */
struct Camera;

/**
 * @brief A class for rendering lines using OpenGL.
 * Manages OpenGL resources and line data.
 */
struct GLRenderLines
{
    /**
     * @brief Creates OpenGL resources for rendering lines.
     * Initializes VAO, VBOs, and shader program.
     */
    void create();

    /**
     * @brief Destroys OpenGL resources used for rendering lines.
     * Cleans up VAO, VBOs, and shader program.
     */
    void destroy();

    /**
     * @brief Adds a vertex and color to the line buffer.
     * @param v The vertex position.
     * @param c The color of the vertex.
     */
    void vertex(const b3Vec3r& v, const b3Color& c);

    /**
     * @brief Renders all lines in the buffer.
     * Uploads vertex and color data to the GPU and issues draw calls.
     */
    void flush();

    // Maximum number of vertices that can be stored in the buffer.
    enum { e_maxVertices = 2 * 512 };

    // Arrays to hold vertex positions and colors.
    b3Vec3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];

    // Current number of vertices.
    int32 m_count;

    // OpenGL objects for rendering.
    GLuint m_vao_id;     ///< Vertex Array Object ID
    GLuint m_vbo_ids[2]; ///< Vertex Buffer Object IDs (position and color)
    GLuint m_program_id; ///< Shader program ID

    // Shader uniform locations.
    GLint m_projection_uniform; ///< Location of the projection matrix uniform
    GLint m_view_uniform;       ///< Location of the view matrix uniform
    GLint m_model_uniform;      ///< Location of the model matrix uniform

    // Shader attribute locations.
    GLint m_vertex_attribute; ///< Location of the vertex attribute
    GLint m_color_attribute;  ///< Location of the color attribute
};

/**
 * @brief Global camera object used for rendering.
 */
extern Camera g_camera;

/**
 * @brief Global light color for rendering.
 */
extern b3Vec3r g_light_color;

/**
 * @brief Global light position for rendering.
 */
extern b3Vec3r g_light_position;


#endif //BOX3D_GL_RENDER_LINES_HPP
