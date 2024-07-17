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

#ifndef BOX3D_GL_RENDER_TRIANGLES_HPP
#define BOX3D_GL_RENDER_TRIANGLES_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"

struct Camera;

/**
 * @brief A class for rendering 3D triangles using OpenGL.
 * Manages vertex, normal, and color data for rendering.
 */
struct gl_render_triangles
{
    /**
     * @brief Creates and initializes the OpenGL buffers and program.
     */
    void create();

    /**
     * @brief Destroys the OpenGL buffers and program.
     */
    void destroy();

    /**
     * @brief Adds a vertex with its normal and color to the buffer.
     * @param v The position of the vertex.
     * @param n The normal of the vertex.
     * @param c The color of the vertex.
     */
    void vertex(const b3Vec3f &v, const b3Vec3f&n, const b3Color &c);
    
    /**
     * @brief Adds a vertex with its normal and color to the buffer.
     * @param v The position of the vertex.
     * @param n The normal of the vertex.
     * @param c The color of the vertex.
     */
    void face(const b3Vec3i& f, const b3Vec3f n[3], const b3Color& c);

    /**
     * @brief Renders the buffered triangles and clears the buffer.
     */
    void flush();

    enum { e_maxVertices = 1 << 17 };///< Maximum number of vertices

    b3Vec3r m_vertices[e_maxVertices];///< Array of vertex positions
    b3Color m_colors[e_maxVertices]; ///< Array of vertex colors
    b3Vec3r m_normals[e_maxVertices]; ///< Array of vertex normals
    int32 m_count; ///< Number of vertices in the buffer

    GLuint m_vaoId; ///< Vertex Array Object ID
    GLuint m_vbo_ids[3]; ///< Vertex Buffer Object IDs
    GLuint m_program_id; ///< Shader program ID

    GLint m_projection_uniform; ///< Projection matrix uniform location
    GLint m_view_uniform; ///< View matrix uniform location
    GLint m_model_uniform; ///< Model matrix uniform location

    GLint m_color_uniform; ///< Color uniform location
    GLint m_light_pos_uniform; ///< Light position uniform location
    GLint m_view_pos_uniform; ///< View position uniform location
    GLint m_vertex_attribute; ///< Vertex attribute location
    GLint m_color_attribute; ///< Color attribute location
    GLint m_normal_attribute; ///< Normal attribute location

    GLint m_const_attenuation_uniform; ///< Constant attenuation uniform location
    GLint m_linear_attenuation_uniform; ///< Linear attenuation uniform location
    GLint m_quadratic_attenuation_uniform; ///< Quadratic attenuation uniform location

    float m_const_attenuation = 1.0f; ///< Constant attenuation factor
    float m_linear_attenuation = 0.004f; ///< Linear attenuation factor
    float m_quadratic_attenuation = 0.000004f; ///< Quadratic attenuation factor
};

/**
 * @brief Global camera object used for rendering.
 */
extern Camera g_camera;

/**
 * @brief Global light color.
 */
extern b3Vec3r g_light_color;

/**
 * @brief Global light position.
 */
extern b3Vec3r g_light_position;

#endif //BOX3D_GL_RENDER_TRIANGLES_HPP
