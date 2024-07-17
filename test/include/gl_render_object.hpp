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

#ifndef BOX3D_GL_RENDER_OBJECT_HPP
#define BOX3D_GL_RENDER_OBJECT_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"
#include <memory>

/**
 * @brief Forward declaration of the Camera class.
 */
struct Camera;

/**
 * @brief Forward declaration of the GLRenderTriangles class.
 */
struct gl_render_triangles;

/**
 * @brief Forward declaration of the GLRenderLines class.
 */
struct gl_render_lines;

/**
 * @brief A class for rendering 3D objects using OpenGL.
 * Manages vertex, normal, and triangle data for rendering.
 */
struct GLRenderObject {

    std::unique_ptr<b3Vec3f[]> m_vertices; ///< Array of vertex positions
    std::unique_ptr<b3Vec3i[]> m_triangles; ///< Array of triangle indices
    std::unique_ptr<b3Vec3f[]> m_normals; ///< Array of vertex normals

    int32 m_v_count = 0; ///< Number of vertices
    int32 m_t_count = 0; ///< Number of triangles

    b3Color m_color = b3Color(0.5f, 0.5f, 0.5f, 1.0f);///< Object color

    /**
     * @brief Sets the color of the object.
     * @param color The new color.
     */
    inline void set_color(const b3Color& color) {
        m_color = color;
    }
    /**
     * @brief Initializes the renderer with a cube shape.
     * @param cube Pointer to the cube shape.
     * @param xf The transformation applied to the shape.
     */
    void setup_renderer(const b3CubeShape* cube, const b3Transr& xf);

    /**
     * @brief Initializes the renderer with a plane shape.
     * @param plane Pointer to the plane shape.
     * @param xf The transformation applied to the shape.
     */
    void setup_renderer(const b3PlaneShape* plane, const b3Transr& xf);

    /**
     * @brief Initializes the renderer with a sphere shape.
     * @param sphere Pointer to the sphere shape.
     * @param xf The transformation applied to the shape.
     */
    void setup_renderer(const b3SphereShape* sphere, const b3Transr& xf);

    /**
     * @brief Renders the object using triangle renderer.
     * @param renderer Pointer to the triangle renderer.
     */
    void render_triangles(gl_render_triangles* renderer);

    /**
     * @brief Renders the object using line renderer.
     * @param renderer Pointer to the line renderer.
     */
    void render_lines(gl_render_lines* renderer);

};

/**
 * @brief Global camera object used for rendering.
 */
extern Camera g_camera;


#endif //BOX3D_GL_RENDER_OBJECT_HPP
