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

// history
//
// author           date                description
// ----------------------------------------------------------------------------
// sherman          2024-4-25           created


#ifndef BOX3D_DRAW_HPP
#define BOX3D_DRAW_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "camera.hpp"
#include "box3d.hpp"

/////////// Forward Declaration ///////////

struct GLRenderPoints;
struct GLRenderLines;
struct gl_render_triangles;

//////////////////////////////////////////


/**
 * @brief This class holds callbacks you can implement to draw a Box3D world
 * that are invoked inside b3World::Step.
*/
class DebugDraw : public b3Draw {

    int32 m_plane_segment = 4;

    int32 m_start_index = 0;

public:

    bool m_show_ui = true;
    GLRenderPoints* m_points = nullptr;
    GLRenderLines* m_lines = nullptr;
    gl_render_triangles* m_triangles = nullptr;

public:

    /**
    * @brief Default constructor
    */
    DebugDraw() = default;
    
    /**
    * @brief Create the predefined shaders for rendering.
    */
    void create();

    /**
    * @brief Draw a box.
    * @param[in] cube   the given cube shape
    * @param[in] xf     the given geometric transformations (translation and rotation)
    * @param[in] color  the current color for rendering
    */
    void draw_box(const b3CubeShape* cube, const b3Transr& xf, const b3Color& color) override;

    /**
    * @brief Draw a plane.
    * @param[in] plane  the given plane shape
    * @param[in] xf     the given geometric transformations (translation and rotation)
    * @param[in] color  the current color for rendering 
    */
    void draw_plane(const b3PlaneShape* plane, const b3Transr& xf, const b3Color& color) override;

    /**
    * @brief Draw a sphere.
    * @param[in] sphere the given sphere shape
    * @param[in] xf     the given geometric transformations (translation and rotation)
    * @param[in] color  the current color for rendering 
    */
    void draw_sphere(const b3SphereShape* sphere, const b3Transr& xf, const b3Color& color) override;

    /**
    * @brief Draw a point.
    * @param[in] p      the given point represented using a vector in 3D
    * @param[in] size   specifies the rasterized diameter of a point
    * @param[in] color  the current color for rendering 
    */
    void draw_point(const b3Vec3r& p, float size, const b3Color& color) override;


    void draw_line(const b3Vec3r& p1, const b3Vec3r& p2, const b3Color& color) override;
    /**
    * @brief Force execution of OpenGL functions 
    * 
    */
    void flush();

    /**
    * @brief Destroy the generated shaders.
    */
    void destroy();
};


extern Camera g_camera;
extern DebugDraw g_debug_draw;
extern b3Vec3f g_light_color;
extern b3Vec3f g_light_position;


#endif //BOX3D_DRAW_HPP
