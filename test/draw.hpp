//
// Created by sherman on 24-4-25.
//

#ifndef BOX3D_DRAW_HPP
#define BOX3D_DRAW_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "camera.hpp"
#include "box3d.hpp"


struct GLRenderPoints;
struct GLRenderLines;
struct gl_render_triangles;


class DebugDraw : public b3Draw {

    int32 m_plane_segment = 4;

    int32 m_start_index = 0;

public:

    bool m_show_ui = true;
    GLRenderPoints* m_points = nullptr;
    GLRenderLines* m_lines = nullptr;
    gl_render_triangles* m_triangles = nullptr;

public:

    DebugDraw() = default;

    void create();

    void draw_box(const b3CubeShape* cube, const b3Transr& xf, const b3Color& color) override;

    void draw_plane(const b3PlaneShape* plane, const b3Transr& xf, const b3Color& color) override;

    void draw_sphere(const b3SphereShape* sphere, const b3Transr& xf, const b3Color& color) override;

    void draw_point(const b3Vec3r& p, float size, const b3Color& color) override;

    void flush();

    void destroy();
};


extern Camera g_camera;
extern DebugDraw g_debug_draw;
extern b3Vec3f g_light_color;
extern b3Vec3f g_light_position;


#endif //BOX3D_DRAW_HPP
