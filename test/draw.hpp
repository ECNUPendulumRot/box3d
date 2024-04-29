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

public:

    bool m_show_ui = true;
    GLRenderPoints* m_points = nullptr;
    GLRenderLines* m_lines = nullptr;
    gl_render_triangles* m_triangles = nullptr;

public:

    DebugDraw() = default;

    void create();

    void draw_box(const b3EdgeIndex* edge_index, const b3FaceIndex* face_index,
                  const b3Vec3r* n, const b3Vec3r* v, const b3Color& color) override;

    void draw_plane(const b3Transformr& xf, const real& hf_w, const real& hf_l, const b3Color& color) override;

    void flush();

    void destroy();
};


extern Camera g_camera;
extern DebugDraw g_debug_draw;
extern b3Vec3r g_light_color;
extern b3Vec3r g_light_position;


#endif //BOX3D_DRAW_HPP
