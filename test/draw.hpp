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


public:

    bool m_show_ui = true;
    GLRenderPoints* m_points = nullptr;
    GLRenderLines* m_lines = nullptr;
    gl_render_triangles* m_triangles = nullptr;

public:

    DebugDraw() = default;

    void create();

    void draw_box(const b3EdgeIndex* edge_index, const b3FaceIndex* face_index,
                  const b3Vector3r* n, const b3Vector3r* v, const b3Color& color) override;

    void flush();

    void destroy();
};


extern Camera g_camera;
extern DebugDraw g_debug_draw;
extern float g_light_color[3];
extern float g_light_position[3];


#endif //BOX3D_DRAW_HPP
