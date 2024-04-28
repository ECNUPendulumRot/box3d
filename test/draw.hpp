//
// Created by sherman on 24-4-25.
//

#ifndef BOX3D_DRAW_HPP
#define BOX3D_DRAW_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"

#include "box3d.hpp"


struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderTriangles;


struct Camera
{
    Camera() {
        m_width = 1280;
        m_height = 800;

        reset_view();
    }

    void build_projection_matrix(float* m) const ;

    void reset_view() {
        m_zoom = 1.0;
        m_lookat = {0, 0, 0};
        m_position = {-2, 2, 7};
    }

    real m_zoom;
    int32 m_width;
    int32 m_height;

    b3Vector3r m_position;
    b3Vector3r m_lookat;
};


class DebugDraw : public b3Draw {


public:

    bool m_show_ui = true;
    GLRenderPoints* m_points = nullptr;
    GLRenderLines* m_lines = nullptr;
    GLRenderTriangles* m_triangles = nullptr;

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

#endif //BOX3D_DRAW_HPP
