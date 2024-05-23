
#ifndef BOX3D_GL_RENDER_OBJECT_HPP
#define BOX3D_GL_RENDER_OBJECT_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"
#include <memory>


struct Camera;
struct gl_render_triangles;
struct gl_render_lines;

struct GLRenderObject {

    std::unique_ptr<b3Vec3f[]> m_vertices;
    std::unique_ptr<b3Vec3i[]> m_triangles;
    std::unique_ptr<b3Vec3f[]> m_normals;

    int32 m_v_count = 0;
    int32 m_t_count = 0;

    b3Color m_color = b3Color(0.5f, 0.5f, 0.5f, 1.0f);

    inline void set_color(const b3Color& color) {
        m_color = color;
    }

    void setup_renderer(const b3CubeShape* cube, const b3Transformr& xf);

    void setup_renderer(const b3PlaneShape* plane, const b3Transformr& xf);

    void setup_renderer(const b3SphereShape* sphere, const b3Transformr& xf);

    void render_triangles(gl_render_triangles* renderer);

    void render_lines(gl_render_lines* renderer);

};


extern Camera g_camera;


#endif //BOX3D_GL_RENDER_OBJECT_HPP
