
#ifndef BOX3D_GL_RENDER_TRIANGLES_HPP
#define BOX3D_GL_RENDER_TRIANGLES_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"

struct Camera;

struct gl_render_triangles
{
    void create();

    void destroy();

    void vertex(const b3Vec3f &v, const b3Vec3f&n, const b3Color &c);

    void face(const b3Vec3i& f, const b3Vec3f n[3], const b3Color& c);


    void flush();

    enum { e_maxVertices = 1 << 15 };

    b3Vec3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];
    b3Vec3r m_normals[e_maxVertices];
    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vbo_ids[3];
    GLuint m_program_id;

    GLint m_projection_uniform;
    GLint m_view_uniform;
    GLint m_model_uniform;

    GLint m_color_uniform;
    GLint m_light_pos_uniform;
    GLint m_view_pos_uniform;
    GLint m_vertex_attribute;
    GLint m_color_attribute;
    GLint m_normal_attribute;

    GLint m_const_attenuation_uniform;
    GLint m_linear_attenuation_uniform;
    GLint m_quadratic_attenuation_uniform;

    float m_const_attenuation = 1.0f;
    float m_linear_attenuation = 0.004f;
    float m_quadratic_attenuation = 0.000004f;
};

extern Camera g_camera;
extern b3Vec3r g_light_color;
extern b3Vec3r g_light_position;

#endif //BOX3D_GL_RENDER_TRIANGLES_HPP
