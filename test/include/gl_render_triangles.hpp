
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

    void vertex(const b3Vector3r& v, const b3Vector3r& n, const b3Color& c);

    void flush();

    enum { e_maxVertices = 512 };
    b3Vector3r m_vertices[3 * e_maxVertices];
    b3Color m_colors[4 * e_maxVertices];
    b3Vector3r m_normals[3 * e_maxVertices];
    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vbo_ids[3];
    GLuint m_program_id;

    GLint m_projection_uniform;
    GLint m_view_uniform;

    GLint m_color_uniform;
    GLint m_light_pos_uniform;
    GLint m_view_pos_uniform;
    GLint m_vertex_attribute;
    GLint m_color_attribute;
    GLint m_normal_attribute;
};

extern Camera g_camera;
extern float g_light_color[3];
extern float g_light_position[3];

#endif //BOX3D_GL_RENDER_TRIANGLES_HPP
