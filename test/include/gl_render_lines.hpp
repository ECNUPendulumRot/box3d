
#ifndef BOX3D_GL_RENDER_LINES_HPP
#define BOX3D_GL_RENDER_LINES_HPP



#include "glad/gl.h"
#include "GLFW/glfw3.h"
#include "gl_common.hpp"
#include "box3d.hpp"


struct Camera;


struct GLRenderLines
{
    void create();

    void destroy();

    void vertex(const b3Vec3r& v, const b3Color& c);

    void flush();

    enum { e_maxVertices = 2 * 512 };
    b3Vec3r m_vertices[e_maxVertices];
    b3Color m_colors[e_maxVertices];

    int32 m_count;

    GLuint m_vao_id;
    GLuint m_vbo_ids[2];
    GLuint m_program_id;

    GLint m_projection_uniform;
    GLint m_view_uniform;
    GLint m_model_uniform;

    GLint m_vertex_attribute;
    GLint m_color_attribute;
};


extern Camera g_camera;
extern b3Vec3r g_light_color;
extern b3Vec3r g_light_position;


#endif //BOX3D_GL_RENDER_LINES_HPP
