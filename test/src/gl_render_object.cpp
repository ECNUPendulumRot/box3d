
#include "gl_render_object.hpp"
#include "camera.hpp"
#include "gl_render_triangles.hpp"


void GLRenderObject::setup_renderer(const b3CubeShape *cube, const b3Transformr &xf)
{
    m_v_count = 8;
    m_t_count = 2 * 6;

    m_vertices = std::make_unique<b3Vec3f[]>(m_v_count);
    m_normals = std::make_unique<b3Vec3f[]>(m_v_count);
    m_triangles = std::make_unique<b3Vec3i[]>(m_t_count);

    b3Vec3f v[8], n[6];
    for (int32 i = 0; i < 8; ++i) {
        v[i] = xf.transform(cube->m_vertices[i]);
    }


    for (int32 i = 0; i < 6; ++i)
        n[i] = xf.rotate(cube->m_normals[i]);

    for (int32 i = 0; i < 8; i++) {
        m_vertices[i] = v[i];
    }

    const b3FaceIndex* faces = cube->m_faces;

    for (int32 i = 0; i < 6; i++) {
        m_normals[faces[i].e1] = n[i];
        m_normals[faces[i].e2] = n[i];
        m_normals[faces[i].e3] = n[i];
        m_normals[faces[i].e4] = n[i];

        m_triangles[2 * i] = {faces[i].e1, faces[i].e2, faces[i].e3};
        m_triangles[2 * i + 1] = {faces[i].e3, faces[i].e4, faces[i].e1};
    }
}


void GLRenderObject::setup_renderer(const b3PlaneShape *plane, const b3Transformr &xf)
{
    m_v_count = 4;
    m_t_count = 2;

    m_vertices = std::make_unique<b3Vec3f[]>(m_v_count);
    m_normals = std::make_unique<b3Vec3f[]>(m_v_count);
    m_triangles = std::make_unique<b3Vec3i[]>(m_t_count);

    b3Vec3f lf = xf.position();
    b3Vec3f h_w = {float(plane->m_half_width), 0.0f, 0.0f};
    b3Vec3f h_l = {0.0f, float(plane->m_half_length), 0.0f};

    h_w = xf.rotate(h_w);
    h_l = xf.rotate(h_l);

    lf = lf - h_w - h_l;


    m_vertices[0] = lf;
    m_vertices[1] = lf + 2.0f * h_w;
    m_vertices[2] = lf + 2.0f * h_w + 2.0f * h_l;
    m_vertices[3] = lf + 2.0f * h_l;

    m_triangles[0] = {0, 1, 2};
    m_triangles[1] = {2, 3, 0};

    m_normals[0] = xf.rotate({0.0f, 0.0f, 1.0f});
    m_normals[1] = xf.rotate({0.0f, 0.0f, 1.0f});
    m_normals[2] = xf.rotate({0.0f, 0.0f, 1.0f});
    m_normals[3] = xf.rotate({0.0f, 0.0f, 1.0f});
}


void GLRenderObject::setup_renderer(const b3SphereShape *sphere, const b3Transformr &xf)
{
    const int32 sector_count = 20;
    const int32 stack_count = 20;

    m_v_count = (sector_count + 1) * (stack_count + 1);
    m_t_count = sector_count * stack_count;

    m_vertices = std::make_unique<b3Vec3f[]>(m_v_count);
    m_normals = std::make_unique<b3Vec3f[]>(m_v_count);
    m_triangles = std::make_unique<b3Vec3i[]>(m_t_count);

    // vertex position
    float x, y, z, xy;
    float radius = sphere->get_radius();
    // vertex normal
    float nx, ny, nz, lengthInv = 1.0f / radius;

    float sector_step = 2.0f * b3_pi / sector_count;
    float stack_step = b3_pi / stack_count;
    float sector_angle, stack_angle;

    int32 index_v = 0;
    int32 index_n = 0;
    for (int32 i = 0; i <= stack_count; ++i) {
        stack_angle = b3_pi / 2.0f - i * stack_step;
        xy = radius * cosf(stack_angle);

        z = radius * sinf(stack_angle);

        for (int32 j = 0; j <= sector_count; ++j) {
            sector_angle = j * sector_step;

            x = xy * cosf(sector_angle);
            y = xy * sinf(sector_angle);
            m_vertices[index_v++] = b3Vec3r(x, y, z);

            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;
            m_normals[index_n++] = xf.rotate(b3Vec3r(nx, ny, nz));
        }
    }

    int32 k1, k2;
    for (int32 i = 0; i < stack_count; ++i) {
        k1 = i * (sector_count + 1);
        k2 = k1 + sector_count + 1;

        for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
            if (i != 0) {
                m_triangles[i * sector_count + j] = {k1, k2, k1 + 1};
            }

            if (i != (stack_count - 1)) {
                m_triangles[i * sector_count + j] = {k1 + 1, k2, k2 + 1};
            }
        }
    }
}


void GLRenderObject::render_triangles(gl_render_triangles *renderer)
{
    for (int32 i = 0; i < m_t_count; i++) {
        b3Vec3i t = m_triangles[i];
        b3Vec3f n[3] = {m_normals[t[0]], m_normals[t[1]], m_normals[t[2]]};
        b3Color c = m_color;
        renderer->vertex(m_vertices[t[0]], m_normals[t[0]], c);
        renderer->vertex(m_vertices[t[1]], m_normals[t[0]], c);
        renderer->vertex(m_vertices[t[2]], m_normals[t[0]], c);
    }
}


void GLRenderObject::render_lines(gl_render_lines *renderer)
{

}

