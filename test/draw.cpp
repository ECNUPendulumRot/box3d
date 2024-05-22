
#include "draw.hpp"
#include "camera.hpp"
#include "gl_render_triangles.hpp"
#include "gl_render_lines.hpp"
#include "gl_render_points.hpp"


Camera g_camera;
DebugDraw g_debug_draw;
b3Vec3f g_light_color = {1.0f, 1.0f, 1.0f};
b3Vec3f g_light_position = {20.0f, 20.0f, 20.0f};


void DebugDraw::create()
{
    m_points = new GLRenderPoints;
    m_points->create();

    m_lines = new GLRenderLines;
    m_lines->create();

    m_triangles = new gl_render_triangles;
    m_triangles->create();
}


void DebugDraw::flush()
{
    m_start_index = 0;
    m_points->flush();
    m_lines->flush();
    m_triangles->flush();
}


void DebugDraw::destroy()
{
    m_points->destroy();
    delete m_points;
    m_points = nullptr;

    m_lines->destroy();
    delete m_lines;
    m_lines = nullptr;

    m_triangles->destroy();
    delete m_triangles;
    m_triangles = nullptr;
}


void DebugDraw::draw_box(const b3CubeShape* cube, const b3Transformr& xf, const b3Color& color)
{
    b3Vec3f v[8], n[6];
    for (int32 i = 0; i < 8; ++i) {
        v[i] = xf.transform(cube->m_vertices[i]);
    }


    for (int32 i = 0; i < 6; ++i)
        n[i] = xf.rotate(cube->m_normals[i]);

    const b3FaceIndex* faces = cube->m_faces;
    const b3EdgeIndex* edges = cube->m_edges;
    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        for (int32 i = 0; i < 12; i++) {
            m_lines->vertex(v[edges[i].v1], color);
            m_lines->vertex(v[edges[i].v2], color);
        }
    } else {

        for (int32 i = 0; i < 8; i++) {
            m_triangles->vertex(v[i]);
        }

        for (int32 i = 0; i < 6; i++) {
            b3Vec3r fn[3] = {n[i], n[i], n[i]};
            m_triangles->face({faces[i].e1 + m_start_index, faces[i].e2 + m_start_index, faces[i].e3 + m_start_index}, fn, color);
            m_triangles->face({faces[i].e3 + m_start_index, faces[i].e4 + m_start_index, faces[i].e1 + m_start_index}, fn, color);
        }
        m_start_index += 8;
    }
}


void DebugDraw::draw_plane(const b3PlaneShape* plane, const b3Transformr &xf, const b3Color &color)
{
    b3Vec3f lf = xf.position();
    b3Vec3f h_w = {float(plane->m_half_width), 0.0f, 0.0f};
    b3Vec3f h_l = {0.0f, float(plane->m_half_length), 0.0f};

    h_w = xf.rotate(h_w);
    h_l = xf.rotate(h_l);

    lf = lf - h_w - h_l;

    b3Vec3f d_w = 2.0 * h_w / float(m_plane_segment);
    b3Vec3f d_l = 2.0 * h_l / float(m_plane_segment);

    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        for (int32 i = 0; i < m_plane_segment; i++) {
            for (int32 j  = 0; j < m_plane_segment; j++) {

                m_lines->vertex(lf + i * d_w + j * d_l, color);
                m_lines->vertex(lf + (i + 1) * d_w + j * d_l, color);

                m_lines->vertex(lf + (i + 1) * d_w + j * d_l, color);
                m_lines->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, color);

                m_lines->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, color);
                m_lines->vertex(lf + i * d_w + (j + 1) * d_l, color);

                m_lines->vertex(lf + i * d_w + (j + 1) * d_l, color);
                m_lines->vertex(lf + i * d_w + j * d_l, color);
            }

        }
    } else {
        m_triangles->vertex(lf);
        m_triangles->vertex(lf + 2 * h_w);
        m_triangles->vertex(lf + 2 * h_w + 2 * h_l);
        m_triangles->vertex(lf + 2 * h_l);

        b3Vec3i tr1 = {m_start_index, m_start_index + 1, m_start_index + 2};
        b3Vec3i tr2 = {m_start_index + 2, m_start_index + 3, m_start_index};

        b3Vec3f n[3] = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
        m_triangles->face(tr1, n, color);
        m_triangles->face(tr2, n, color);

        m_start_index += 4;
    }
}


void DebugDraw::draw_sphere(const b3SphereShape* sphere, const b3Transformr &xf, const b3Color &color)
{
    // vertex position
    float x, y, z, xy;
    float radius = sphere->get_radius();
    // vertex normal
    float nx, ny, nz, lengthInv = 1.0f / radius;

    const int32 sector_count = 20;
    const int32 stack_count = 20;

    b3Vec3r vertices[(sector_count + 1) * (stack_count + 1)];
    b3Vec3r normals[(sector_count + 1) * (stack_count + 1)];

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
            vertices[index_v++] = b3Vec3r(x, y, z);

            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;
            normals[index_n++] = xf.rotate(b3Vec3r(nx, ny, nz));
        }
    }


    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        int32 k1, k2;
        for (int32 i = 0; i < stack_count; ++i) {
            k1 = i * (sector_count + 1);
            k2 = k1 + sector_count + 1;

            for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
                if (i != 0) {
                    m_lines->vertex(xf.transform(vertices[k1]), color);
                    m_lines->vertex(xf.transform(vertices[k2]), color);
                    m_lines->vertex(xf.transform(vertices[k1 + 1]), color);
                }

                if (i != (stack_count - 1)) {
                    m_lines->vertex(xf.transform(vertices[k1 + 1]), color);
                    m_lines->vertex(xf.transform(vertices[k2]), color);
                    m_lines->vertex(xf.transform(vertices[k2 + 1]), color);
                }
            }
        }

    } else {
        int32 k1, k2;

        for (const auto & v : vertices) {
            m_triangles->vertex(xf.transform(v));
        }

        for (int32 i = 0; i < stack_count; ++i) {
            k1 = i * (sector_count + 1);
            k2 = k1 + sector_count + 1;

            for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
                if (i != 0) {
                    b3Vec3r fn[3] = {normals[k1], normals[k2], normals[k1 + 1]};
                    m_triangles->face({m_start_index + k1, m_start_index + k2, m_start_index + k1 + 1}, fn, color);
                }

                if (i != (stack_count - 1)) {
                    b3Vec3r fn[3] = {normals[k1 + 1], normals[k2], normals[k2 + 1]};
                    m_triangles->face({m_start_index + k1 + 1, m_start_index + k2, m_start_index + k2 + 1}, fn, color);
                }
            }
        }
        m_start_index += (sector_count + 1) * (stack_count + 1);
    }
}


void DebugDraw::draw_point(const b3Vec3r &p, float size, const b3Color &color)
{
    m_points->vertex(p, color, size);
}


