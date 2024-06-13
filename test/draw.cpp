#include "draw.hpp"
#include "camera.hpp"
#include "gl_render_triangles.hpp"
#include "gl_render_lines.hpp"
#include "gl_render_points.hpp"

#include "spdlog/spdlog.h"

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
        for (int32 i = 0; i < 6; i++) {
            m_triangles->vertex(v[faces[i].e1], n[i], color);
            m_triangles->vertex(v[faces[i].e2], n[i], color);
            m_triangles->vertex(v[faces[i].e3], n[i], color);
            m_triangles->vertex(v[faces[i].e3], n[i], color);
            m_triangles->vertex(v[faces[i].e4], n[i], color);
            m_triangles->vertex(v[faces[i].e1], n[i], color);
        }
    }
}


void DebugDraw::draw_plane(const b3PlaneShape* plane, const b3Transformr &xf, const b3Color &color)
{
    b3Vec3f lf = xf.position();
    b3Vec3f d_w = {float(plane->m_half_width), 0.0f, 0.0f};
    b3Vec3f d_l = {0.0f, float(plane->m_half_length), 0.0f};

    d_w = xf.rotate(d_w);
    d_l = xf.rotate(d_l);

    lf = lf - d_w - d_l;

    d_w = 2.0 * d_w / float(m_plane_segment);
    d_l = 2.0 * d_l / float(m_plane_segment);

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
        for (int32 i = 0; i < m_plane_segment; i++) {
            for (int j = 0; j < m_plane_segment; j++) {
                m_triangles->vertex(lf + i * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + (i + 1) * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + i * d_w + (j + 1) * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
                m_triangles->vertex(lf + i * d_w + j * d_l, b3Vec3r(0.0f, 0.0f, 1.0f), color);
            }
        }
    }
}


void DebugDraw::draw_sphere(const b3SphereShape* sphere, const b3Transformr &xf, const b3Color &color)
{
    // vertex position
    float x, y, z, xy;
    float radius = sphere->get_radius();
    // vertex normal
    float nx, ny, nz, lengthInv = 1.0f / radius;

    const int32 sector_count = 16;
    const int32 stack_count = 16;

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
            normals[index_n++] = b3Vec3r(nx, ny, nz);

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
        for (int32 i = 0; i < stack_count; ++i) {
            k1 = i * (sector_count + 1);
            k2 = k1 + sector_count + 1;

            for (int32 j = 0; j < sector_count; ++j, ++k1, ++k2) {
                if (i != 0) {
                    m_triangles->vertex(xf.transform(vertices[k1]), xf.transform(normals[k1]), color);
                    m_triangles->vertex(xf.transform(vertices[k2]), xf.transform(normals[k2]), color);
                    m_triangles->vertex(xf.transform(vertices[k1 + 1]), xf.transform(normals[k1 + 1]), color);
                }

                if (i != (stack_count - 1)) {
                    m_triangles->vertex(xf.transform(vertices[k1 + 1]), xf.transform(normals[k1 + 1]), color);
                    m_triangles->vertex(xf.transform(vertices[k2]), xf.transform(normals[k2]), color);
                    m_triangles->vertex(xf.transform(vertices[k2 + 1]), xf.transform(normals[k2 + 1]), color);
                }
            }
        }
    }
}


void DebugDraw::draw_cone(const b3ConeShape *cone, const b3Transformr &xf, const b3Color &color)
{
    // the tip of the cone in the world space
    b3Vec3r tip = xf.position();

    b3Vec3r axis = xf.rotation_matrix().col(2);

    b3Vec3r base_center = tip + cone->get_height() * axis;

    static const int base_point_count = 6;

    b3Vec3r base_points[base_point_count];
    b3Vec3r base_normals[base_point_count];

    real radius = cone->get_radius();

    // TODO: reduce this, we define the point count in the b3_cone_shape class.
    real theta = 0;
    real delta_theta = b3_pi * 2 / base_point_count;

    const b3Mat33r& R = xf.rotation_matrix();

    for (int i = 0; i < base_point_count; i++) {
        base_normals[i].set(cos(theta), sin(theta), 0);
        base_points[i] = base_center + radius * R * base_normals[i];

        theta += delta_theta;
    }


    // TODO: This ?
    b3Vec3r normal = {0, 0, 1};

    if (m_draw_flags & b3Draw::e_frame_only_bit) {
        // base triangle face
        m_lines->vertex(base_points[0], color);
        m_lines->vertex(base_points[base_point_count - 1], color);
        for (int i = 1; i < base_point_count; i++) {
            m_lines->vertex(base_points[i], color);
            m_lines->vertex(base_points[i - 1], color);
        }

        for (int i = 0; i < base_point_count; i++) {
            m_lines->vertex(base_center, color);
            m_lines->vertex(base_points[i], color);
        }

        // the conical surface
        for (int i = 0; i < base_point_count; i++) {
            m_lines->vertex(tip, color);
            m_lines->vertex(base_points[i], color);
        }
    } else {
        // base triangle face
        m_triangles->vertex(base_center, normal, color);
        m_triangles->vertex(base_points[0], normal, color);
        m_triangles->vertex(base_points[base_point_count - 1], normal, color);
        for (int i = 1; i < base_point_count; i++) {
            m_triangles->vertex(base_center, normal, color);
            m_triangles->vertex(base_points[i], normal, color);
            m_triangles->vertex(base_points[i - 1], normal, color);
        }

        // the conical surface
        m_triangles->vertex(tip, normal, color);
        m_triangles->vertex(base_points[0], normal, color);
        m_triangles->vertex(base_points[base_point_count - 1], normal, color);
        for (int i = 1; i < base_point_count; i++) {
            m_triangles->vertex(tip, normal, color);
            m_triangles->vertex(base_points[i], normal, color);
            m_triangles->vertex(base_points[i - 1], normal, color);
        }
    }
}


void DebugDraw::draw_point(const b3Vec3r &p, float size, const b3Color &color)
{
    m_points->vertex(p, color, size);
}


