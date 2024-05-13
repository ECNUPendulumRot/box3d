
#include "geometry/b3_cube_shape.hpp"

#include "common/b3_common.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_aabb.hpp"

b3CubeShape::b3CubeShape() {
    m_radius = b3_polygon_radius;
    m_type = e_cube;
}


void b3CubeShape::set_as_box(double hx, double hy, double hz) {
    b3_assert(hx > 0.0f);
    b3_assert(hy > 0.0f);
    b3_assert(hz > 0.0f);

    m_centroid.set_zero();

    // TODO: check the order of the axis
    m_vertices[0].set(hx, -hy, -hz);
    m_vertices[1].set(hx, hy, -hz);
    m_vertices[2].set(-hx, hy, -hz);
    m_vertices[3].set(-hx, -hy, -hz);

    m_vertices[4].set(hx, -hy, hz);
    m_vertices[5].set(hx, hy, hz);
    m_vertices[6].set(-hx, hy, hz);
    m_vertices[7].set(-hx, -hy, hz);

    m_normals[0].set(0.0f, 0.0f, -1.0f);
    m_normals[1].set(0.0f, 1.0f, 0.0f);
    m_normals[2].set(0.0f, 0.0f, 1.0f);
    m_normals[3].set(0.0f, -1.0f, 0.0f);
    m_normals[4].set(1.0f, 0.0f, 0.0f);
    m_normals[5].set(-1.0f, 0.0f, 0.0f);

    m_faces[0] = { 0, 3, 2, 1 };
    m_faces[1] = { 1, 2, 6, 5 };
    m_faces[2] = { 4, 5, 6, 7 };
    m_faces[3] = { 0, 4, 7, 3 };
    m_faces[4] = { 0, 1, 5, 4 };
    m_faces[5] = { 2, 3, 7, 6 };

    m_edges[0] = { 0, 1 };
    m_edges[1] = { 1, 2 };
    m_edges[2] = { 2, 3 };
    m_edges[3] = { 3, 0 };
    m_edges[4] = { 4, 5 };
    m_edges[5] = { 5, 6 };
    m_edges[6] = { 6, 7 };
    m_edges[7] = { 7, 4 };
    m_edges[8] = { 0, 4 };
    m_edges[9] = { 1, 5 };
    m_edges[10] = { 2, 6 };
    m_edges[11] = { 3, 7 };

    m_xyz.set(2 * hx, 2 * hy, 2 * hz);
    m_h_xyz.set(hx, hy, hz);

}


void b3CubeShape::get_bound_aabb(b3AABB *aabb, const b3Transformr &xf, int32 childIndex) const {
    b3_NOT_USED(childIndex);

    b3Vec3r min = xf.transform(m_vertices[0]);
    b3Vec3r max = min;

    for (int32 i = 1; i < 8; ++i) {
  	    b3Vec3r v = xf.transform(m_vertices[i]);
  	    min = b3_min_coeff(min, v);
  	    max = b3_max_coeff(max, v);
    }

    b3Vec3r r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}


void b3CubeShape::compute_mass_properties(b3MassProperty &mass_data, real density) const {
    mass_data.m_center = m_centroid;

    mass_data.m_volume = m_xyz.x * m_xyz.y * m_xyz.z;

    mass_data.m_mass = density * mass_data.m_volume;

    real x2 = m_xyz.x * m_xyz.x;
    real y2 = m_xyz.y * m_xyz.y;
    real z2 = m_xyz.z * m_xyz.z;
    real ot = 1.0 / 12.0;

    real i11 = ot * mass_data.m_mass * (y2 + z2);
    real i22 = ot * mass_data.m_mass * (x2 + z2);
    real i33 = ot * mass_data.m_mass * (x2 + y2);

    mass_data.m_Inertia = b3Mat33r::zero();
    mass_data.m_Inertia(0, 0) = i11;
    mass_data.m_Inertia(1, 1) = i22;
    mass_data.m_Inertia(2, 2) = i33;
}


void b3CubeShape::init_view_data() {
    m_view_data.m_vertex_count = 8;
    void *mem = m_block_allocator->allocate(m_view_data.m_vertex_count * 3 * sizeof(double));
    m_view_data.m_V = new (mem) double;

    m_view_data.m_face_count = 12;
    mem = m_block_allocator->allocate(m_view_data.m_face_count * 3 * sizeof(int));
    m_view_data.m_F = new (mem) int;
    m_view_data.set_face_row(0, { 0, 1, 2 });
    m_view_data.set_face_row(1, { 0, 2, 3 });
    m_view_data.set_face_row(2, { 4, 5, 6 });
    m_view_data.set_face_row(3, { 4, 6, 7 });
    m_view_data.set_face_row(4, { 0, 1, 5 });
    m_view_data.set_face_row(5, { 0, 5, 4 });
    m_view_data.set_face_row(6, { 1, 2, 6 });
    m_view_data.set_face_row(7, { 1, 6, 5 });
    m_view_data.set_face_row(8, { 2, 3, 7 });
    m_view_data.set_face_row(9, { 2, 7, 6 });
    m_view_data.set_face_row(10, { 3, 0, 4 });
    m_view_data.set_face_row(11, { 3, 4, 7 });
}


void b3CubeShape::setup_view_data(const b3Transformr &xf) {
    int index = 0;
    for (const b3Vec3r &vertex : m_vertices) {
  	    b3Vec3r v = xf.transform(vertex);
  	    m_view_data.m_V[index++] = v.x;
  	    m_view_data.m_V[index++] = v.y;
  	    m_view_data.m_V[index++] = v.z;
    }
}


b3Shape *b3CubeShape::clone() const {
    void *mem = m_block_allocator->allocate(sizeof(b3CubeShape));
    auto *clone = new (mem) b3CubeShape;
    *clone = *this;
    return clone;
}



