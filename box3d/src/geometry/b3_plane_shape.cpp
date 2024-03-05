
#include "geometry/b3_plane_shape.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_aabb.hpp"

b3PlaneShape::b3PlaneShape() {
    m_radius = b3_polygon_radius;
    m_type = e_plane;
}


void b3PlaneShape::set_as_plane(double length, double width) {
    m_half_length = length / 2.0;
    m_half_width = width / 2.0;

}


void b3PlaneShape::get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 child_index) const {
    b3_NOT_USED(child_index);

    b3Vector3d min;
    b3Vector3d max;

    b3Vector3d vertices[4];

    vertices[0].set(-m_half_width,-m_half_length, 0);
    vertices[1].set(-m_half_width, m_half_length, 0);
    vertices[2].set(m_half_width,  m_half_length, 0);
    vertices[3].set(m_half_width, -m_half_length, 0);

    for (int32 i = 0; i < 4; i++) {
        b3Vector3d v = xf.transform(vertices[i]);
        min = b3_min_coeff(min, v);
        max = b3_max_coeff(max, v);
    }

    b3Vector3d r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}


void b3PlaneShape::compute_mass_properties(b3MassProperty &mass_data, double density) const {
    mass_data.m_center = b3Vector3d::zero();
    mass_data.m_volume = 0;
    mass_data.m_mass = 0;
    mass_data.m_Inertia = b3Matrix3d::zero();
}


b3Shape* b3PlaneShape::clone() const {
    void* mem = m_block_allocator->allocate(sizeof(b3PlaneShape));
    auto* clone = new (mem) b3PlaneShape;
    *clone = *this;
    return clone;
}

int b3PlaneShape::segment_count = 20;

void b3PlaneShape::init_view_data() {
    m_view_data.m_vertex_count = 4 * (segment_count + 1);
    void* mem = m_block_allocator->allocate(m_view_data.m_vertex_count * 3 * sizeof(double));
    m_view_data.m_V = new (mem) double;

    m_view_data.m_edge_count = 2 * (segment_count + 1);
    mem = m_block_allocator->allocate(m_view_data.m_edge_count * 2 * sizeof(int32));
    m_view_data.m_E = new (mem) int32;

    for(int32 i = 0; i < m_view_data.m_edge_count * 2; ++i) {
        m_view_data.m_E[i] = i;
    }
}


void b3PlaneShape::setup_view_data(const b3TransformD &xf) {

    b3Vector3d vertices[4];

    vertices[0].set(-m_half_width,-m_half_length, 0);
    vertices[1].set(-m_half_width, m_half_length, 0);
    vertices[2].set(m_half_width,  m_half_length, 0);
    vertices[3].set(m_half_width, -m_half_length, 0);

    for(int i = 0; i < 4; ++i) {
        vertices[i] = xf.transform(vertices[i]);
    }

    b3Vector3d length_step = (vertices[1] - vertices[0]) / (double)segment_count;
    b3Vector3d width_step = (vertices[3] - vertices[0]) / (double)segment_count;

    int index = 0;
    // horizontal edges
    for(int i = 0; i <= segment_count; ++i) {
        m_view_data.m_V[index++] = vertices[0].x() + i * width_step.x();
        m_view_data.m_V[index++] = vertices[0].y() + i * width_step.y();
        m_view_data.m_V[index++] = vertices[0].z() + i * width_step.z();
        m_view_data.m_V[index++] = vertices[1].x() + i * width_step.x();
        m_view_data.m_V[index++] = vertices[1].y() + i * width_step.y();
        m_view_data.m_V[index++] = vertices[1].z() + i * width_step.z();
    }
    // vertical edges
    for(int i = 0; i <= segment_count; ++i) {
        m_view_data.m_V[index++] = vertices[0].x() + i * length_step.x();
        m_view_data.m_V[index++] = vertices[0].y() + i * length_step.y();
        m_view_data.m_V[index++] = vertices[0].z() + i * length_step.z();
        m_view_data.m_V[index++] = vertices[3].x() + i * length_step.x();
        m_view_data.m_V[index++] = vertices[3].y() + i * length_step.y();
        m_view_data.m_V[index++] = vertices[3].z() + i * length_step.z();
    }
}