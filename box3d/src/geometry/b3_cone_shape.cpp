
#include "geometry/b3_cone_shape.hpp"

#include "collision/b3_aabb.hpp"
#include "common/b3_block_allocator.hpp"
#include "dynamics/b3_mass_property.hpp"

void b3ConeShape::set_as_cone(real radius, real height)
{
    m_centroid.set_zero();
    m_radius = radius;
    m_height = height;

    real theta = 0;
    real delta_theta = b3_pi * 2 / B3_CONE_MAX_BASE_POINTS;

    for (int i = 0; i < B3_CONE_MAX_BASE_POINTS; i++) {
        m_vertices[i] = m_radius * b3Vec3r(cos(theta), sin(theta), 0);
        m_vertices[i].z = m_height;

        theta += delta_theta;
    }
}


void b3ConeShape::get_bound_aabb(b3AABB *aabb, const b3Transformr &xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    b3Vec3r abs_half_extent(m_radius, m_radius, m_height * 0.5);

    b3Mat33r R = xf.rotation_matrix();

    // xf.position() is the tip of the cone in world space
    b3Vec3r center = xf.position() + R.col(2) * m_height * 0.5;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R(i, j) = b3_abs(R(i, j));
        }
    }

    abs_half_extent = R * abs_half_extent;
    b3Vec3r upper_bound = center + abs_half_extent;
    b3Vec3r lower_bound = center - abs_half_extent;

    aabb->set_aabb(lower_bound, upper_bound);
}


void b3ConeShape::compute_mass_properties(b3MassProperty &mass_data, real density) const
{
    mass_data.m_center = m_centroid;

    // volume = 1 / 3 * Gh, G = 1 / 2 pi r^2  ==> volume = 1 / 6 pi r^2h
    mass_data.m_volume = b3_pi_6 * m_radius * m_radius * m_height;

    mass_data.m_mass = density * mass_data.m_volume;

    // https://zh.wikipedia.org/wiki/%E8%BD%89%E5%8B%95%E6%85%A3%E9%87%8F%E5%88%97%E8%A1%A8
    mass_data.m_Inertia = b3Mat33r::zero();
    mass_data.m_Inertia(2, 2) = 0.3 * mass_data.m_mass * m_radius * m_radius;
    mass_data.m_Inertia(0, 0) = 0.5 * mass_data.m_Inertia(0, 0) + 0.6 * mass_data.m_mass * m_height * m_height;
    mass_data.m_Inertia(1, 1) = 0.5 * mass_data.m_Inertia(0, 0) + 0.6 * mass_data.m_mass * m_height * m_height;
}


b3Shape* b3ConeShape::clone() const {
    void* mem = m_block_allocator->allocate(sizeof(b3ConeShape));
    b3ConeShape* clone = new (mem) b3ConeShape();
    *clone = *this;
    return clone;
}