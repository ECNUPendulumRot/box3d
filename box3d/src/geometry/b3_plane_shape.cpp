
#include "geometry/b3_plane_shape.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_aabb.hpp"


b3PlaneShape::b3PlaneShape()
{
    m_radius = real(0.0);
    m_type = e_plane;
}


void b3PlaneShape::set_as_plane(real length, real width)
{
    m_half_length = length / 2.0;
    m_half_width = width / 2.0;
    m_vertices[0].set(-m_half_width, -m_half_length, 0);
    m_vertices[1].set(-m_half_width, m_half_length, 0);
    m_vertices[2].set(m_half_width, m_half_length, 0);
    m_vertices[3].set(m_half_width, -m_half_length, 0);
}


void b3PlaneShape::get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    b3Vec3r min;
    b3Vec3r max;

    b3Vec3r vertices[4];

    vertices[0].set(-m_half_width, -m_half_length, 0);
    vertices[1].set(-m_half_width, m_half_length, 0);
    vertices[2].set(m_half_width, m_half_length, 0);
    vertices[3].set(m_half_width, -m_half_length, 0);

    for (int32 i = 0; i < 4; i++) {
        b3Vec3r v = xf.transform(vertices[i]);
        min = b3_min_coeff(min, v);
        max = b3_max_coeff(max, v);
    }

    b3Vec3r r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}


void b3PlaneShape::compute_mass_properties(b3MassProperty &mass_data, real density) const
{
    // plane is static object
    mass_data.m_center = b3Vec3r::zero();
    mass_data.m_volume = 0;
    mass_data.m_mass = 0;
    mass_data.m_Inertia = b3Mat33r::zero();
}

b3Shape* b3PlaneShape::clone() const
{
    void* mem = m_block_allocator->allocate(sizeof(b3PlaneShape));
    auto* clone = new (mem) b3PlaneShape;
    *clone = *this;
    return clone;
}


