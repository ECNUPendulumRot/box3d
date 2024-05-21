
#include "geometry/b3_sphere_shape.hpp"

#include <memory>

#include "common/b3_allocator.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_aabb.hpp"


b3SphereShape::b3SphereShape()
{
    m_radius = 0;
    m_type = e_sphere;
}


void b3SphereShape::set_as_sphere(real radius)
{
    m_radius = radius;
    m_centroid.set_zero();
}


void b3SphereShape::get_bound_aabb(b3AABB *aabb, const b3Transformr& xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    b3Vec3r centroid = xf.transform(m_centroid);

    b3Vec3r radius(m_radius, m_radius, m_radius);

    aabb->m_min = centroid - radius;
    aabb->m_max = centroid + radius;
}


void b3SphereShape::compute_mass_properties(b3MassProperty& mass_data, real density) const
{
    mass_data.m_center = m_centroid;

    mass_data.m_volume = 4 * real(b3_pi) * m_radius * m_radius * m_radius / 3;

    mass_data.m_mass = density * mass_data.m_volume;

    real ixx = real(0.4) * mass_data.m_mass * m_radius * m_radius;

    mass_data.m_Inertia = b3Mat33r::zero();
    mass_data.m_Inertia(0, 0) = ixx;
    mass_data.m_Inertia(1, 1) = ixx;
    mass_data.m_Inertia(2, 2) = ixx;
}


b3Shape* b3SphereShape::clone() const
{
    void *memory = m_block_allocator->allocate(sizeof(b3SphereShape));
    auto *clone = new (memory) b3SphereShape;
    *clone = *this;
    return clone;
}

