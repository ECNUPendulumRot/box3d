
#include "geometry/b3_cylinder_shape.hpp"

#include "collision/b3_aabb.hpp"
#include "common/b3_block_allocator.hpp"
#include "dynamics/b3_mass_property.hpp"

void b3CylinderShape::set_as_cylinder(real radius, real height) {
    m_radius = radius;
    m_height = height;

    real theta = 0;
    real delta_theta = b3_pi * 2 / B3_CYLINDER_MAX_POINTS;

    for (int i = 0; i < B3_CYLINDER_MAX_POINTS; i++) {
        m_vertices[i] = m_radius * b3Vec3r(cos(theta), sin(theta), 0);
        theta += delta_theta;
    }
}

void b3CylinderShape::get_bound_aabb(b3AABB *aabb, const b3Transformr &xf, int32 child_index) const {
    b3Vec3r half_extents(m_radius, m_radius, m_height * 0.5);
    b3Mat33r abs_R = xf.rotation_matrix().absolute();
    b3Vec3r center = xf.position();
    b3Vec3r extent = abs_R * half_extents;
    aabb->min() = center - extent;
    aabb->max() = center + extent;
}

void b3CylinderShape::compute_mass_properties(b3MassProperty &mass_data, real density) const {
    mass_data.m_center = m_centroid;
    mass_data.m_mass = density;
    real div12 = mass_data.m_mass / 12.f;
    real div4 = mass_data.m_mass / 4.f;
    real div2 = mass_data.m_mass / 2.f;

    real radius2 = (m_radius + get_margin()) * (m_radius + get_margin());
    real height2 = (m_height + get_margin()) * (m_height + get_margin());

    real t1 = div12 * height2 + div4 * radius2;
    real t2 = div2 * radius2;

    mass_data.m_local_Inertia.set_zero();
    mass_data.m_local_Inertia(0, 0) = t1;
    mass_data.m_local_Inertia(1, 1) = t1;
    mass_data.m_local_Inertia(2, 2) = t2;

    mass_data.m_local_Inertia(0, 0) = 1;
    mass_data.m_local_Inertia(1, 1) = 1;
    mass_data.m_local_Inertia(2, 2) = 1;
}


b3Shape* b3CylinderShape::clone() const {
    void* mem = m_block_allocator->allocate(sizeof(b3CylinderShape));
    b3CylinderShape* clone = new (mem) b3CylinderShape();
    *clone = *this;
    return clone;
}