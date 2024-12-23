
#include "geometry/b3_cone_shape.hpp"

#include "collision/b3_aabb.hpp"
#include "common/b3_block_allocator.hpp"
#include "dynamics/b3_mass_property.hpp"

void b3ConeShape::set_as_cone(real radius, real height)
{
    m_radius = radius;
    m_height = height;
    m_half_height = height * real(0.5);

    m_sin_angle = m_radius / b3_sqrt(m_radius * m_radius + m_height * m_height);

    real theta = 0;
    real delta_theta = b3_pi * 2 / B3_CONE_MAX_BASE_POINTS;

    for (int i = 0; i < B3_CONE_MAX_BASE_POINTS; i++) {
        m_vertices[i] = m_radius * b3Vec3r(cos(theta), sin(theta), 0);
        // m_vertices[i].z = m_height;
        m_vertices[i].z = -m_half_height;

        theta += delta_theta;
    }
}

b3Vec3r b3ConeShape::cone_local_support(const b3Vec3r& v) const {
    real half_height = m_height * real(0.5);

    b3Vec3r res;
    if (v[2] > v.length() * m_sin_angle) {
        res[0] = 0;
        res[1] = 0;
        res[2] = half_height;
    } else {
        real s = b3_sqrt(v[0] * v[0] + v[1] * v[1]);
        if (s > b3_real_epsilon) {
            real d = m_radius / s;
            res[0] = v[0] * d;
            res[1] = v[1] * d;
            res[2] = -half_height;
        } else {
            res[0] = 0;
            res[1] = 0;
            res[2] = -half_height;
        }
    }
    return res;
}

b3Vec3r b3ConeShape::local_get_support_vertex(const b3Vec3r &vec) const {
    return cone_local_support(vec);
}


b3Vec3r b3ConeShape::local_get_supporting_vertex(const b3Vec3r& vec) const {
    b3Vec3r sup_vertex = cone_local_support(vec);
    b3Vec3r vec_norm = vec;
    if (vec_norm.length2() < (b3_real_epsilon * b3_real_epsilon)) {
        vec_norm.set(real(-1.), real(-1.), real(-1.));
    }
    vec_norm.normalized();
    sup_vertex += m_collision_margin * vec_norm;
    return sup_vertex;
}

void b3ConeShape::get_bound_aabb(b3AABB *aabb, const b3Transformr &xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    real margin = m_collision_margin;
    b3Mat33r R_T = xf.rotation_matrix().transpose();

    b3Vec3r min_aabb, max_aabb;

    for (int i = 0; i < 3; i++) {
        b3Vec3r vec = b3Vec3r::zero();
        vec[i] = real(1.);

        b3Vec3r sv = local_get_supporting_vertex(R_T * vec);

        b3Vec3r tmp = xf.transform(sv);
        max_aabb[i] = tmp[i] + margin;
        vec[i] = real(-1.);
        tmp = xf.transform(local_get_supporting_vertex(R_T * vec));
        min_aabb[i] = tmp[i] - margin;
    }

    aabb->set_aabb(min_aabb, max_aabb);

//    b3Vec3r abs_half_extent(m_radius + marge, m_radius + marge, m_height * 0.5 + marge);
//
//    b3Mat33r R = xf.rotation_matrix();
//
//    // xf.position() is the tip of the cone in world space
//    // 这里计算的圆锥轴线的中点
//    b3Vec3r center = xf.position();
//    // center += R.col(2) * m_height * 0.5;
//
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            R(i, j) = b3_abs(R(i, j));
//        }
//    }
//
//    abs_half_extent = R * abs_half_extent;
//    b3Vec3r upper_bound = center + abs_half_extent;
//    b3Vec3r lower_bound = center - abs_half_extent;

//    aabb->set_aabb(lower_bound, upper_bound);
}


void b3ConeShape::compute_mass_properties(b3MassProperty &mass_data, real density) const
{
    mass_data.m_center = m_centroid;

    // volume = 1 / 3 * Gh, G = 1 / 2 pi r^2  ==> volume = 1 / 6 pi r^2h
    mass_data.m_volume = b3_pi_6 * m_radius * m_radius * m_height;

    get_margin();

    // mass_data.m_mass = density * mass_data.m_volume;
    mass_data.m_mass = density;

    b3Vec3r half_extents(m_radius, m_radius, m_height * 0.5);
    half_extents += 2 * get_margin() * b3Vec3r(1, 1, 1);

    real lx = 2 * half_extents.x + get_margin();
    real ly = 2 * half_extents.y + get_margin();
    real lz = 2 * half_extents.z + get_margin();

    const real x2 = lx * lx;
    const real y2 = ly * ly;
    const real z2 = lz * lz;

    const real scaled = mass_data.m_mass * real(0.08333333);
    mass_data.m_local_Inertia.set_zero();
    mass_data.m_local_Inertia(0, 0) = scaled * (y2 + z2);
    mass_data.m_local_Inertia(1, 1) = scaled * (x2 + z2);
    mass_data.m_local_Inertia(2, 2) = scaled * (x2 + y2);
}


b3Shape* b3ConeShape::clone() const {
    void* mem = m_block_allocator->allocate(sizeof(b3ConeShape));
    b3ConeShape* clone = new (mem) b3ConeShape();
    *clone = *this;
    return clone;
}