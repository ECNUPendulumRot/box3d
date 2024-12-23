
#include "collision/b3_persistent_manifold.hpp"


static inline real calc_area_four_points(const b3Vec3r& p0, const b3Vec3r& p1, const b3Vec3r& p2, const b3Vec3r& p3)
{
    b3Vec3r a[3], b[3];
    a[0] = p0 - p1;
    a[1] = p0 - p2;
    a[2] = p0 - p3;
    b[0] = p2 - p3;
    b[1] = p1 - p3;
    b[2] = p1 - p2;

    b3Vec3r temp0 = a[0].cross(b[0]);
    b3Vec3r temp1 = a[1].cross(b[1]);
    b3Vec3r temp2 = a[2].cross(b[2]);

    return b3_max(b3_max(temp0.length2(), temp1.length2()), temp2.length2());
}


int b3PersistentManifold::sort_points(const b3PersistentManifoldPoint &point)
{
    // calculate 4 possible cases areas, and take the biggest area
    // also need to keep 'deepest'

    int max_penetration_index = -1;

    real max_penetration = point.get_distance();
    for (int i = 0; i < 4; ++i) {
        // penetration is negative
        if (m_points[i].get_distance() < max_penetration) {
            max_penetration_index = i;
            max_penetration = m_points[i].get_distance();
        }
    }

    real res0{0}, res1{0}, res2{0}, res3{0};

    /// approximate the convex hull area using 3 points
    /// if use 4 points to compute the area: it is more accurate but slower

    auto compute_area = [](const b3Vec3r& a, const b3Vec3r& b) {
        return (a.cross(b)).length2();
    };

    if (max_penetration_index != 0) {
        b3Vec3r a = point.m_local_pointA - m_points[1].m_local_pointA;
        b3Vec3r b = m_points[3].m_local_pointA - m_points[2].m_local_pointA;
        res0 = compute_area(a, b);
    }

    if (max_penetration_index != 1) {
        b3Vec3r a = point.m_local_pointA - m_points[0].m_local_pointA;
        b3Vec3r b = m_points[3].m_local_pointA - m_points[2].m_local_pointA;
        res1 = compute_area(a, b);
    }

    if (max_penetration_index != 2) {
        b3Vec3r a = point.m_local_pointA - m_points[0].m_local_pointA;
        b3Vec3r b = m_points[3].m_local_pointA - m_points[1].m_local_pointA;
        res2 = compute_area(a, b);
    }

    if (max_penetration_index != 3) {
        b3Vec3r a = point.m_local_pointA - m_points[0].m_local_pointA;
        b3Vec3r b = m_points[2].m_local_pointA - m_points[1].m_local_pointA;
        res3 = compute_area(a, b);
    }

    real max_area = b3_max(b3_max(res0, res1), res2);
    max_area = b3_max(max_area, res3);
    return max_area;
}


int b3PersistentManifold::get_cache_entry(const b3PersistentManifoldPoint &new_point) const
{
    real shortest_distance = m_contact_breaking_threshold * m_contact_breaking_threshold;
    int nearest_point = -1;
    for (int i = 0; i < m_point_count; ++i) {
        const b3PersistentManifoldPoint& point = m_points[i];

        b3Vec3r diffA = point.m_local_pointA - new_point.m_local_pointA;
        const real dist_manifold_point = diffA.length2();

        if (dist_manifold_point < shortest_distance) {
            shortest_distance = dist_manifold_point;
            nearest_point = i;
        }
    }
    return nearest_point;
}


int b3PersistentManifold::add_manifold_point(const b3PersistentManifoldPoint &new_point)
{
    int insert_index = m_point_count;
    if (m_point_count >= 4) {
        insert_index = sort_points(new_point);
    } else {
        m_point_count++;
    }
    m_points[insert_index] = new_point;
    return insert_index;
}


void b3PersistentManifold::refresh_contact_points(const b3Transformr &xfA, const b3Transformr &xfB)
{
    for (int i = 0; i < m_point_count; i++) {
        b3PersistentManifoldPoint& manifold_point = m_points[i];
        b3Vec3r world_pointA = xfA.transform(manifold_point.m_local_pointA);
        b3Vec3r world_pointB = xfB.transform(manifold_point.m_local_pointB);
        // TODO: delete this
        manifold_point.m_position_world_on_A = world_pointA;
        manifold_point.m_position_world_on_B = world_pointB;
        manifold_point.m_distance = (world_pointB - world_pointA).dot(manifold_point.m_normal_world_on_A);
        manifold_point.m_lifetime++;
    }

    for (int i = m_point_count - 1; i >= 0; i--) {
        b3PersistentManifoldPoint& manifold_point = m_points[i];

        // contact becomes invalid when signed distance exceeds margin (projected on contact_normal direction)
        if (!valid_contact_distance(manifold_point)) {
            remove_contact_point(i);
            continue;
        }

        // contact also becomes invalid when relative movement orthogonal to normal exceeds margin
        b3Vec3r projected_point = manifold_point.m_position_world_on_A + manifold_point.m_normal_world_on_A * manifold_point.m_distance;
        b3Vec3r projected_difference = manifold_point.m_position_world_on_B - projected_point;
        real distance2 = projected_difference.length2();
        if (distance2 > m_contact_breaking_threshold * m_contact_breaking_threshold) {
            remove_contact_point(i);
            continue;
        }

        // not remove
        // TODO: maybe add contact point processed callback
    }
}


void b3PersistentManifold::remove_contact_point(int index)
{
    m_point_count--;
    if (index != m_point_count) {
        m_points[index] = m_points[m_point_count];
    }
}


void b3PersistentManifold::replace_contact_point(const b3PersistentManifoldPoint&new_point, int insert_index)
{
    b3_assert(valid_contact_distance(new_point));

    // TODO: Maybe use Bullet's Maintain persistency
    int lifeTime = m_points[insert_index].get_lifetime();
    real applied_impulse = m_points[insert_index].m_applied_impulse;
    real applied_lateral_impulse1 = m_points[insert_index].m_applied_tangent_impulse1;
    real applied_lateral_impulse2 = m_points[insert_index].m_applied_tangent_impulse2;
    real prev_rhs = m_points[insert_index].m_prevRHS;

    m_points[insert_index] = new_point;
    m_points[insert_index].m_applied_impulse = applied_impulse;
    m_points[insert_index].m_prevRHS = prev_rhs;
    m_points[insert_index].m_applied_tangent_impulse1 = applied_lateral_impulse1;
    m_points[insert_index].m_applied_tangent_impulse2 = applied_lateral_impulse2;
    m_points[insert_index].m_lifetime = lifeTime;
}

