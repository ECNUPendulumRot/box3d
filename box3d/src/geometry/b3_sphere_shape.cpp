
#include "geometry/b3_sphere_shape.hpp"

#include "common/b3_allocator.hpp"

#include "collision/b3_aabb.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"


#define K_SEGMENTS 20

const b3SphereConfig b3SphereShape::m_config;


b3SphereConfig::b3SphereConfig() {

    m_segments = K_SEGMENTS;
    
    double k_increments = b3_pi / m_segments;
    double sin_inc = sin(k_increments);
    double cos_inc = cos(k_increments);

    b3Vector3d rot_col_1(cos_inc, 0, -sin_inc);
    b3Vector3d rot_col_2(0, 1, 0);
    b3Vector3d rot_col_3(sin_inc, 0, cos_inc);
    m_rot_y = b3Matrix3d(rot_col_1, rot_col_2, rot_col_3);

    rot_col_1 = {cos_inc, -sin_inc, 0};
    rot_col_2 = {sin_inc, cos_inc, 0};
    rot_col_3 = {0, 0, 1};
    m_rot_z = b3Matrix3d(rot_col_1, rot_col_2, rot_col_3);

    // the sphere is divided into two points on the end of sphere
    // and %m_segments - 1% rings. 
    // Every ring has %m_ring_points_count% points.
    m_ring_points_count = 2 * m_segments;
    m_vertices_count = m_ring_points_count * (m_segments - 1) + 2;
    m_faces_count = (m_segments - 1) * 2 * m_ring_points_count;

    m_vertices_size = m_vertices_count * 3;
    m_faces_size = m_faces_count * 3;
}


b3SphereShape::b3SphereShape() {
    m_radius = 0;
    m_type = e_sphere;
}


void b3SphereShape::set_as_sphere(double radius) {
    m_radius = radius;
    m_centroid.set_zero();
}


void b3SphereShape::get_bound_aabb(b3AABB *aabb, const b3TransformD& xf, int32 child_index) const {
    
    b3_NOT_USED(child_index);

    b3Vector3d centroid = xf.transform(m_centroid);

    b3Vector3d radius(m_radius, m_radius, m_radius);

    aabb->m_min = centroid - radius;
    aabb->m_max = centroid + radius;
}


void b3SphereShape::compute_mass_properties(b3MassProperty& mass_data, double density) const {

    mass_data.m_center = m_centroid;

    mass_data.m_volume = 4 * b3_pi * m_radius * m_radius * m_radius / 3;

    mass_data.m_mass = density * mass_data.m_volume;

    double ixx = 0.4 * mass_data.m_mass * m_radius * m_radius;

    mass_data.m_Inertia = b3Matrix3d::zero();
    mass_data.m_Inertia(0, 0) = ixx;
    mass_data.m_Inertia(1, 1) = ixx;
    mass_data.m_Inertia(2, 2) = ixx;
}


b3Shape* b3SphereShape::clone() const {
    
    void* mem = b3_alloc(sizeof(b3SphereShape));
    auto* clone = new (mem) b3SphereShape;
    *clone = *this;
    return clone;
}



void b3SphereShape::init_view_data() {

    m_view_data = new b3ViewData();

    m_view_data->m_vertex_count = m_config.m_vertices_count;

    void* mem = b3_alloc(m_config.m_vertices_size * sizeof(double));
    m_view_data->m_V = new (mem) double;
    
    const b3Matrix3d& rot_y = m_config.m_rot_y;
    const b3Matrix3d& rot_z = m_config.m_rot_z;

    b3Vector3d v1(0, 0, 1);
    b3Vector3d v2;
    int index = 0;

    // transform the center of sphere to world frame
    auto body_pose = m_body->get_pose();
    b3Vector3d world_center = m_body->get_pose().transform(m_centroid);

    for(int i = 1; i < m_config.m_segments; ++i) {
        v1 = rot_y * v1;
        v2 = v1;

        for(int j = 0; j < m_config.m_segments; ++j) {
            v2 = rot_z * v2;

            b3Vector3d v = world_center + m_radius * v2;
            m_view_data->m_V[index++] = v.x();
            m_view_data->m_V[index++] = v.y();
            m_view_data->m_V[index++] = v.z();
        }

        v2 = v1;
        double v2_x = v2.x();

        v2.x() = -v2_x;
        for(int j = 0; j < m_config.m_segments; ++j) {
            v2 = rot_z * v2;

            b3Vector3d v = world_center + m_radius * v2;
            m_view_data->m_V[index++] = v.x();
            m_view_data->m_V[index++] = v.y();
            m_view_data->m_V[index++] = v.z();
        }
    }

    v1 = rot_y * v1;
    // two end of sphere, actually are (0, 0, 1) and (0, 0, -1)
    v2 = rot_z * v1;


    b3Vector3d v = world_center + m_radius * v2;
    m_view_data->m_V[index++] = v.x();
    m_view_data->m_V[index++] = v.y();
    m_view_data->m_V[index++] = v.z();


    v = world_center - m_radius * v2;
    m_view_data->m_V[index++] = v.x();
    m_view_data->m_V[index++] = v.y();
    m_view_data->m_V[index++] = v.z();
        

    // the number of rings is k_segments - 1, 
    // every adjacent ring need construct the triangle face.
    
    // the number of points on one ring

    // point_number + point_number + (k_segments - 1 - 1) * 2 * point_number;
    // = (k_segments - 1) * 2 * point_number

    m_view_data->m_face_count = m_config.m_faces_count;
    mem = b3_alloc(m_config.m_faces_size * sizeof(int));
    m_view_data->m_F = new (mem) int;

    index = 0;
    {
        // the point (0, 0, 1) with the first ring
        int first = m_config.m_vertices_count - 1;
        int second = 0;

        m_view_data->m_F[index++] = first;
        m_view_data->m_F[index++] = second;
        m_view_data->m_F[index++] = m_config.m_ring_points_count - 1;

        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            m_view_data->m_F[index++] = first;
            m_view_data->m_F[index++] = second;
            m_view_data->m_F[index++] = second + 1;

            second = second + 1;
        }
    }

    {
        // the point (0, 0, -1) with the last ring
        int first = m_config.m_vertices_count - 2;
        int second = m_config.m_vertices_count - m_config.m_ring_points_count - 2;

        m_view_data->m_F[index++] = first;
        m_view_data->m_F[index++] = second;
        m_view_data->m_F[index++] = m_config.m_vertices_count - 3;

        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            m_view_data->m_F[index++] = first;
            m_view_data->m_F[index++] = second;
            m_view_data->m_F[index++] = second + 1;
            second = second + 1;
        }
    }

    int first_ring_index = -1;
    int second_ring_index = 0;
    for(int i = 0; i < m_config.m_segments - 2; ++i) {
        
        // construct face between adjacent ring.

        first_ring_index++;
        second_ring_index = first_ring_index + m_config.m_ring_points_count;
        // first construct two face index overflow
        // first ring start, end  and second ring start
        // first ring end, and second ring start and end
        m_view_data->m_F[index++] = first_ring_index;
        m_view_data->m_F[index++] = second_ring_index - 1;
        m_view_data->m_F[index++] = second_ring_index;
        m_view_data->m_F[index++] = second_ring_index - 1;
        m_view_data->m_F[index++] = second_ring_index;
        m_view_data->m_F[index++] = second_ring_index + m_config.m_ring_points_count - 1;

        // two near ring
        // first ring points index(based on first_ring_index):    0, 1, 2, 3, ……， n-2, n-1, 0
        //                                                                              ------
        //                                                                              |    /| 
        //                                                                              |  /  |
        //                                                                              |-----
        // second ring points index(based on second_ring_index):  0, 1, 2, 3, ……， n-2, n-1, 0
        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            m_view_data->m_F[index++] = first_ring_index;
            m_view_data->m_F[index++] = second_ring_index;
            m_view_data->m_F[index++] = second_ring_index + 1;

            second_ring_index++;

            m_view_data->m_F[index++] = first_ring_index;
            m_view_data->m_F[index++] = second_ring_index;
            m_view_data->m_F[index++] = first_ring_index + 1;
            first_ring_index++;
        }
    }

}