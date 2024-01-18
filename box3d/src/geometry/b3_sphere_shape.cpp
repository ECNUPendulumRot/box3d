
#include "geometry/b3_sphere_shape.hpp"
#include "common/b3_allocator.hpp"

#include "dynamics/b3_body.hpp"


#define K_SEGMENTS 20

const box3d::b3SphereConfig box3d::b3SphereShape::m_config;


box3d::b3SphereConfig::b3SphereConfig() {

    m_segments = K_SEGMENTS;
    
    double k_increments = b3_pi / m_segments;
    double sin_inc = sin(k_increments);
    double cos_inc = cos(k_increments);

    m_rot_y <<  cos_inc,        0,      sin_inc,
                0,              1,      0,
                -sin_inc,       0,      cos_inc;

    m_rot_z <<  cos_inc,    -sin_inc,   0,
                sin_inc,    cos_inc,    0,
                0,          0,          1;

    // the sphere is divided into two points on the end of sphere
    // and %m_segments - 1% rings. 
    // Every ring has %m_ring_points_count% points.
    m_ring_points_count = 2 * m_segments;
    m_vertices_rows = m_ring_points_count * (m_segments - 1) + 2;
    m_faces_rows = (m_segments - 1) * 2 * m_ring_points_count;
}


box3d::b3SphereShape::b3SphereShape() {
    m_radius = 0;
    m_type = e_sphere;
}


void box3d::b3SphereShape::set_as_sphere(double radius) {
    m_radius = radius;
    m_centroid.set_zero();
}


void box3d::b3SphereShape::get_bound_aabb(box3d::b3AABB *aabb, const b3TransformD& xf, int32 child_index) const {
    
    b3_NOT_USED(child_index);

    b3Vector3d centroid = xf.transform(m_centroid);

    b3Vector3d radius(m_radius, m_radius, m_radius);

    aabb->m_min = centroid - radius;
    aabb->m_max = centroid + radius;
}


void box3d::b3SphereShape::compute_mass_properties(b3MassProperty& mass_data, double density) const {

    mass_data.m_center = m_centroid;

    mass_data.m_volume = 4 * b3_pi * m_radius * m_radius * m_radius / 3;

    mass_data.m_mass = density * mass_data.m_volume;

    double ixx = 0.4 * mass_data.m_mass * m_radius * m_radius;

    mass_data.m_Inertia << ixx, 0.0, 0.0, 
                           0.0, ixx, 0.0,
                           0.0, 0.0, ixx;
}


box3d::b3Shape* box3d::b3SphereShape::clone() const {
    
    void* mem = b3_alloc(sizeof(b3SphereShape));
    auto* clone = new (mem) b3SphereShape;
    *clone = *this;
    return clone;
}


void box3d::b3SphereShape::get_view_data(b3ViewData* view_data) const {

    view_data->m_V.resize(m_config.m_vertices_rows, 3);
    
    const E3Matrix3d& rot_y = m_config.m_rot_y;
    const E3Matrix3d& rot_z = m_config.m_rot_z;

    Eigen::Vector3d v1(0, 0, 1);
    Eigen::Vector3d v2;
    int index = 0;

    // transform the center of sphere to world frame
    auto body_pose = m_body->get_pose();
    Eigen::Vector3d world_center = m_body->get_pose().transform(m_centroid).eigen_vector3();

    for(int i = 1; i < m_config.m_segments; ++i) {
        v1 = rot_y * v1;
        v2 = v1;

        for(int j = 0; j < m_config.m_segments; ++j) {
            v2 = rot_z * v2;
            view_data->m_V.row(index++) = world_center + m_radius * v2;
        }

        v2 = v1;
        double v2_x = v2.x();
        v2.x() = -v2_x;
        for(int j = 0; j < m_config.m_segments; ++j) {
            v2 = rot_z * v2;
            view_data->m_V.row(index++) = world_center + m_radius * v2;
        }
    }

    v1 = rot_y * v1;
    // two end of sphere, actually are (0, 0, 1) and (0, 0, -1)
    v2 = rot_z * v1;
    view_data->m_V.row(index++) = world_center + m_radius * v2;
    view_data->m_V.row(index++) = world_center - m_radius * v2;
        

    // the number of rings is k_segments - 1, 
    // every adjacent ring need construct the triangle face.
    
    // the number of points on one ring

    // point_number + point_number + (k_segments - 1 - 1) * 2 * point_number;
    // = (k_segments - 1) * 2 * point_number
    view_data->m_F.resize(m_config.m_faces_rows, 3);
    index = 0;
    {
        // the point (0, 0, 1) with the first ring
        int first = m_config.m_vertices_rows - 1;
        int second = 0;

        view_data->m_F.row(index++) << first, second, m_config.m_ring_points_count - 1;

        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            view_data->m_F.row(index++) << first, second, second + 1;
            second = second + 1;
        }
    }

    {
        // the point (0, 0, -1) with the last ring
        int first = m_config.m_vertices_rows - 2;
        int second = m_config.m_vertices_rows - m_config.m_ring_points_count - 2;

        view_data->m_F.row(index++) << first, second, m_config.m_vertices_rows - 3;

        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            view_data->m_F.row(index++) << first, second, second + 1;
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
        view_data->m_F.row(index++) << first_ring_index, 
                                       second_ring_index - 1, 
                                       second_ring_index;
        view_data->m_F.row(index++) << second_ring_index - 1, 
                                       second_ring_index, 
                                       second_ring_index + m_config.m_ring_points_count - 1;
        // two near ring
        // first ring points index(based on first_ring_index):    0, 1, 2, 3, ……， n-2, n-1, 0
        //                                                                              ------
        //                                                                              |    /| 
        //                                                                              |  /  |
        //                                                                              |-----
        // second ring points index(based on second_ring_index):  0, 1, 2, 3, ……， n-2, n-1, 0
        for(int j = 1; j < m_config.m_ring_points_count; ++j) {
            view_data->m_F.row(index++) << first_ring_index, second_ring_index, second_ring_index + 1;
            second_ring_index++;
            view_data->m_F.row(index++) << first_ring_index, second_ring_index, first_ring_index + 1;
            first_ring_index++;
        }
    }

}