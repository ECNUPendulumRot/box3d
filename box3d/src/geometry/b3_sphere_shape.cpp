
#include "geometry/b3_sphere_shape.hpp"
#include "common/b3_allocator.hpp"

#include <Eigen/Dense>

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
    // default is divide 2
    const int k_segments = 8;
    const double k_increments = b3_pi / k_segments;

    double sin_inc = sin(k_increments);
    double cos_inc = cos(k_increments);

    // Sample n rings on the sphere
    // Sample n points on the ring
    // At both ends of the sphere, the sampling points are the same, so one is need.
    view_data->m_V.resize(2 * k_segments * (k_segments - 1) + 2, 3);

    // Perform rotation to avoid additional trigonometry.
    // z = r * sin(a)
    // x = r * cos(a) * cos(b)
    // y = r * cos(a) * sin(b)
    b3Matrix3d rot_y, rot_z;
    rot_y << cos_inc, 0, sin_inc,
             0, 1, 0,
             -sin_inc, 0, cos_inc;

    rot_z << cos_inc, 0, -sin_inc,
             sin_inc, 0, cos_inc,
             0, 0, 1;
    Eigen::Vector3d v1(0, 0, 1);
    int index = 0;
    for(int i = 0; i < k_segments; ++i) {
        v1 = rot_y * v1;
        Eigen::Vector3d v2 = v1;

        if(i == k_segments - 1) {
            // two end of sphere
            // actually are (0, 0, 1) and (0, 0, -1)
            v2 = rot_z * v2;
            view_data->m_V.row(index++) = m_centroid.eigen_vector3() + m_radius * v2;
            view_data->m_V.row(index++) = m_centroid.eigen_vector3() - m_radius * v2;
            break;
        }

        for(int j = 0; j < k_segments; ++j) {
            v2 = rot_z * v2;
            view_data->m_V.row(index++) = m_centroid.eigen_vector3() + m_radius * v2;
        }

        v2 = -v1;
        for(int j = 0; j < k_segments; ++j) {
            v2 = rot_z * v2;
            view_data->m_V.row(index++) = m_centroid.eigen_vector3() + m_radius * v2;
        }
    }

    int rows = view_data->m_V.rows();

    // now we have 2n(n-1) + 2 points(n = k_segments)
    // 0 - 2n-1 are the first ring and ordering
    // 2n - 4n-1 are the second ring and ordering 
    // ....
    // the 2n(n-1)th point is end of the point on the sphere(-z)
    // the 2n(n-1)+1th point is end of the point on the sphere(z)

    // the number of rings is k_segments - 1, 
    // every adjacent ring need construct the triangle face.
    
    // the number of points on one ring
    int point_number = 2 * k_segments;

    // point_number + point_number + (k_segments - 1 - 1) * 2 * point_number;
    // = (k_segments - 1) * 2 * point_number
    view_data->m_F.resize((k_segments - 1) * 2 * point_number, 3);

    {
        // the point (0, 0, 1) with the first ring
        int first = rows - 1;
        int second = 0;
        view_data->m_F.row(index++) << first, second, point_number - 1;
        for(int j = 1; j < point_number; ++j) {
            view_data->m_F.row(index++) << first, second, second + 1;
            second = second + 1;
        }
    }
    {
        // the point (0, 0, -1) with the last ring
        int first = rows - 2;
        int second = rows - point_number - 2;
        // rows - 2point_num - 3 + 1 = rows - 2point_num - 2
        view_data->m_F.row(index++) << first, second, rows - 3;
        for(int j = 1; j < point_number; ++j) {
            view_data->m_F.row(index++) << first, second, second + 1;
            second = second + 1;
        }
    }


    // the number of ring is k_segment - 1
    for(int i = 0; i < k_segments - 2; ++i) {
        // construct face between adjacent ring.
        // first ring points are the index: [point_number * i, point_number * (i + 1) -1]
        // second ring points are the index: [point_number * (i + 1), point_number * (i + 2) - 1] 
        int first_ring_index = point_number * i;
        int second_ring_index = first_ring_index + point_number;
        // first construct two face index overflow
        // first ring start, end  and second ring end
        // first ring start, and second ring start and end
        int second_ring_end = second_ring_index + point_number - 1;
        view_data->m_F.row(index++) << first_ring_index, second_ring_index - 1, second_ring_index;
        view_data->m_F.row(index++) << second_ring_index - 1, second_ring_index, second_ring_end;
        
        for(int j = 1; j < point_number; ++j) {
            view_data->m_F.row(index++) << first_ring_index, second_ring_index, second_ring_index - 1;
            second_ring_index++;
            view_data->m_F.row(index++) << first_ring_index, second_ring_index, first_ring_index + 1;
            first_ring_index++;
        }
    }

}