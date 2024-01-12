
#include "geometry/b3_plane_shape.hpp"
#include "common/b3_allocator.hpp"


box3d::b3PlaneShape::b3PlaneShape() {
    m_radius = b3_polygon_radius;
}


void box3d::b3PlaneShape::set_as_plane(b3Vector3d& point, b3Vector3d& normal, double d) {
    m_point = point;
    m_normal = normal;
    m_radius = d;
}


void box3d::b3PlaneShape::get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 child_index) const {
    b3_NOT_USED(child_index);

    
}
