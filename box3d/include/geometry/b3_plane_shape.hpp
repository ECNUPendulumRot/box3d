
#ifndef BOX3D_B3PLANESHAPE_HPP
#define BOX3D_B3PLANESHAPE_HPP

#include "geometry/b3_shape.hpp"

namespace box3d {

    class b3PlaneShape;

}


class box3d::b3PlaneShape : public b3Shape {

    b3Vector3d m_point;

    b3Vector3d m_normal;

public:

    b3PlaneShape();

    /**
     * @param point: a point on this plane
     * @param normal: the normal of this plane
     * @param d: the thickness of this plane， default is b3_polygon_radius = 0.01
    */
    void set_as_plane(b3Vector3d& point, b3Vector3d& normal, double d = b3_polygon_radius);

    virtual ~b3PlaneShape() = default;

    int32 get_child_count() const {
        return 1;
    }

    void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, double density) const override;

    b3Shape* clone() const override;
};


#endif  // BOX3D_B3PLANESHAPE_HPP