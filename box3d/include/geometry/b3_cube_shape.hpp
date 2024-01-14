
#ifndef BOX3D_B3_CUBE_SHAPE_HPP
#define BOX3D_B3_CUBE_SHAPE_HPP


#include "geometry/b3_shape.hpp"


namespace box3d{

    class b3CubeShape;

}


class box3d::b3CubeShape : public b3Shape {

    b3Vector3d m_centroid;

    b3Vector3d m_vertices[8];

    b3Vector3d m_normals[6];

    b3Vector3d m_xyz;

    b3Vector3d m_h_xyz;
    
public:

    b3CubeShape();

    /**
     * @brief create the cube with length, width and height
     * @param hx: the half width of the cube
     * @param hy: the half length of the cube
     * @param hz: the half height of the cube
     */
    void set_as_box(double hx, double hy, double hz);

    virtual ~b3CubeShape() = default;

    int32 get_child_count() const override {
        return 1;
    }

    inline b3Vector3d get_half_xyz() const {
        return m_h_xyz;
    }

    void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 childIndex) const override;

    void compute_mass_properties(b3MassProperty& mass_data, double density) const override;

    void get_view_data(b3ViewData* view_data) const override;

    b3Shape* clone() const override;

};


#endif //BOX3D_B3_CUBE_SHAPE_HPP
