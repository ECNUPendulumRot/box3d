
#ifndef BOX3D_B3PLANESHAPE_HPP
#define BOX3D_B3PLANESHAPE_HPP


#include "geometry/b3_shape.hpp"


class b3PlaneShape : public b3Shape {

public:

    real m_half_length, m_half_width;

    b3PlaneShape();

    /**
     * @param point: a point on this plane
     * @param normal: the normal of this plane
    */
    void set_as_plane(real length, real width);

    int32 get_child_count() const {
        return 1;
    }

    void get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    b3Shape* clone() const override;
};


#endif  // BOX3D_B3PLANESHAPE_HPP