
#ifndef BOX3D_B3PLANESHAPE_HPP
#define BOX3D_B3PLANESHAPE_HPP

#include "geometry/b3_shape.hpp"

class b3PlaneShape : public b3Shape {

    double m_half_length, m_half_width;

    static int segment_count;

public:

    b3PlaneShape();

    /**
     * @param point: a point on this plane
     * @param normal: the normal of this plane
    */
    void set_as_plane(double length, double width);

    virtual ~b3PlaneShape() = default;

    int32 get_child_count() const {
        return 1;
    }

    void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, double density) const override;

    b3Shape* clone() const override;

    void init_view_data() override;

    void setup_view_data(const b3TransformD& xf) override;
};


#endif  // BOX3D_B3PLANESHAPE_HPP