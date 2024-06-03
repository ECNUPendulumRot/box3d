
#include "b3_shape.hpp"

// suppose the cone axis is z-axis
/**
 *  the shape is
 *
 *  z   ^
 *     |                    --------
 *     |                    \      /
 *     |_ _ _ _ _ > y        \    /
 *    /                       \  /
 *   x                         \/
 *
 */

class b3ConeShape : public b3Shape {

    real m_sin_angle;
    real m_height;

    b3Vec3r m_centroid;

public:

    b3ConeShape() {
        m_type = e_cone;
    }

    ~b3ConeShape() = default;

    int32 get_child_count() const override {
        return 0;
    }

    void set_as_cone(real radius, real height);

    void get_bound_aabb(b3AABB* aabb, const b3Transformr& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    b3Shape* clone() const override;

    real get_height() const {
        return m_height;
    }
};