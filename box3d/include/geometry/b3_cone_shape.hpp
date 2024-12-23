
#ifndef B3_CONE_SHAPE_HPP
#define B3_CONE_SHAPE_HPP

#include "b3_shape.hpp"

// suppose the cone axis is z-axis
/**
 *  the shape is
 *
 *  z   ^
 *     |                      /|
 *     |                    /   |
 *     |_ _ _ _ _ > y      /     |
 *    /                    --------
 *   x
 *
 *  in the local frame, the m_centroid is the center of the bottom circle and the middle point of the tip
 */

#define B3_CONE_MAX_BASE_POINTS 40

class b3ConeShape : public b3Shape {

    real m_height;

    real m_half_height;

    real m_sin_angle;

    b3Vec3r m_centroid;

    b3Vec3r m_vertices[B3_CONE_MAX_BASE_POINTS];

public:

    b3ConeShape() {
        m_type = e_cone;
    }

    ~b3ConeShape() = default;

    int32 get_child_count() const override {
        return 1;
    }

    virtual b3Vec3r local_get_support_vertex(const b3Vec3r& vec) const override;

    void set_as_cone(real radius, real height);

    void get_bound_aabb(b3AABB* aabb, const b3Transformr& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    b3Vec3r local_get_supporting_vertex(const b3Vec3r& vec) const;
    b3Vec3r cone_local_support(const b3Vec3r& v) const;

    b3Shape* clone() const override;

    real get_height() const {
        return m_height;
    }

    real get_half_height() const {
        return m_half_height;
    }

    const b3Vec3r* get_vertices() const {
        return m_vertices;
    }
};

#endif