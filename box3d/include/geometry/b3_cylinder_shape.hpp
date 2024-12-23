
#ifndef B3_CYLINDER_SHAPE_HPP
#define B3_CYLINDER_SHAPE_HPP

#include "b3_shape.hpp"

#define B3_CYLINDER_MAX_POINTS 40

class b3CylinderShape : public b3Shape {

    real m_height;

    b3Vec3r m_vertices[B3_CYLINDER_MAX_POINTS];

public:

    b3CylinderShape() {
        m_type = e_cylinder;
    }

    ~b3CylinderShape() = default;

    int32 get_child_count() const override {
        return 1;
    }

    // virtual b3Vec3r local_get_support_vertex(const b3Vec3r& vec) const override;

    void set_as_cylinder(real radius, real height);

    void get_bound_aabb(b3AABB* aabb, const b3Transformr& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    // b3Vec3r local_get_supporting_vertex(const b3Vec3r& vec) const;

    b3Shape* clone() const override;

    real get_height() const {
        return m_height;
    }

    const b3Vec3r* get_vertices() const {
        return m_vertices;
    }
};

#endif