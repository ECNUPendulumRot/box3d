
#ifndef BOX3D_B3SPHERESHAPE_HPP
#define BOX3D_B3SPHERESHAPE_HPP


#include "geometry/b3_shape.hpp"


class b3SphereShape : public b3Shape {

    b3Vec3r m_centroid;

public:

    b3SphereShape();

    int32 get_child_count() const override {
        return 1;
    }

    b3Vec3r get_centroid() const {
        return m_centroid;
    }

    /**
     * when create a sphere, we need call this function at first.
     * @param radius
     */
    void set_as_sphere(real radius);

    b3Shape* clone() const override;

    void get_bound_aabb(b3AABB* aabb, const b3Transformr& xf, int32 child_index) const override;

    void compute_mass_properties(b3MassProperty& mass_data, real density) const override;

    real get_radius() const override {
        return m_radius;
    }
};


#endif // BOX3D_B3SPHERESHAPE_HPP