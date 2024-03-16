
#ifndef BOX3D_B3SPHERESHAPE_HPP
#define BOX3D_B3SPHERESHAPE_HPP


#include "geometry/b3_shape.hpp"


struct b3SphereConfig {

    int m_segments;

    b3Matrix3r m_rot_y, m_rot_z;

    int m_vertices_count;
    int m_faces_count;

    // size = count * 3;
    int m_vertices_size;
    int m_faces_size;

    int m_ring_points_count;

    b3SphereConfig();

};


class b3SphereShape : public b3Shape {

    b3Vector3r m_centroid;

    // control generate sphere view data
    static const b3SphereConfig m_config;

    /**
     * when the first get view data, we need generate all vertices of sphere.
     * after first, we only need to move the vertices of sphere.
     * because the rotation is not effect to the sphere.
     */
    bool m_first_setup_view;

    /**
     * This is used to move vertices of sphere.
     */
    b3Vector3r m_old_center;

public:

    b3SphereShape();

    int32 get_child_count() const override {
        return 1;
    }

    b3Vector3r get_centroid() const {
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

    void init_view_data() override;

    void setup_view_data(const b3Transformr& xf) override;
};


#endif // BOX3D_B3SPHERESHAPE_HPP