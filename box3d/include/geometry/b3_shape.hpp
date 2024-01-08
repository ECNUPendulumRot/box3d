
#ifndef BOX3D_B3_SHAPE_CPP
#define BOX3D_B3_SHAPE_HPP

#include "common/b3_types.hpp"

#include "dynamics/b3_pose.hpp"

#include "collision/b3_aabb.hpp"

#include "dynamics/b3_inertia.hpp"


namespace box3d {

    class b3Shape;

    class b3ViewData;

    enum b3ShapeType {
        e_type_not_defined = -1,
        e_mesh       = 0,
        e_cube       = 1,
        e_sphere     = 2,
        e_type_count = 3
    };

    ///////////////////////

    class b3Body;

}


struct box3d::b3ViewData {

    /**
     * @brief Vertices of the mesh
     * @details Each row is vertex's x, y, z coordinates
     */
    b3MatrixXd m_V;

    /**
     * @brief Faces of the mesh
     * @details Each row is a face with 3 indexes to vertex
     */
    b3MatrixXi m_F;

    b3MatrixXd vertexes() const {
        return m_V;
    }

    b3MatrixXi faces() const {
        return m_F;
    }

};


class box3d::b3Shape {

    b3ShapeType m_type = e_type_not_defined;

    /**
     * The radius of the shape.
     * This is used for extending the shape for collision test
     */
    double m_radius = 0;

protected:

    /**
     * @brief Next mesh in the list
     * This is used for managing memory of meshes.
     */
    b3Shape* m_next = nullptr;

    /**
     * @brief The body that the mesh belongs to.
     */
    b3Body* m_body = nullptr;

public:

    virtual ~b3Shape() = default;

    virtual int32 get_child_count() const {
        return 0;
    };

    virtual void get_bound_aabb(b3AABB* aabb, const b3PoseD& xf, int32 childIndex) const {
        b3_NOT_USED(aabb);
        b3_NOT_USED(xf);
        b3_NOT_USED(childIndex);
    };


    virtual void compute_mass_properties(b3MassProperty& mass_data, float density) const {
        b3_NOT_USED(mass_data);
        b3_NOT_USED(density);
    };

    virtual void get_view_data(b3ViewData* view_data) const {
        b3_NOT_USED(view_data);
    };

    virtual b3Shape* clone() const {
        return nullptr;
    };

    b3ShapeType get_type() const {
        return m_type;
    }

    void set_relative_body(b3Body* body) {
        m_body = body;
    }

    b3Shape* next() const {
        return m_next;
    }

    void set_next(b3Shape* next) {
        m_next = next;
    }
};

#endif //BOX3D_B3_SHAPE_CPP
