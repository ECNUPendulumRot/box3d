
#ifndef BOX3D_B3_SHAPE_HPP
#define BOX3D_B3_SHAPE_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"

#include "common/b3_block_allocator.hpp"

/////////// Forward Delaration ///////////

class b3Body;

class b3AABB;

class b3MassProperty;

//////////////////////////////////////////


enum b3ShapeType {
    e_type_not_defined = -1,
    e_mesh       = 0,
    e_cube       = 1,
    e_sphere     = 2,
    e_type_count = 3
};


struct b3ViewData {

    /**
     * @brief Vertices of the mesh
     * @details Each row is vertex's x, y, z coordinates
     */
    double* m_V = nullptr;
    int m_vertex_count = 0;

    /**
     * @brief Faces of the mesh
     * @details Each row is a face with 3 indexes to vertex
     */
    int* m_F = nullptr;

    int m_face_count = 0;

    typedef  struct {
        int fv0;
        int fv1;
        int fv2;
    } FaceIndice;

    inline void set_vertex_row(int vertex_index, const b3Vector3d& vertex) const {
        *(m_V + 3 * vertex_index)     = vertex.m_x;
        *(m_V + 3 * vertex_index + 1) = vertex.m_y;
        *(m_V + 3 * vertex_index + 2) = vertex.m_z;
    }

    inline void set_face_row(int face_index, const FaceIndice& face) const {
        *(m_F + 3 * face_index)     = face.fv0;
        *(m_F + 3 * face_index + 1) = face.fv1;
        *(m_F + 3 * face_index + 2) = face.fv2;
    }

    bool is_empty() const {
        return m_vertex_count == 0 || m_face_count == 0;
    }
};


class b3Shape {

protected:

    b3ShapeType m_type = e_type_not_defined;

    /**
     * The radius of the shape.
     * This is used for extending the AABB of the shape for collision test
     */
    double m_radius = 0;

    /**
     * @brief Next mesh in the list
     * This is used for managing memory of meshes.
     */
    b3Shape* m_next = nullptr;

    /**
     * @brief The body that the mesh belongs to.
     */
    b3Body* m_body = nullptr;

    /**
     * @brief This is used for gui
     */
    b3ViewData m_view_data;

    b3BlockAllocator* m_block_allocator = nullptr;

public:

    virtual ~b3Shape() {
        m_block_allocator->free(m_view_data.m_V, 3 * sizeof(double) * m_view_data.m_vertex_count);
        m_block_allocator->free(m_view_data.m_F, 3 * sizeof(int) * m_view_data.m_face_count);
    }

    virtual int32 get_child_count() const {
        return 0;
    };

    virtual void get_bound_aabb(b3AABB* aabb, const b3TransformD& xf, int32 child_index) const {
        b3_NOT_USED(aabb);
        b3_NOT_USED(xf);
        b3_NOT_USED(child_index);
    };

    virtual void compute_mass_properties(b3MassProperty& mass_data, double density) const {
        b3_NOT_USED(mass_data);
        b3_NOT_USED(density);
    };

    b3ViewData get_view_data(const b3TransformD& xf) {
        if(m_view_data.is_empty()) {
            init_view_data();
        }

        // object maybe has velocity
        setup_view_data(xf);

        return m_view_data;
    }

    virtual void init_view_data() {

    }

    virtual void setup_view_data(const b3TransformD& xf) {
        b3_NOT_USED(xf);
    }

    virtual b3Shape* clone() const {
        return nullptr;
    };

    b3ShapeType get_type() const {
        return m_type;
    }

    void set_relative_body(b3Body* body) {
        m_body = body;
    }

    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
    }

    b3Shape* next() const {
        return m_next;
    }

    void set_next(b3Shape* next) {
        m_next = next;
    }

    virtual double get_radius() const {
        return m_radius;
    }

    b3Body* get_body() const {
        return m_body;
    }

};


#endif //BOX3D_B3_SHAPE_HPP