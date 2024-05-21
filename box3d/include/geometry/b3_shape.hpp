
#ifndef BOX3D_B3_SHAPE_HPP
#define BOX3D_B3_SHAPE_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Delaration ///////////

class b3Body;

class b3AABB;

class b3MassProperty;

class b3BlockAllocator;


//////////////////////////////////////////


enum b3ShapeType {
    e_type_not_defined = -1,
    e_cube       = 0,
    e_sphere     = 1,
    e_plane      = 2,
    e_type_count = 3
};


class b3Shape {

protected:

    b3ShapeType m_type = e_type_not_defined;

    /**
     * The radius of the shape.
     * This is used for extending the AABB of the shape for collision test
     */
    real m_radius = 0;

    /**
     * @brief Next mesh in the list
     * This is used for managing memory of meshes.
     */
    b3Shape* m_next = nullptr;

    /**
     * @brief The body that the mesh belongs to.
     */
    b3Body* m_body = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    /**
     * @brief This is used for gui
     */
    b3Vec3r m_color = b3Vec3r::zero();

public:

    virtual ~b3Shape() = default;

    virtual int32 get_child_count() const {
        return 0;
    };

    virtual void get_bound_aabb(b3AABB* aabb, const b3Transformr& xf, int32 child_index) const {
        b3_NOT_USED(aabb);
        b3_NOT_USED(xf);
        b3_NOT_USED(child_index);
    };

    virtual void compute_mass_properties(b3MassProperty& mass_data, real density) const {
        b3_NOT_USED(mass_data);
        b3_NOT_USED(density);
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

    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
    }

    b3Shape* next() const {
        return m_next;
    }

    void set_next(b3Shape* next) {
        m_next = next;
    }

    virtual real get_radius() const {
        return m_radius;
    }

    b3Body* get_body() const {
        return m_body;
    }

    void set_color(const b3Vec3r& color) {
        m_color = color;
    }

    b3Vec3r get_color() const {
        return m_color;
    }
};


#endif //BOX3D_B3_SHAPE_HPP
