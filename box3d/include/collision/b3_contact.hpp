#ifndef B3_CONTACT_HPP
#define B3_CONTACT_HPP

#include "common/b3_allocator.hpp"
#include "geometry/b3_shape.hpp"
#include "collision/b3_collision.hpp"

namespace box3d {

    class b3Contact;

    class b3ContactEdge;

    struct b3ContactRegister;

    /////////////////////

    class b3Fixture;

    class b3Body;
}

struct box3d::b3ContactEdge {

    b3Body* m_other;

    b3Contact* m_contact;

    b3ContactEdge* m_prev;

    b3ContactEdge* m_next;

};


typedef box3d::b3Contact* b3ContactCreateFcn(box3d::b3Fixture* fixture_A, int32 index_A,
                                             box3d::b3Fixture* fixture_B, int32 index_B);

typedef void b3ContactDestroyFcn(box3d::b3Contact* contact);

struct box3d::b3ContactRegister {

    b3ContactCreateFcn* create_fcn;
    b3ContactDestroyFcn* destroy_fcn;
    bool primary;

};


class box3d::b3Contact {

    b3Fixture* m_fixture_a;
    b3Fixture* m_fixture_b;

    b3ContactEdge m_node_a;
    b3ContactEdge m_node_b;

    int32 m_index_a;
    int32 m_index_b;

    ///////// for general linked list /////////
    b3Contact* m_prev;
    b3Contact* m_next;

    b3Manifold m_manifold;

    static b3ContactRegister s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];

    static bool s_initialized;

public:

    b3Contact(b3Fixture* f_A, int32 index_A, b3Fixture* f_B, int32 index_B);

    inline b3Fixture* get_fixture_a() const {
        return m_fixture_a;
    }

    inline b3Fixture* get_fixture_b() const {
        return m_fixture_b;
    }

    inline b3Contact* get_prev() const {
        return m_prev;
    }

    inline b3Contact* get_next() const {
        return m_next;
    }

    inline void set_next(b3Contact* next) {
        m_next = next;
    }
    
    inline void set_prev(b3Contact* prev) {
        m_prev = prev;
    }

    inline void set_fixture_a(b3Fixture* fixture_a) {
        m_fixture_a = fixture_a;
    }

    inline void set_fixture_b(b3Fixture* fixture_b) {
        m_fixture_b = fixture_b;
    }

    inline int32 get_child_index_a() const {
        return m_index_a;
    }

    inline int32 get_child_index_b() const {
        return m_index_b;
    }

    inline b3Manifold* get_manifold() {
        return &m_manifold;
    }

    virtual void evaluate(b3Manifold* manifold, const b3TransformD& xfA, const b3TransformD& xfB) = 0;

protected:

    static void initialize_registers();

    static void add_type(b3ContactCreateFcn* create_fcn, b3ContactDestroyFcn* destroy_fcn,
                         b3ShapeType type_A, b3ShapeType type_B);

    static b3Contact* create(b3Fixture* fixture_A, int32 index_A, b3Fixture* fixture_B, int32 index_B);

    static void destroy(b3Contact* contact);
};




#endif