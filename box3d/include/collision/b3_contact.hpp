
#ifndef BOX3D_CONTACT_HPP
#define BOX3D_CONTACT_HPP

#include "geometry/b3_shape.hpp"
#include "collision/b3_collision.hpp"

#include "common/b3_block_allocator.hpp"

/////////// Forward Delaration ///////////

class b3Body;
class b3Contact;
class b3Fixture;
class b3ContactListener;

//////////////////////////////////////////


struct b3ContactEdge {
    /**
     * the shape is contact with m_other.
     */
    b3Body* m_other = nullptr;
    /**
     * this contact
     */
    b3Contact* m_contact = nullptr;

    b3ContactEdge* m_prev = nullptr;

    b3ContactEdge* m_next = nullptr;

};

// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
inline real b3_mix_friction(real friction1, real friction2) {
    return b3_sqrt(friction1 * friction2);
}

// Restitution mixing law. The idea is to allow for anything to bounce off an inelastic surface.
inline real b3_mix_restitution(real restitution1, real restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

// Restitution mixing law. This picks the lowest value.
inline float b3_mix_restitution_threshold(float threshold1, float threshold2)
{
    return threshold1 < threshold2 ? threshold1 : threshold2;
}

typedef b3Contact* b3ContactCreateFcn(b3Fixture* fixture_A, int32 index_A,
                                      b3Fixture* fixture_B, int32 index_B,
                                      b3BlockAllocator* block_allocator);

typedef void b3ContactDestroyFcn(b3Contact* contact, b3BlockAllocator* block_allocator);


struct b3ContactRegister {

    b3ContactCreateFcn* create_fcn;
    b3ContactDestroyFcn* destroy_fcn;
    bool primary;

};


class b3Contact {

public:

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

    bool is_static_collide = false;

    static b3ContactRegister s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];

    static bool s_initialized;


    ////////// Coefficients related to the material of the object ///////////

    real m_restitution;

    real m_friction;

    real m_restitution_threshold;

    uint32 m_flags = 0;

    int32 m_toi_count = 0;
    real m_toi;

    b3Contact(b3Fixture* f_A, int32 index_A, b3Fixture* f_B, int32 index_B);

    enum {
        // this is used to generate island.
        e_island_flag = 1,

        // when aabb overlap, but two shapes maybe not intersecting.
        // so this flag is set when two shapes are intersecting or touching.
        e_touching_flag = 1 << 1,

        e_toi_flag = 1 << 2
    };

    void set_flag(uint32 flag) {
        m_flags |= flag;
    }

    void unset_flag(uint32 flag) {
        m_flags &= ~flag;
    }

    // test of a flag is set
    bool test_flag(uint32 flag) {
        if(m_flags & flag) {
            return true;
        }
        return false;
    }

    inline b3Fixture* get_fixture_a() const {
        return m_fixture_a;
    }

    inline b3Fixture* get_fixture_b() const {
        return m_fixture_b;
    }

    b3ContactEdge* get_node_a() {
        return &m_node_a;
    }

    b3ContactEdge* get_node_b() {
        return &m_node_b;
    }

    inline b3Contact* prev() const {
        return m_prev;
    }

    inline b3Contact* next() const {
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

    real get_restitution() const {
        return m_restitution;
    }

    real get_restitution_threshold() const {
        return m_restitution_threshold;
    }

    real get_friction() const {
        return m_friction;
    }

    void get_world_manifold(b3WorldManifold* world_manifold) const;

    // generate manifold between two shapes
    virtual void evaluate(b3Manifold* manifold, const b3Transr& xfA, const b3Transr& xfB) = 0;

protected:

    friend class b3ContactManager;

    static void initialize_registers();

    static void add_type(b3ContactCreateFcn* create_fcn, b3ContactDestroyFcn* destroy_fcn,
                         b3ShapeType type_A, b3ShapeType type_B);

    static b3Contact* create(b3Fixture* fixture_A, int32 index_A, b3Fixture* fixture_B, int32 index_B, b3BlockAllocator* block_allocator);

    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    void update(b3ContactListener* listener);

};


#endif // BOX3D_CONTACT_HPP