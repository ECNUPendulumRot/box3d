
#ifndef B3_CONTACT_MANAGER
#define B3_CONTACT_MANAGER


#include "common/b3_types.hpp"
#include "collision/b3_broad_phase.hpp"
#include "common/b3_world_callback.hpp"


/////////// Forward Delaration ///////////

class b3FixtureProxy;

class b3Contact;

class b3BlockAllocator;

//////////////////////////////////////////


class b3ContactManager {

public:

    friend class b3World;

    b3BroadPhase m_broad_phase;

    b3Contact* m_contact_list = nullptr;

    b3Contact* m_static_contact_list = nullptr;

    int32 m_contact_count = 0;
    int32 m_static_contact_count = 0;
    int32 m_normal_contact_count = 0;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3ContactListener* m_contact_listener = nullptr;

    void find_new_contact();

    // Broad-phase callback, add a contact
    void add_pair(b3FixtureProxy* fixture_proxy_a, b3FixtureProxy* fixture_proxy_b);

    /**
     * destroy a contact and remove it from the contact list
     */
    void destroy(b3Contact* contact);

    /*
     * @brief determine if the aabbs of two fixtures are overlapping,
     * if overlapping, will generate manifold.
     * if not overlapping, will destroy the contact.
     */
    void collide();

    b3BroadPhase* get_broad_phase() {
        return &m_broad_phase;
    }

    int32 get_contact_count() const {
        return m_contact_count;
    }

    b3Contact* get_contact_list() const {
        return m_contact_list;
    }

    b3Contact* get_static_contact_list() const {
        return m_static_contact_list;
    }

    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
        m_broad_phase.set_block_allocator(block_allocator);
    }
};


#endif