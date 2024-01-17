
#ifndef B3_CONTACT_MANAGER
#define B3_CONTACT_MANAGER

#include "common/b3_types.hpp"
#include "collision/b3_broad_phase.hpp"

namespace box3d {
    class b3Contact;
    class b3Fixture;
    class b3FixtureProxy;

    class b3ContactManager;
}

class box3d::b3ContactManager {

    b3BroadPhase m_broad_phase;

    b3Contact* m_contact_list = nullptr;

    int32 m_contact_count = 0;

public:

    void find_new_contact();

    // Broad-phase callback
    void add_pair(b3FixtureProxy* fixture_proxy_a, b3FixtureProxy* fixture_proxy_b);

    void destory(b3Contact* contact);

    // TODO
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

};


#endif