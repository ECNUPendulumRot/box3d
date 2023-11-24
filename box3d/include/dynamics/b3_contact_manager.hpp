
#ifndef B3_CONTACT_MANAGER
#define B3_CONTACT_MANAGER

#include "common/b3_types.hpp"

namespace box3d {

    class b3Contact;
    class b3BroadPhase;
    class b3Fixture;

    class b3ContactManager;
}

class box3d::b3ContactManager {
    b3BroadPhase m_broadPhase;
    b3Contact* m_contact_list;
    int32 m_contact_count = 0;

public:
    void FindNewContact();

    // Broad-phase callback
    void add_pair(b3FixtureProxy* fixture_proxy_a, b3FixtureProxy* fixture_proxy_b);

    void destory(b3Contact* contact);

    // TODO
    void collide();
};


#endif