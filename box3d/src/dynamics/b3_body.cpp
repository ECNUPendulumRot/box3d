
#include "dynamics/b3_body.hpp"

#include "dynamics/b3_world.hpp"

#include "collision/b3_fixture.hpp"


box3d::b3Fixture* box3d::b3Body::create_fixture(const box3d::b3FixtureDef &def) {

    void* memory = b3_alloc(sizeof(b3Fixture));
    b3Fixture* fixture = new(memory) b3Fixture;
    fixture->create_fixture(def, this);
    fixture->create_rigid_proxy(m_world->get_broad_phase());

    return fixture;
}
