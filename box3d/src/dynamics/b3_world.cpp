
#include "dynamics/b3_world.hpp"
#include "dynamics/b3_rigid_body.hpp"

#include "common/b3_allocator.hpp"

box3d::b3World::b3World():
    m_body_list(nullptr),
    m_body_count(0)
{
    ;
}


box3d::b3Body* box3d::b3World::create_body(const box3d::b3BodyDef &def)
{
    b3Body* body = nullptr;

    switch (def.get_type()) {

        case b3BodyType::b3_RIGID:
            body = new b3RigidBody(def);
            break;

        default:
            break;
    }

    if (body == nullptr)
        return nullptr;

    body->set_next(m_body_list);
    m_body_list = body;
    ++m_body_count;

    return body;
}


void box3d::b3World::test_step()
{
    b3Body* body = m_body_list;

    double delta_t  = 1.0 / m_hz;

    while (body != nullptr) {
        body->test_step_by_force(delta_t);
        body = body->next();
    }
}


