
#include "dynamics/b3_world.hpp"
#include "dynamics/b3_rigid_body.hpp"

#include "common/b3_allocator.hpp"

box3d::b3World::b3World():
    m_rigid_body_list(nullptr),
    m_body_count(0)
{
    ;
}


box3d::b3Body* box3d::b3World::create_body(const box3d::b3BodyDef &def)
{
    //b3Body* body = nullptr;

    switch (def.get_type()) {

        case b3BodyType::b3_RIGID:
            body = new b3BodyRigid(def);
            body->set_next(m_rigid_body_list);
            m_rigid_body_list = body;
            ++m_body_count;
            break;

        default:
            break;
    }

    if (body == nullptr)
        return nullptr;

//    body->set_next(m_body_list);
//    m_body_list = body;
//    ++m_body_count;

    return body;
}


void box3d::b3World::test_step()
{
    b3Body* body = m_body_list;

    double delta_t  = 1.0 / m_hz;

    solve_rigid(delta_t);

}


void box3d::b3World::solve_rigid(double delta_t) {
    b3Body *body = m_rigid_body_list;

    while (body != nullptr) {

        auto* rigid_body = (b3BodyRigid*) body;

        auto rigid_pose = rigid_body->get_pose();
        auto rigid_velocity = rigid_body->get_velocity();

        rigid_velocity.set_linear(rigid_velocity.linear() + m_gravity * rigid_body->m_inv_mass * delta_t);

        rigid_pose.set_linear(rigid_pose.linear() + rigid_velocity.linear() * delta_t);

        rigid_body->set_pose(rigid_pose);
        rigid_body->set_velocity(rigid_velocity);

        body = body->next();
    }
}


box3d::b3BodyRigid* box3d::b3World::create_rigid_body(const box3d::b3BodyDefRigid &def)
{
    void* memory = b3_alloc(sizeof (b3BodyRigid));
    b3BodyRigid* body = new(memory) b3BodyRigid(def);
    body->set_next(m_rigid_body_list);
    m_rigid_body_list = body;
    ++m_body_count;
}

