
#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"


void box3d::b3Fixture::create_fixture(const box3d::b3FixtureDef &f_def, box3d::b3Body *body)
{
    m_restitution = f_def.get_restitution();
    m_friction = f_def.get_friction();
    m_body_mesh = body->mesh();
    m_body = body;

    m_proxy = (b3FixtureProxy*)b3_alloc(sizeof(b3FixtureProxy));

    m_proxy->m_proxy_id = b3FixtureProxy::b3NullProxy;

}


// TODO: Implement this proxy
void box3d::b3Fixture::create_rigid_proxy()
{
    m_proxy->m_aabb = m_body_mesh->get_bounding_aabb();
    m_proxy->m_fixture = this;
    m_proxy->m_proxy_id = m_body->m_world->m_broad_phase.create_proxy(m_proxy->m_aabb, m_proxy);
}
