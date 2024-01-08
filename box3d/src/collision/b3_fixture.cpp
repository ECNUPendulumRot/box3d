
#include "collision/b3_fixture.hpp"

#include "collision/b3_broad_phase.hpp"
#include "dynamics/b3_body.hpp"


void box3d::b3Fixture::create_fixture(const box3d::b3FixtureDef &f_def, box3d::b3Body *body)
{
    m_restitution = f_def.get_restitution();
    m_friction = f_def.get_friction();

    m_body = body;

    m_shape = f_def.get_shape()->clone();

    m_proxies = (b3FixtureProxy*)b3_alloc(sizeof(b3FixtureProxy));

    int32 child_count = m_shape->get_child_count();

    m_proxies = (b3FixtureProxy*)b3_alloc(child_count * sizeof(b3FixtureProxy));

    for (int32 i = 0; i < child_count; ++i) {
        m_proxies[i].m_fixture = nullptr;
        m_proxies[i].m_proxy_id = b3FixtureProxy::b3NullProxy;
    }

    m_proxy_count = 0;
}


void box3d::b3Fixture::create_proxy(b3BroadPhase* broad_phase, b3PoseD& pose)
{
    b3_assert(m_proxy_count == 0);
    m_proxy_count = m_shape->get_child_count();

    for (int32 i = 0; i < m_proxy_count; ++i)
    {
        b3FixtureProxy* proxy = m_proxies + i;
        m_shape->get_bound_aabb(&proxy->m_aabb, pose, i);
        proxy->m_proxy_id = broad_phase->create_proxy(m_proxies->m_aabb, m_proxies);
        proxy->m_fixture = this;
        proxy->m_child_id = i;
    }

}
