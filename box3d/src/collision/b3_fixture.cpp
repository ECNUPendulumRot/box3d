
#include "collision/b3_fixture.hpp"

#include "collision/b3_broad_phase.hpp"
#include "dynamics/b3_body.hpp"

#include "common/b3_block_allocator.hpp"


void b3Fixture::create_fixture(
  b3BlockAllocator *block_allocator,
  const b3FixtureDef &f_def, b3Body *body)
{
    m_restitution = f_def.m_restitution;
    m_friction = f_def.m_friction;
    m_density = f_def.m_density;
    m_rolling_friction = f_def.m_rolling_friction;
    m_spinning_friction = f_def.m_spinning_friction;
    m_body = body;
    m_local_transform = f_def.m_local_transform;

    f_def.m_shape->set_block_allocator(block_allocator);
    m_shape = f_def.m_shape->clone();
    m_shape->set_relative_body(body);

    int32 child_count = m_shape->get_child_count();

    // m_proxies has as many proxies as number of children shapes
    m_proxies = (b3FixtureProxy *)block_allocator->allocate(child_count * sizeof(b3FixtureProxy));

    for (int32 i = 0; i < child_count; ++i) {
        m_proxies[i].m_fixture = nullptr;
        m_proxies[i].m_proxy_id = b3FixtureProxy::b3NullProxy;
    }

    m_proxy_count = 0;
}

b3Transformr b3Fixture::get_world_transform(const b3Transformr &xf) {
    // first transform local vertices into body space
    // second transform body space vertices into world space
    b3Transformr world_xf(xf.rotation_matrix() * m_local_transform.position() + xf.position(),
                    xf.rotation_matrix() * m_local_transform.rotation_matrix());
    return world_xf;
}

void b3Fixture::create_proxy(b3BroadPhase *broad_phase, b3Transformr &m_xf)
{
    b3_assert(m_proxy_count == 0);
    m_proxy_count = m_shape->get_child_count();

    b3Transformr xf = get_world_transform(m_xf);

    // create proxy for each child shape
    for (int32 i = 0; i < m_proxy_count; ++i) {
        b3FixtureProxy *proxy = m_proxies + i;
        m_shape->get_bound_aabb(&proxy->m_aabb, xf, i);
        proxy->m_proxy_id = broad_phase->create_proxy(proxy->m_aabb, proxy);
        proxy->m_fixture = this;
        proxy->m_child_id = i;
    }
}


void b3Fixture::synchronize(b3BroadPhase *broad_phase, const b3Transformr &transform1, const b3Transformr &transform2)
{
    if(m_proxy_count == 0) {
        return;
    }

    b3Transformr world_transform1 = get_world_transform(transform1);
    b3Transformr world_transform2 = get_world_transform(transform2);

    // the body position is updated, so we need to update the proxy's AABBs
    for(int32 i = 0; i <  m_proxy_count; ++i) {
        b3FixtureProxy* proxy = m_proxies + i;

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        b3AABB aabb1, aabb2;
        m_shape->get_bound_aabb(&aabb1, world_transform1, proxy->m_child_id);
        m_shape->get_bound_aabb(&aabb2, world_transform2, proxy->m_child_id);

        proxy->m_aabb.combine(aabb1, aabb2);

  	    broad_phase->move_proxy(proxy->m_proxy_id, proxy->m_aabb);
    }
}
