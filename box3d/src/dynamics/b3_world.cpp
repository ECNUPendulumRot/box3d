
#include "dynamics/b3_world.hpp"


box3d::b3World::b3World():
    m_body_list(nullptr),
    m_body_count(0),
    m_gravity(b3Vector3d(0.0, 0.0, -9.8))
{
    ;
}
