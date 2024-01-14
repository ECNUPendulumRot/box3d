
#include "dynamics/b3_inertia.hpp"


box3d::b3Inertia::b3Inertia():
    m_I(E3Matrix3d::Identity()),
    m_rel_p(nullptr)
{
    ;
}


