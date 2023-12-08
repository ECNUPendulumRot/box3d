
#include "dynamics/b3_body_affine.hpp"


box3d::b3BodyAffine::b3BodyAffine()
{
    m_q.setZero();

    m_q[3]  = 1.0;
    m_q[7]  = 1.0;
    m_q[11] = 1.0;
}
