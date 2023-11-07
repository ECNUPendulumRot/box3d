
#include "dynamics/b3_pose.hpp"


template <typename T>
box3d::b3Pose<T>::b3Pose()
{
    m_p.set_zero();
    m_r.set_zero();
    m_rel_p = nullptr;
}


template<typename T>
box3d::b3Pose<T>::b3Pose(const T &x,
                         const T &y,
                         const T &z,
                         const T &r_x,
                         const T &r_y,
                         const T &r_z):
    m_p(b3Vector3<T>(x,   y,   z)),
    m_r(b3Vector3<T>(r_x, r_y, r_z)),
    m_rel_p(nullptr)
{
    ;
}


template class box3d::b3Pose<float>;
template class box3d::b3Pose<double>;