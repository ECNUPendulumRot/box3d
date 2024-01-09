
#include <Eigen/Geometry>

#include "dynamics/b3_pose.hpp"



template <typename T>
box3d::b3Transform<T>::b3Transform()
{
    m_p.set_zero();
    m_r.set_zero();
}


template<typename T>
box3d::b3Transform<T>::b3Transform(const T &x,
                                   const T &y,
                                   const T &z,
                                   const T &r_x,
                                   const T &r_y,
                                   const T &r_z):
    m_p(b3Vector3<T>(x,   y,   z)),
    m_r(b3Vector3<T>(r_x, r_y, r_z))
{
    ;
}


template<typename T>
box3d::b3Transform<T>::b3Transform(Eigen::Vector3<T> position, Eigen::Vector3<T> rotation)
{
    m_p = b3Vector3<T>(position);
    m_r = b3Vector3<T>(rotation);
}


template class box3d::b3Transform<float>;
template class box3d::b3Transform<double>;