
#include <Eigen/Geometry>

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


template<typename T>
box3d::b3Pose<T>::b3Pose(Eigen::Vector3<T> position, Eigen::Vector3<T> rotation)
{
    m_p = b3Vector3<T>(position);
    m_r = b3Vector3<T>(rotation);
    m_rel_p = nullptr;
}


template<typename T>
Eigen::Vector3<T> box3d::b3Pose<T>::transform(const Eigen::Vector3<T> &v)
{
    // Initialize the rotation matrix
    static Eigen::Matrix3<T> rotation_matrix = [&]() -> Eigen::Matrix3<T>{
        if (m_r.is_zero()) {
            return Eigen::Matrix3<T>::Identity();
        } else {
            Eigen::AngleAxis<T> angle_axis(m_r.length(), m_r.normalized().eigen_vector3());
            return angle_axis.toRotationMatrix();
        }
    }();

    return rotation_matrix * v + m_p.eigen_vector3();
}


template class box3d::b3Pose<float>;
template class box3d::b3Pose<double>;