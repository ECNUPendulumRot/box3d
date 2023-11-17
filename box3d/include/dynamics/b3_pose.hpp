
#ifndef BOX3D_B3_POSE_HPP
#define BOX3D_B3_POSE_HPP


#include "common/b3_types.hpp"

namespace box3d {
    template <typename T>
    class b3Pose;
}


template <typename T>
class box3d::b3Pose {

    /**
     * The position part of the pose relative to m_rel_p.
     */
    b3Vector3<T> m_p;

    /**
     * The orientation part of the pose relative to m_rel_p.
     */
    b3Vector3<T> m_r;

    /**
     * The relative pose of this pose.
     * If this is a nullptr, then just relative to world frame
     */
    b3Pose<T>* m_rel_p;

public:

    /**
     * @brief Construct a new b3Pose object
     */
    b3Pose();

    /**
     * @brief Construct a new b3Pose object
     * @param position
     * @param rotation
     */
    b3Pose(Eigen::Vector3<T> position, Eigen::Vector3<T> rotation);

    /**
     * @brief Construct a new b3Pose object
     * @param x: The x position
     * @param y: The y position
     * @param z: The z position
     * @param r_x: The x rotation theta
     * @param r_y: The y rotation theta
     * @param r_z: The z rotation theta
     */
    b3Pose(const T& x,
           const T& y,
           const T& z,
           const T& r_x,
           const T& r_y,
           const T& r_z);

    Eigen::Vector3<T> transform(const Eigen::Vector3<T>& v);

    inline void set_relative_pose(b3Pose<T>* rel_pose){
        m_rel_p = rel_pose;
    };

    inline void set_linear(b3Vector3<T> p){
        m_p = p;
    };

    inline void set_linear(const T& x, const T& y, const T& z){
        m_p = b3Vector3<T>(x, y, z);
    };

    inline void set_angular(b3Vector3<T> p){
        m_r = p;
    };

    inline void set_angular(const T& x, const T& y, const T& z){
        m_r = b3Vector3<T>(x, y, z);
    };

    inline b3Vector3<T> linear() const {
        return m_p;
    };

    inline b3Vector3<T> angular() const {
        return m_r;
    };

    inline b3Matrix3<T> rotation_matrix() const {
        if (m_r.is_zero()) {
            return Eigen::Matrix3<T>::Identity();
        } else {
            Eigen::AngleAxis<T> angle_axis(m_r.length(), m_r.normalized().eigen_vector3());
            return angle_axis.toRotationMatrix();
        }
    }

    static b3Pose<T> zero() {
        return b3Pose<T>(T(0), T(0), T(0), T(0), T(0), T(0));
    };

};


using b3PoseF = box3d::b3Pose<float>;
using b3PoseD = box3d::b3Pose<double>;

#endif //BOX3D_B3_POSE_HPP
