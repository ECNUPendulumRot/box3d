
#ifndef BOX3D_B3_POSE_HPP
#define BOX3D_B3_POSE_HPP


#include <Eigen/Geometry>

#include "common/b3_types.hpp"

#include "math/b3_matrix.hpp"


namespace box3d {

    template <typename T>
    class b3Transform;

}


template <typename T>
class box3d::b3Transform {

    /**
     * The position part of the pose.
     */
    b3Vector3<T> m_p;

    /**
     * The orientation part of the pose.
     */
    b3Vector3<T> m_r;

    /**
     * The rotation matrix of the pose.
     */
    b3Matrix3d m_r_t;

public:

    /**
     * @brief Construct a new b3Transform object
     */
    b3Transform();

    b3Transform(const b3Transform& other) {
        m_p = other.m_p;
        m_r = other.m_r;
    }

    /**
     * @brief Construct a new b3Transform object
     * @param position
     * @param rotation
     */
    b3Transform(Eigen::Vector3<T> position, Eigen::Vector3<T> rotation);

    /**
     * @brief Construct a new b3Transform object
     * @param x: The x position
     * @param y: The y position
     * @param z: The z position
     * @param r_x: The x rotation theta
     * @param r_y: The y rotation theta
     * @param r_z: The z rotation theta
     */
    b3Transform(const T& x,
                const T& y,
                const T& z,
                const T& r_x,
                const T& r_y,
                const T& r_z);

    inline void set_linear(b3Vector3<T> p){
        m_p = p;
    };

    inline void set_linear(const T& x, const T& y, const T& z){
        m_p = b3Vector3<T>(x, y, z);
    };

    inline void set_angular(b3Vector3<T> p){
        m_r = p;
        m_r_t = rotation_matrix();
    };

    inline void set_angular(const T& x, const T& y, const T& z){
        m_r = b3Vector3<T>(x, y, z);
        m_r_t = rotation_matrix();
    };

    inline b3Vector3<T> linear() const {
        return m_p;
    };

    inline b3Vector3<T> angular() const {
        return m_r;
    };

    inline b3Matrix3<T> rotation_matrix_b3() const {
        return m_r_t;
    };

    inline E3Matrix3<T> rotation_matrix() const {
        if (m_r.is_zero()) {
            return Eigen::Matrix3<T>::Identity();
        } else {
            Eigen::AngleAxis<T> angle_axis(m_r.length(), m_r.normalized().eigen_vector3());
            return angle_axis.toRotationMatrix();
        }
    }

    static b3Transform<T> zero() {
        return b3Transform<T>(T(0), T(0), T(0), T(0), T(0), T(0));
    };

    inline b3Vector3<T> transform(const b3Vector3<T>& v) const {
        return b3Vector3<T>(rotation_matrix() * v.eigen_vector3() + m_p.eigen_vector3());
    }

    inline b3Vector3d<T> transform_local(const b3Vector3d<T>& v) const {
        return b3Vector3d<T>((v - m_p).eigen_vector3() * rotation_matrix());
    }

};


using b3TransformF = box3d::b3Transform<float>;
using b3TransformD = box3d::b3Transform<double>;


#endif //BOX3D_B3_POSE_HPP
