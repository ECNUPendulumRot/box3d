
#ifndef BOX3D_B3_TRANSFORM_HPP
#define BOX3D_B3_TRANSFORM_HPP


#include "common/b3_types.hpp"

#include "math/b3_mat33.hpp"
#include "math/b3_quat.hpp"


template <typename T>
struct b3Trans {

public:

    // The position part of the pose.
    b3Vec3<T> m_p;

    // The rotation matrix of the pose.
    b3Mat33<T> m_r;

    b3Trans() = default;

    b3Trans(const b3Trans& other) {
        m_p = other.m_p;
        m_r = other.m_r;
    }

    b3Trans(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r = q.rotation_matrix();
    }

    void set(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r = q.rotation_matrix();
    }

    void set_position(const b3Vec3<T>& p) {
        m_p = p;
    }

    void set_euler_angles(const b3Vec3<T>& euler) {
        b3Quat<T> q(euler);
        m_r = q.rotation_matrix();
    }


    inline const b3Mat33<T>& rotation_matrix() const {
        return m_r;
    };

    inline b3Vec3<T> transform(const b3Vec3<T>& v) const {
        return m_r * v + m_p;
    }

    inline b3Vec3<T> rotate(const b3Vec3<T>& v) const {
        return m_r * v;
    }

    inline b3Vec3<T> transform_local(const b3Vec3<T>& v) const {
        return m_r.transpose() * (v - m_p);
    }

    inline b3Vec3<T> position() const {
        return m_p;
    }

};


/////////////////////////////////////////////////////////////////////


//template<typename T>
//b3Mat33<T> b3Transform<T>::rotation_matrix() const {
//    if (m_r.is_zero()) {
//        return b3Mat33<T>::identity();
//    }
//
//    // Rodrigues' rotation formula:
//    // R = I + sin(theta) * K + (1 - cos(theta)) * K^2
//    T l = m_r.length();
//    b3Vec3<T> v = m_r / l;
//
//    b3Mat33<T> res;
//    b3Vec3<T> sin_axis = sin(l) * v;
//    T c = cos(l);
//    b3Vec3<T> cos1_axis = (T(1) - c) * v;
//
//    T tmp = cos1_axis.x * v.y;
//    res(0, 1) = tmp - sin_axis.z;
//    res(1, 0) = tmp + sin_axis.z;
//
//    tmp = cos1_axis.x * v.z;
//    res(0, 2) = tmp + sin_axis.y;
//    res(2, 0) = tmp - sin_axis.y;
//
//    tmp = cos1_axis.y * v.z;
//    res(1, 2) = tmp - sin_axis.x;
//    res(2, 1) = tmp + sin_axis.x;
//
//    cos1_axis = cos1_axis.cwise_product(v);
//    res(0, 0) = cos1_axis.x + c;
//    res(1, 1) = cos1_axis.y + c;
//    res(2, 2) = cos1_axis.z + c;
//
//    return res;
//}


using b3Transf = b3Trans<float>;
using b3Transd = b3Trans<double>;
using b3Transr = b3Trans<real>;

#endif //BOX3D_B3_TRANSFORM_HPP
