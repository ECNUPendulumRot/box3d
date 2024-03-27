
#ifndef BOX3D_B3_TRANSFORM_HPP
#define BOX3D_B3_TRANSFORM_HPP


#include "common/b3_types.hpp"

#include "math/b3_matrix.hpp"
#include "math/b3_quaternion.hpp"


template <typename T>
struct b3Transform {

public:

    // The position part of the pose.
    b3Vector3<T> m_p;

    // The rotation matrix of the pose.
    b3Matrix3<T> m_r_t;

    b3Transform() = default;

    b3Transform(const b3Transform& other) {
        m_p = other.m_p;
        m_r_t = other.m_r_t;
    }

    b3Transform(const b3Vector3<T>& p, const b3Quaternion<T>& q) {
        m_p = p;
        m_r_t = q.rotation_matrix();
    }

    inline b3Matrix3<T> rotation_matrix() const {
        return m_r_t;
    };

    inline b3Vector3<T> transform(const b3Vector3<T>& v) const {
        return m_r_t * v + m_p;
    }

    inline b3Vector3<T> transform_local(const b3Vector3<T>& v) const {
        return m_r_t.transpose() * (v - m_p);
    }

    inline b3Vector3<T> position() const {
        return m_p;
    }

};


/////////////////////////////////////////////////////////////////////


//template<typename T>
//b3Matrix3<T> b3Transform<T>::rotation_matrix() const {
//    if (m_r.is_zero()) {
//        return b3Matrix3<T>::identity();
//    }
//
//    // Rodrigues' rotation formula:
//    // R = I + sin(theta) * K + (1 - cos(theta)) * K^2
//    T l = m_r.length();
//    b3Vector3<T> v = m_r / l;
//
//    b3Matrix3<T> res;
//    b3Vector3<T> sin_axis = sin(l) * v;
//    T c = cos(l);
//    b3Vector3<T> cos1_axis = (T(1) - c) * v;
//
//    T tmp = cos1_axis.x() * v.y();
//    res(0, 1) = tmp - sin_axis.z();
//    res(1, 0) = tmp + sin_axis.z();
//
//    tmp = cos1_axis.x() * v.z();
//    res(0, 2) = tmp + sin_axis.y();
//    res(2, 0) = tmp - sin_axis.y();
//
//    tmp = cos1_axis.y() * v.z();
//    res(1, 2) = tmp - sin_axis.x();
//    res(2, 1) = tmp + sin_axis.x();
//
//    cos1_axis = cos1_axis.cwise_product(v);
//    res(0, 0) = cos1_axis.x() + c;
//    res(1, 1) = cos1_axis.y() + c;
//    res(2, 2) = cos1_axis.z() + c;
//
//    return res;
//}


using b3Transformf = b3Transform<float>;
using b3Transformd = b3Transform<double>;
using b3Transformr = b3Transform<real>;

#endif //BOX3D_B3_TRANSFORM_HPP
