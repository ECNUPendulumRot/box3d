
#ifndef BOX3D_B3_QUATERNION_HPP
#define BOX3D_B3_QUATERNION_HPP


#include "b3_vector.hpp"


template <typename T>
struct b3Quaternion {

    union {
        T m_ts[4];
        struct {
            T m_w, m_x, m_y, m_z;
        };

    };

    b3Quaternion() {
        m_w = T(1);
        m_x = m_y = m_z = T(0);
    }

    explicit b3Quaternion(const b3Vector3<T>& v) {
        m_w = T(0);
        m_x = v.m_x;
        m_y = v.m_y;
        m_z = v.m_z;
    }

    b3Quaternion(const b3Quaternion& other) {
        m_w = other.m_w;
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }

    b3Quaternion(const T& w, const b3Vector3<T>& v) {
        m_w = w;
        m_x = v.m_x;
        m_y = v.m_y;
        m_z = v.m_z;
    }

    b3Quaternion(T w, T x, T y, T z) {
        m_w = w;
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline void set(T w, T x, T y, T z) {
        m_w = w;
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline void normalize() {
        T length = m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z;

        if (length == T(0)) {
            m_w = T(1);
            m_x = m_y = m_z = T(0);
            return;
        }

        length = T(1) / b3_sqrt(length);

        m_w *= length;
        m_x *= length;
        m_y *= length;
        m_z *= length;
    }

    inline b3Matrix3<T> rotation_matrix() const {
        T ww = m_w * m_w;
        T xx = m_x * m_x;
        T yy = m_y * m_y;
        T zz = m_z * m_z;
        T xy = m_x * m_y;
        T xz = m_x * m_z;
        T yz = m_y * m_z;
        T wx = m_w * m_x;
        T wy = m_w * m_y;
        T wz = m_w * m_z;

        b3Matrix3<T> result;

        result.m_11 = ww + xx - yy - zz;
        result.m_12 = T(2) * (xy - wz);
        result.m_13 = T(2) * (xz + wy);

        result.m_21 = T(2) * (xy + wz);
        result.m_22 = ww - xx + yy - zz;
        result.m_23 = T(2) * (yz - wx);

        result.m_31 = T(2) * (xz - wy);
        result.m_32 = T(2) * (yz + wx);
        result.m_33 = ww - xx - yy + zz;

        return result;

    }

};

//////////////////////////////////////////////////////

using b3Quaterniond = b3Quaternion<double>;
using b3Quaternionf = b3Quaternion<float>;
using b3Quaternionr = b3Quaternion<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Quaternion<T> operator*(const b3Quaternion<T>& q1, const b3Quaternion<T>& q2) {

    b3Vector3<T> v1(q1.m_x, q1.m_y, q1.m_z);
    b3Vector3<T> v2(q2.m_x, q2.m_y, q2.m_z);

    T w = q1.m_w * q2.m_w - v1.dot(v2);

    b3Vector3<T> v = q1.m_w * v2 + q2.m_w * v1 + v1.cross(v2);

    return b3Quaternion<T>(w, v);
}


template <typename T>
inline b3Quaternion<T> operator+(const b3Quaternion<T>& q1, const b3Quaternion<T>& q2) {

    return b3Quaternion<T>(q1.m_w + q2.m_w,
                           q1.m_x + q2.m_x,
                           q1.m_y + q2.m_y,
                           q1.m_z + q2.m_z);
}


template <typename T>
inline b3Quaternion<T> operator-(const b3Quaternion<T>& q1, const b3Quaternion<T>& q2) {

    return b3Quaternion<T>(q1.m_w - q2.m_w,
                           q1.m_x - q2.m_x,
                           q1.m_y - q2.m_y,
                           q1.m_z - q2.m_z);
}


template <typename T>
inline b3Quaternion<T> operator*(const b3Quaternion<T>& q, const T& s) {

    return b3Quaternion<T>(q.m_w * s,
                           q.m_x * s,
                           q.m_y * s,
                           q.m_z * s);
}


template <typename T>
inline b3Quaternion<T> operator*(const T& s, const b3Quaternion<T>& q) {

    return q * s;
}


template <typename T>
inline b3Quaternion<T> b3_aa_to_quaternion(const b3Vector3<T>& v) {

    T angle = v.length();

    if (angle == T(0)) {
        return b3Quaternion<T>();
    }

    T w = b3_cos(angle * T(0.5));
    b3Vector3<T> v1 = b3_sin(angle * T(0.5)) / angle * v;

    return b3Quaternion<T>(w, v1);
}


#endif //BOX3D_B3_QUATERNION_HPP
