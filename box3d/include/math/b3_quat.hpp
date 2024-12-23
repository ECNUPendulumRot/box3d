
#ifndef BOX3D_B3_QUAT_HPP
#define BOX3D_B3_QUAT_HPP


#include "b3_vec3.hpp"

template <typename T>
struct b3Quat {

    union {
        T m_ts[4];
        struct {
            T m_w, m_x, m_y, m_z;
        };

    };

    b3Quat() {
        m_w = T(1);
        m_x = m_y = m_z = T(0);
    }

    explicit b3Quat(const b3Vec3<T>& v) {
        // v = [x, y, z]
        // roll, pitch, yaw
        T sin_roll_half = sin(v.x * 0.5);
        T cos_roll_half = cos(v.x * 0.5);
        T sin_pitch_half = sin(v.y * 0.5);
        T cos_pitch_half = cos(v.y * 0.5);
        T sin_yaw_half = sin(v.z * 0.5);
        T cos_yaw_half = cos(v.z * 0.5);
        m_w = T(cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half);
        m_x = T(sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half);
        m_y = T(cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half);
        m_z = T(cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half);
    }

    b3Quat(const b3Quat& other) {
        m_w = other.m_w;
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }

    b3Quat(const T& w, const b3Vec3<T>& v) {
        m_w = w;
        m_x = v.x;
        m_y = v.y;
        m_z = v.z;
    }

    b3Quat(T w, T x, T y, T z) {
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

    b3Quat<T> inverse() const {
        return b3Quat<T>(m_w, -m_x, -m_y, -m_z);
    }

    b3Quat<T>& operator/=(const T& s) {
        b3_assert(s != real(0.0));
        return *this *= real(1.0) / s;
    }

    b3Quat<T>& operator*=(const T& s) {
        m_x *= s;
        m_y *= s;
        m_z *= s;
        m_w *= s;
        return *this;
    }

    b3Quat<T>& safe_normalize() {
        real l2 = length2();
        if (l2 > b3_real_epsilon) {
            normalize();
        }
        return *this;
    }

    inline b3Quat<T> normalize() {
        return *this /= length();
    }

    T length() const {
        return b3_sqrt(length2());
    }

    T length2() const {
        return dot(*this);
    }

    T dot(const b3Quat<T>& q) const {
        return m_x * q.m_x + m_y * q.m_y + m_z * q.m_z + m_w * q.m_w;
    }

//    b3M rotation_matrix() const {
//        T ww = m_w * m_w;
//        T xx = m_x * m_x;
//        T yy = m_y * m_y;
//        T zz = m_z * m_z;
//        T xy = m_x * m_y;
//        T xz = m_x * m_z;
//        T yz = m_y * m_z;
//        T wx = m_w * m_x;
//        T wy = m_w * m_y;
//        T wz = m_w * m_z;
//
//        b3Mat33<T> result;
//
//        result.m_11 = ww + xx - yy - zz;
//        result.m_12 = T(2) * (xy - wz);
//        result.m_13 = T(2) * (xz + wy);
//
//        result.m_21 = T(2) * (xy + wz);
//        result.m_22 = ww - xx + yy - zz;
//        result.m_23 = T(2) * (yz - wx);
//
//        result.m_31 = T(2) * (xz - wy);
//        result.m_32 = T(2) * (yz + wx);
//        result.m_33 = ww - xx - yy + zz;
//
//        return result;
//    }
};

//////////////////////////////////////////////////////

using b3Quaterniond = b3Quat<double>;
using b3Quaternionf = b3Quat<float>;
using b3Quaternionr = b3Quat<real>;

//////////////////////////////////////////////////////

template <typename T>
b3Quat<T> operator*(const b3Quat<T>& q, const b3Vec3<T>& w) {
    return b3Quat<T>(-q.m_x * w.x - q.m_y * w.y - q.m_z * w.z,
                     q.m_w * w.x + q.m_y * w.z - q.m_z * w.y,
                     q.m_w * w.y + q.m_z * w.x - q.m_x * w.z,
                     q.m_w * w.z + q.m_x * w.y - q.m_y * w.x);
}

template <typename T>
b3Vec3<T> quat_rotate(const b3Quat<T>& rotation, const b3Vec3<T>& v) {
    b3Quat<T> q = rotation * v;
    q = q * rotation.inverse();
    return b3Vec3<T>(q.m_x, q.m_y, q.m_z);
}

template <typename T>
inline b3Quat<T> operator*(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    b3Vec3<T> v1(q1.m_x, q1.m_y, q1.m_z);
    b3Vec3<T> v2(q2.m_x, q2.m_y, q2.m_z);

    T w = q1.m_w * q2.m_w - v1.dot(v2);

    b3Vec3<T> v = q1.m_w * v2 + q2.m_w * v1 + v1.cross(v2);

    return b3Quat<T>(w, v);
}


template <typename T>
inline b3Quat<T> operator+(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    return b3Quat<T>(q1.m_w + q2.m_w,
                           q1.m_x + q2.m_x,
                           q1.m_y + q2.m_y,
                           q1.m_z + q2.m_z);
}


template <typename T>
inline b3Quat<T> operator-(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    return b3Quat<T>(q1.m_w - q2.m_w,
                           q1.m_x - q2.m_x,
                           q1.m_y - q2.m_y,
                           q1.m_z - q2.m_z);
}


template <typename T>
inline b3Quat<T> operator*(const b3Quat<T>& q, const T& s) {

    return b3Quat<T>(q.m_w * s,
                           q.m_x * s,
                           q.m_y * s,
                           q.m_z * s);
}


template <typename T>
inline b3Quat<T> operator*(const T& s, const b3Quat<T>& q) {

    return q * s;
}


template <typename T>
inline b3Quat<T> b3_aa_to_quaternion(const b3Vec3<T>& v) {

    T angle = v.length();

    if (angle == T(0)) {
        return b3Quat<T>();
    }

    T w = b3_cos(angle * T(0.5));
    b3Vec3<T> v1 = b3_sin(angle * T(0.5)) / angle * v;

    return b3Quat<T>(w, v1);
}


#endif //BOX3D_B3_QUAT_HPP
