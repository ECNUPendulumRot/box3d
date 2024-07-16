// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_QUAT_HPP
#define BOX3D_B3_QUAT_HPP


#include "b3_vec3.hpp"
#include "b3_mat33.hpp"

/**
 * @brief The b3Quat class represents a quaternion, a mathematical concept used to handle
 * rotations in 3D space.
 */
template <typename T>
struct b3Quat {

    union {
        /**
         * @brief An array representing the components of the quaternion.
         */
        T m_ts[4];

        /**
         * @brief Components of the quaternion
         */
        struct {
            T m_w, m_x, m_y, m_z;
        };

    };

    /**
     * @brief the constructor of b3Quat
     */
    b3Quat() {
        m_w = T(1);
        m_x = m_y = m_z = T(0);
    }
    /**
     * @brief the constructor of b3Quat
     * @param v A vector representing the Euler angles.
     */
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

    /**
     * @brief Copy constructor that initializes the quaternion from another quaternion.
     * @param other The quaternion to copy.
     */
    b3Quat(const b3Quat& other) {
        m_w = other.m_w;
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }

    /**
     * @brief Initializes the quaternion using a scalar part and a vector part.
     * @param w The scalar part of the quaternion.
     * @param v The vector part of the quaternion.
     */
    b3Quat(const T& w, const b3Vec3<T>& v) {
        m_w = w;
        m_x = v.x;
        m_y = v.y;
        m_z = v.z;
    }

    /**
     * @brief the constructor of b3Quat
     * @param w The scalar part.
     * @param x The vector parts.
     * @param y The vector parts.
     * @param z The vector parts.
     */
    b3Quat(T w, T x, T y, T z) {
        m_w = w;
        m_x = x;
        m_y = y;
        m_z = z;
    }

    /**
     * @brief Sets the components of the quaternion.
     * @param w The scalar part.
     * @param x The vector parts.
     * @param y The vector parts.
     * @param z The vector parts.
     */
    inline void set(T w, T x, T y, T z) {
        m_w = w;
        m_x = x;
        m_y = y;
        m_z = z;
    }

    /**
     * @brief Normalizes the quaternion to unit length.
     */
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

    /**
     * @brief Converts the quaternion to a 3x3 rotation matrix.
     * @return  The corresponding 3x3 rotation matrix.
     */
    inline b3Mat33<T> rotation_matrix() const {
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

        b3Mat33<T> result;

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

    /**
     * @brief Subtracts another quaternion from this quaternion.
     * @param q The quaternion to subtract.
     * @return The result of the subtraction.
     */
    inline b3Quat& operator-= (const b3Quat& q) {
        m_w -= q.m_w;
        m_x -= q.m_x;
        m_y -= q.m_y;
        m_z -= q.m_z;
        return *this;
    }

    /**
     * @brief Adds another quaternion to this quaternion.
     * @param q The quaternion to add.
     * @return The result of the addition.
     */
    inline b3Quat& operator+= (const b3Quat& q) {
        m_w += q.m_w;
        m_x += q.m_x;
        m_y += q.m_y;
        m_z += q.m_z;
        return *this;
    }
};

//////////////////////////////////////////////////////

using b3Quatd = b3Quat<double>; ///< This alias defines b3Quatd as a quaternion where the components (m_w, m_x, m_y, m_z) are of type double.
using b3Quatf = b3Quat<float>; ///< This alias defines b3Quatf as a quaternion where the components (m_w, m_x, m_y, m_z) are of type float.
using b3Quatr = b3Quat<real>; ///< This alias defines b3Quatr as a quaternion where the components (m_w, m_x, m_y, m_z) are of type real.

//////////////////////////////////////////////////////

/**
 * @brief To multiply two quaternions q1 and q2
 * @param q1 The first quaternion.
 * @param q2  The second quaternion.
 * @return A new quaternion resulting from the multiplication of q1 and q2.
 */
template <typename T>
inline b3Quat<T> operator*(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    b3Vec3<T> v1(q1.m_x, q1.m_y, q1.m_z);
    b3Vec3<T> v2(q2.m_x, q2.m_y, q2.m_z);

    T w = q1.m_w * q2.m_w - v1.dot(v2);

    b3Vec3<T> v = q1.m_w * v2 + q2.m_w * v1 + v1.cross(v2);

    return b3Quat<T>(w, v);
}

/**
 * @brief To add two quaternions component-wise, resulting in a new quaternion.
 * @param q1 The first quaternion.
 * @param q2 The second quaternion.
 * @return A new quaternion resulting from the component-wise addition of q1 and q2.
 */
template <typename T>
inline b3Quat<T> operator+(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    return b3Quat<T>(q1.m_w + q2.m_w,
                           q1.m_x + q2.m_x,
                           q1.m_y + q2.m_y,
                           q1.m_z + q2.m_z);
}

/**
 * @brief To subtract the components of q2 from q1, resulting in a new quaternion.
 * @param q1 The first quaternion.
 * @param q2 The second quaternion.
 * @return A new quaternion resulting from the component-wise subtraction of q2 from q1.
 */
template <typename T>
inline b3Quat<T> operator-(const b3Quat<T>& q1, const b3Quat<T>& q2) {

    return b3Quat<T>(q1.m_w - q2.m_w,
                           q1.m_x - q2.m_x,
                           q1.m_y - q2.m_y,
                           q1.m_z - q2.m_z);
}

/**
 * @brief To scale a quaternion by a scalar s, resulting in a new quaternion.
 * @param q The quaternion to be scaled.
 * @param s The scalar value.
 * @return A new quaternion with each component of q multiplied by s.
 */
template <typename T>
inline b3Quat<T> operator*(const b3Quat<T>& q, const T& s) {

    return b3Quat<T>(q.m_w * s,
                           q.m_x * s,
                           q.m_y * s,
                           q.m_z * s);
}

/**
 * @brief To scale a quaternion by a scalar s
 * @param s The scalar value.
 * @param q The quaternion to be scaled.
 * @return A new quaternion with each component of q multiplied by s.
 */
template <typename T>
inline b3Quat<T> operator*(const T& s, const b3Quat<T>& q) {

    return q * s;
}

/**
 * @brief To convert an axis-angle representation to a quaternion.
 * @param v The axis-angle vector.
 * @return A quaternion representing the same rotation as the axis-angle vector v.
 */
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
