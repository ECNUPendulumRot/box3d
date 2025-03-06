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



#ifndef BOX3D_B3_TRANSFORM_HPP
#define BOX3D_B3_TRANSFORM_HPP


#include "common/b3_types.hpp"

#include "math/b3_mat33.hpp"
#include "math/b3_quat.hpp"

/**
 * @brief The b3Trans template class represents a 3D transformation, which includes
 * a position vector and a rotation matrix
 * @tparam T It allows the creation of classes or functions that can work with any data type
 */
template <typename T>
struct b3Trans {

public:

    // The position part of the pose.
    /**
     * @brief The position part of the pose.
     */
    b3Vec3<T> m_p;

    // The rotation matrix of the pose.
    /**
     * @brief The rotation matrix of the pose.
     */
    b3Mat33<T> m_r;

    /**
     * @brief the constructor of b3Trans struct
     */
    b3Trans() = default;

    /**
      * @brief the constructor of b3Trans struct
      * @param other The b3Trans object to copy from.
      */
    b3Trans(const b3Trans& other) {
        m_p = other.m_p;
        m_r = other.m_r;
    }

    /**
     * @brief the constructor of b3Trans struct
     * @param p The position vector.
     * @param q The quaternion representing the rotation.
     */
    b3Trans(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r = q.rotation_matrix();
    }

    /**
     * @brief Sets the position and rotation of the transformation.
     * @param p The position vector.
     * @param q The quaternion representing the rotation.
     */
    void set(const b3Vec3<T>& p, const b3Quat<T>& q) {
        m_p = p;
        m_r = q.rotation_matrix();
    }

    /**
     * @brief Sets the position of the transformation.
     * @param p The position vector.
     */
    void set_position(const b3Vec3<T>& p) {
        m_p = p;
    }

    /**
     * @brief Sets the rotation using Euler angles.
     * @param euler The Euler angles.
     */
    void set_euler_angles(const b3Vec3<T>& euler) {
        b3Quat<T> q(euler);
        m_r = q.rotation_matrix();
    }

    /**
     * @brief Sets the rotation using a quaternion.
     * @param q The quaternion.
     */
    void set_quaternion(const b3Quat<T>& q) {
        m_r = q.rotation_matrix();
    }

    /**
     * @brief Gets the rotation matrix.
     * @return Returns the rotation matrix of the transformation
     */
    inline const b3Mat33<T>& rotation_matrix() const {
        return m_r;
    };

    /**
     * @brief Transforms a vector by the transformation.
     * @param v The vector to transform.
     * @return The transformed vector.
     */
    inline b3Vec3<T> transform(const b3Vec3<T>& v) const {
        return m_r * v + m_p;
    }

    /**
     * @brief Rotates a vector by the transformation.
     * @param v The vector to rotate.
     * @return  The rotated vector.
     */
    inline b3Vec3<T> rotate(const b3Vec3<T>& v) const {
        return m_r * v;
    }

    /**
     * @brief Transforms a vector in local coordinates.
     * @param v The vector to transform.
     * @return The transformed vector in local coordinates.
     */
    inline b3Vec3<T> transform_local(const b3Vec3<T>& v) const {
        return m_r.transpose() * (v - m_p);
    }

    inline b3Vec3<T> ne_transform_local(const b3Vec3<T>& v) const {
        return m_r.transpose() * (m_p - v);
    }

    /**
     * @brief Gets the position vector.
     * @return The position vector
     */
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


using b3Transf = b3Trans<float>; ///< This creates an alias b3Transf for the class b3Trans specialized with float as the template parameter T.
using b3Transd = b3Trans<double>; ///< This creates an alias b3Transd for the class b3Trans specialized with double as the template parameter T.
using b3Transr = b3Trans<real>; ///< creates an alias b3Transr for the class b3Trans specialized with real.

#endif //BOX3D_B3_TRANSFORM_HPP
