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

#ifndef BOX3D_B3_VEC3_HPP
#define BOX3D_B3_VEC3_HPP


#include <initializer_list>

#include "math/b3_math_op.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"

/**
 * @brief The b3Vec3 class template is designed to represent a 3-dimensional vector and
 * provide various utility functions for vector operations
 */
template <typename T>
struct b3Vec3 {

    union {
        T m_ts[3]; ///<  An array holding the three components of the vector.
        struct {
            T x, y, z; ///< The three components of the vector.
        };
    };

    /**
     * @brief Constructor of b3Vec3
     */
    b3Vec3() {
        x = y = z = T(0);
    }

    /**
     * @brief Copy Constructor
     * @param other  another vector.
     */
    b3Vec3(const b3Vec3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    /**
     * @brief Assigns values from an initializer list to the vector.
     * @param list an initializer list with 3 elements
     * @return A reference to the current object
     */
    inline b3Vec3& operator=(std::initializer_list<T> list) {
        b3_assert(list.size() == 3);
        auto it = list.begin();
        x = *it++;
        y = *it++;
        z = *it;

        return *this;
    }

    /**
     * @brief Initializes the vector with given x, y, and z values.
     * @param x the x-component.
     * @param y the y-component.
     * @param z the z-component.
     */
    inline b3Vec3(T x, T y, T z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    /**
     * @brief Sets the vector's components to the provided x, y, and z values.
     * @param x the x-component.
     * @param y the y-component.
     * @param z the z-component.
     */
    inline void set(T x, T y, T z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    /**
     * @brief Adds another vector to the current vector.
     * @param v the vector to add.
     * @return A reference to the current object.
     */
    inline b3Vec3& operator+=(const b3Vec3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    /**
     * @brief Subtracts another vector from the current vector.
     * @param v the vector to subtract
     * @return A reference to the current object.
     */
    inline b3Vec3& operator-=(const b3Vec3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    /**
     * @brief Multiplies the current vector by a scalar.
     * @param s the scalar value
     * @return A reference to the current object
     */
    template <typename U>
    inline b3Vec3& operator*=(U s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /**
     * @brief Divides the current vector by a scalar.
     * @param s the scalar value.
     * @return A reference to the current object.
     */
    template <typename U>
    inline b3Vec3& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    /**
     * @brief Computes the dot product of the vector with another vector.
     * @param v  the other vector.
     * @return The dot product of the two vectors
     */
    inline T dot(const b3Vec3<T>& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    /**
     * @brief Computes the cross product of the vector with another vector
     * @param v the other vector.
     * @return A new vector that is the cross product of the two vectors.
     */
    inline b3Vec3 cross(const b3Vec3<T>& v) const {
        return b3Vec3(y * v.z - z * v.y,
                      z * v.x - x * v.z,
                      x * v.y - y * v.x);
    }

    /**
     * @brief Multiplies the vector's components with another vector's components
     * @param v  the other vector.
     * @return A new vector with component-wise products.
     */
    inline b3Vec3 cwise_product(const b3Vec3<T>& v) const {
        return b3Vec3(x * v.x, y * v.y, z * v.z);
    }

    /**
     * @brief Checks if all components are approximately zero
     * @return true if all components are approximately zero, otherwise false.
     */
    inline bool is_zero() const{
        return b3_abs(x) < b3_real_epsilon && b3_abs(y) < b3_real_epsilon && b3_abs(z) < b3_real_epsilon;
    }

    /**
     * @brief Returns a normalized (unit length) vector.
     * @return A new vector that is the normalized version of the current vector.
     */
    inline b3Vec3 normalized() const {
        return *this / length();
    }

    /**
     * @brief Returns a vector with each component's absolute value
     * @return A new vector with the absolute values of the components.
     */
    inline b3Vec3 abs() const {
        return b3Vec3(b3_abs(x), b3_abs(y), b3_abs(z));
    }

    /**
     * @brief Computes the dot product in an element-wise manner
     * @param v  the other vector.
     * @return A new vector with the element-wise dot products.
     */
    inline b3Vec3 array_dot(const b3Vec3<T>& v) const {
        return b3Vec3(x * v.x, y * v.y, z * v.z);
    }

    /**
     * @brief Returns the length (magnitude) of the vector.
     * @return The length of the vector.
     */
    inline T length() const {
        return b3_sqrt<T>(length2());
    }

    /**
     * @brief Returns the squared length of the vector
     * @return The squared length of the vector.
     */
    inline T length2() const {
        return dot(*this);
    }

    /**
     * @brief Provides access to the vector's components by index
     * @param i  the index
     * @return  The value at the specified index.
     */
    inline T operator[](int i) const {
        return m_ts[i];
    }

    /**
     * @brief Provides access to the vector's components by index
     * @param i the index
     * @return The value at the specified index.
     */
    inline T& operator[](int i) {
        return m_ts[i];
    }

    /**
     * @brief Sets all components to zero.
     */
    inline void set_zero() {
        x = y = z = T(0);
    }

    /**
     * @brief Static Function to Get a Zero Vector
     * @return A new vector with all components set to zero.
     */
    static b3Vec3 zero() {
        return b3Vec3(T(0), T(0), T(0));
    }

    /**
     * @brief Returns a pointer to the underlying array of components
     * @return A pointer to the underlying array of components.
     */
    inline T* data() {
        return m_ts;
    }

    /**
     * @brief Rounds each component to zero if it is close to zero.
     */
    inline void round_to_zero() {
        x = b3_round_to_zero(x);
        y = b3_round_to_zero(y);
        z = b3_round_to_zero(z);
    }
};


//////////////////////////////////////////////////////

using b3Vec3d = b3Vec3<double>; ///< This alias defines b3Vec3d as a b3Vec3 with the template parameter T set to double.
using b3Vec3f = b3Vec3<float>; ///< This alias defines b3Vec3f as a b3Vec3 with the template parameter T set to float.
using b3Vec3r = b3Vec3<real>; ///< This alias defines b3Vec3r as a b3Vec3 with the template parameter T set to real.
using b3Vec3i = b3Vec3<int32>; ///< This alias defines b3Vec3i as a b3Vec3 with the template parameter T set to int32

//////////////////////////////////////////////////////

/**
 * @brief To add two b3Vec3 vectors component-wise.
 * @param v1 The first vector of type b3Vec3<T>.
 * @param v2 The second vector of type b3Vec3<T>.
 * @return
 */
template <typename T>
inline b3Vec3<T> operator+(const b3Vec3<T>& v1, const b3Vec3<T>& v2) {
    return b3Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

/**
 * @brief To subtract one b3Vec3 vector from another component-wise.
 * @param v1 The first vector of type b3Vec3<T>.
 * @param v2 The second vector of type b3Vec3<T>.
 * @return  A new b3Vec3 vector where each component is the difference between
 * the corresponding components of v1 and v2.
 */
template <typename T>
inline b3Vec3<T> operator-(const b3Vec3<T>& v1, const b3Vec3<T>& v2) {
    return b3Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

/**
 * @brief  To negate all components of a b3Vec3 vector.
 * @param v The vector of type b3Vec3<T> to be negated.
 * @return A new b3Vec3 vector where each component is the negation of the
 * corresponding component of v.
 */
template <typename T>
inline b3Vec3<T> operator-(const b3Vec3<T>& v) {
    return b3Vec3(-v.x, -v.y, -v.z);
}

/**
 * @brief To multiply a b3Vec3 vector by a scalar from the left.
 * @param s The scalar of type U.
 * @param v The vector of type b3Vec3<T>.
 * @return A new b3Vec3 vector where each component is the product of the scalar
 * s and the corresponding component of v.
 */
template <typename T, typename U>
inline b3Vec3<T> operator*(U s, const b3Vec3<T>& v) {
    return b3Vec3<T>(s * v.x, s * v.y, s * v.z);
}

/**
 * @brief To multiply a b3Vec3 vector by a scalar from the right
 * @param v The vector of type b3Vec3<T>.
 * @param s The scalar of type U.
 * @return A new b3Vec3 vector where each component is the product of the scalar
 * s and the corresponding component of v.
 */
template <typename T, typename U>
inline b3Vec3<T> operator*(const b3Vec3<T>& v, U s) {
    return s * v;
}

/**
 * @brief  To divide each component of a b3Vec3 vector by a scalar.
 * @param v The vector of type b3Vec3<T>.
 * @param s The scalar of type T.
 * @return A new b3Vec3 vector where each component is the quotient of
 * the corresponding component of v divided by the scalar s
 */
template <typename T>
inline b3Vec3<T> operator/(const b3Vec3<T>& v, T s) {
    return v * (T(1.0) / s) ;
}

/**
 * @brief To find the component-wise minimum of two b3Vec3 vectors.
 * @param a The first vector of type b3Vec3<T>.
 * @param b The second vector of type b3Vec3<T>
 * @return A new b3Vec3 vector where each component is the minimum of the corresponding
 * components of a and b.
 */
template <typename T>
inline b3Vec3<T> b3_min_coeff(const b3Vec3<T>& a, const b3Vec3<T>& b){
    return b3Vec3(b3_min(a.x, b.x), b3_min(a.y, b.y), b3_min(a.z, b.z));
}

/**
 * @brief To find the component-wise maximum of two b3Vec3 vectors.
 * @param a The first vector of type b3Vec3<T>.
 * @param b The second vector of type b3Vec3<T>.
 * @return A new b3Vec3 vector where each component is the maximum of the corresponding
 * components of a and b.
 */
template <typename T>
inline b3Vec3<T> b3_max_coeff(const b3Vec3<T>& a, const b3Vec3<T>& b){
    return b3Vec3(b3_max(a.x, b.x), b3_max(a.y, b.y), b3_max(a.z, b.z));
}

#define SQRT12 real(0.7071067811865475244008443621048490)

template <typename T>
inline void b3_plane_space(const T& n, T& p, T& q) {
    if (b3_abs(n[2]) > SQRT12) {
        // choose p in y-z plane
        real a = n[1] * n[1] + n[2] * n[2];
        real k = 1.0 / b3_sqrt(a);
        p[0] = 0;
        p[1] = -n[2] * k;
        p[2] = n[1] * k;
        // set q = n x p
        q[0] = a * k;
        q[1] = -n[0] * p[2];
        q[2] = n[0] * p[1];
    } else {
        // choose p in x-y plane
        real a = n[0] * n[0] + n[1] * n[1];
        real k = 1.0 / b3_sqrt(a);
        p[0] = -n[1] * k;
        p[1] = n[0] * k;
        p[2] = 0;
        // set q = n x p
        q[0] = -n[2] * p[1];
        q[1] = n[2] * p[0];
        q[2] = a * k;
    }
}

#endif //BOX3D_B3_VEC3_HPP
