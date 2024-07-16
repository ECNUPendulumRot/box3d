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


#ifndef BOX3D_B3_VEC2_HPP
#define BOX3D_B3_VEC2_HPP


#include <initializer_list>

#include "math/b3_math_op.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"

/**
 * @brief The b3Vec2 class is designed to represent a 2-dimensional vector and provides
 * various operations that can be performed on 2D vectors.
 */
template <typename T>
struct b3Vec2 {

    /**
     * @brief The union allows access to the vector's components either through an array
     * m_ts or through named members x and y.
     */
    union {
        T m_ts[2];
        struct {
            T x, y;
        };
    };

    /**
     * @brief constructor of b3Vec2
     */
    b3Vec2() {
        x = y = T(0);
    }

    /**
     * @brief Copy constructor that initializes the vector with the values of another vector.
     * @param other The vector to copy from.
     */
    b3Vec2(const b3Vec2& other) {
        x = other.x;
        y = other.y;
    }

    /**
     * @brief Assigns values from an initializer list to the vector components.
     * @param list An initializer list with two elements.
     * @return A reference to the modified vector.
     */
    inline b3Vec2& operator=(std::initializer_list<T> list) {
        b3_assert(list.size() == 2);
        auto it = list.begin();
        x = *it++;
        y = *it;
        return *this;
    }

    /**
     * @brief Constructor of b3Vec2
     * @param x The x-component.
     * @param y The y-component.
     */
    inline b3Vec2(T x, T y) {
        this->x = x;
        this->y = y;
    }

    /**
     * @brief Sets the vector components to the given values.
     * @param x The x-component.
     * @param y The y-component.
     */
    inline void set(T x, T y) {
        this->x = x;
        this->y = y;
    }

    /**
     * @brief Adds another vector to this vector component-wise.
     * @param v The vector to add.
     * @return A reference to the modified vector.
     */
    inline b3Vec2& operator+=(const b3Vec2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    /**
     * @brief Subtracts another vector from this vector component-wise.
     * @param v The vector to subtract.
     * @return A reference to the modified vector
     */
    inline b3Vec2& operator-=(const b3Vec2& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    /**
     * @brief Multiplies the vector components by a scalar.
     * @param s The scalar to multiply by.
     * @return A reference to the modified vector.
     */
    template <typename U>
    inline b3Vec2& operator*=(U s) {
        x *= s;
        y *= s;
        return *this;
    }

    /**
     * @brief Divides the vector components by a scalar.
     * @param s The scalar to divide by
     * @return A reference to the modified vector.
     */
    template <typename U>
    inline b3Vec2& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    /**
     * @brief Computes the dot product with another vector.
     * @param v The vector to dot with.
     * @return The dot product.
     */
    inline T dot(const b3Vec2<T>& v) const {
        return x * v.x + y * v.y;
    }

    /**
     * @brief Computes the component-wise product with another vector.
     * @param v The vector to multiply with.
     * @return The component-wise product vector.
     */
    inline b3Vec2 cwise_product(const b3Vec2<T>& v) const {
        return b3Vec2(x * v.x, y * v.y);
    }

    /**
     * @brief Checks if the vector is a zero vector (both components are close to zero).
     * @return True if the vector is zero, false otherwise.
     */
    inline bool is_zero() const{
        return b3_abs(x) < b3_real_epsilon && b3_abs(y) < b3_real_epsilon < b3_real_epsilon;
    }

    /**
     * @brief Returns a normalized (unit length) version of the vector.
     * @return The normalized vector.
     */
    inline b3Vec2 normalized() const {
        return *this / length();
    }

    /**
     * @brief Returns a vector with the absolute values of the components.
     * @return The absolute value vector.
     */
    inline b3Vec2 abs() const {
        return b3Vec2(b3_abs(x), b3_abs(y));
    }

    /**
     * @brief Computes the component-wise product
     * @param v  The vector to multiply with
     * @return  The component-wise product vector.
     */
    inline b3Vec2 array_dot(const b3Vec2<T>& v) const {
        return b3Vec2(x * v.x, y * v.y);
    }

    /**
     * @brief Computes the length (magnitude) of the vector.
     * @return The length of the vector.
     */
    inline T length() const {
        return b3_sqrt<T>(length2());
    }

    /**
     * @brief Computes the squared length of the vector.
     * @return The squared length of the vector.
     */
    inline T length2() const {
        return dot(*this);
    }

    /**
     * @brief Accesses the i-th component of the vector (read-only).
     * @param i  The index of the component
     * @return  The value of the i-th component.
     */
    inline T operator[](int i) const {
        return m_ts[i];
    }

    /**
     * @brief Accesses the i-th component of the vector
     * @param i The index of the component
     * @return A reference to the i-th component.
     */
    inline T& operator[](int i) {
        return m_ts[i];
    }

    /**
     * @brief Sets the vector components to zero.
     */
    inline void set_zero() {
        x = y = T(0);
    }

    /**
     * @brief Returns a zero vector.
     * @return A zero vector.
     */
    static b3Vec2 zero() {
        return b3Vec2(T(0), T(0), T(0));
    }

    /**
     * @brief Returns a pointer to the underlying array of components.
     * @return A pointer to the array of components.
     */
    inline T* data() {
        return m_ts;
    }
};


//////////////////////////////////////////////////////

using b3Vec2d = b3Vec2<double>; ///< Defines b3Vec2d as an alias for b3Vec2<double>.
using b3Vec2f = b3Vec2<float>; ///< Defines b3Vec2f as an alias for b3Vec2<float>.
using b3Vec2r = b3Vec2<real>; ///< Defines b3Vec2r as an alias for b3Vec2<real>.

//////////////////////////////////////////////////////

/**
 * @brief Adds two 2D vectors component-wise.
 * @param v1 First vector of type b3Vec2<T>.
 * @param v2 Second vector of type b3Vec2<T>.
 * @return Returns a new b3Vec2<T> that is the result of the component-wise addition of v1 and v2.
 */
template <typename T>
inline b3Vec2<T> operator+(const b3Vec2<T>& v1, const b3Vec2<T>& v2) {
    return b3Vec2(v1.x + v2.x, v1.y + v2.y);
}

/**
 * @brief Subtracts the second 2D vector from the first one component-wise.
 * @param v1 First vector of type b3Vec2<T>.
 * @param v2 Second vector of type b3Vec2<T>.
 * @return Returns a new b3Vec2<T> that is the result of the component-wise subtraction of v2 from v1.
 */
template <typename T>
inline b3Vec2<T> operator-(const b3Vec2<T>& v1, const b3Vec2<T>& v2) {
    return b3Vec2(v1.x - v2.x, v1.y - v2.y);
}

/**
 * @brief Negates both components of the 2D vector.
 * @param v A vector of type b3Vec2<T>.
 * @return Returns a new b3Vec2<T> with both components negated.
 */
template <typename T>
inline b3Vec2<T> operator-(const b3Vec2<T>& v) {
    return b3Vec2(-v.x, -v.y);
}

/**
 * @brief Multiplies a 2D vector by a scalar.
 * @param s Scalar value of type U.
 * @param v A vector of type b3Vec2<T>.
 * @return Returns a new b3Vec2<T> that is the result of the scalar multiplication
 */
template <typename T, typename U>
inline b3Vec2<T> operator*(U s, const b3Vec2<T>& v) {
    return b3Vec2<T>(s * v.x, s * v.y);
}

/**
 * @brief Multiplies a 2D vector by a scalar. This function is just a convenience wrapper
 * that calls the previous function.
 * @param v A vector of type b3Vec2<T>.
 * @param s Scalar value of type U.
 * @return Returns a new b3Vec2<T> that is the result of the scalar multiplication.
 */
template <typename T, typename U>
inline b3Vec2<T> operator*(const b3Vec2<T>& v, U s) {
    return s * v;
}

/**
 * @brief Divides a 2D vector by a scalar.
 * @param v A vector of type b3Vec2<T>.
 * @param s Scalar value of type T.
 * @return Returns a new b3Vec2<T> that is the result of dividing each component of v by s.
 */
template <typename T>
inline b3Vec2<T> operator/(const b3Vec2<T>& v, T s) {
    return v * (T(1.0) / s) ;
}


#endif //BOX3D_B3_VEC2_HPP
