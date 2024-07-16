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


#ifndef BOX3D_B3_VEC12_HPP
#define BOX3D_B3_VEC12_HPP


#include <initializer_list>

#include "math/b3_vec3.hpp"
#include "math/b3_math_op.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"

#include <memory>

/**
 * @brief represent a 12-dimensional vector, providing various mathematical operations
 * and utility functions to manipulate these vectors.
 */
template <typename T>
struct b3Vec12 {

    union {
        T m_ts[12]; ///< An array of 12 elements of type T representing the components of the vector.
    };

    /**
     * @brief Constructor of b3Vec12
     */
    b3Vec12() {
        memset(m_ts, 0, sizeof(m_ts));
    }

    /**
     * @brief Constructor of b3Vec12
     * @param other A reference to another b3Vec12 vector to copy from.
     */
    b3Vec12(const b3Vec12& other) {
        memcpy(m_ts, other.m_ts, sizeof(m_ts));
    }

    /**
     * @brief Assigns values from an initializer list to the vector components.
     * @param list An initializer list of double values.
     * @return A reference to the updated b3Vec12 vector.
     */
    inline b3Vec12& operator=(std::initializer_list<double> list) {
        b3_assert(list.size() == 3);
        auto it = list.begin();
        for (int i = 0; i < 12; ++i) {
            m_ts[i] = *(it++);
        }

        return *this;
    }

    /**
     * @brief Adds another b3Vec12 vector to the current vector component-wise.
     * @param v A reference to another b3Vec12 vector.
     * @return A reference to the updated b3Vec12 vector.
     */
    inline b3Vec12& operator+=(const b3Vec12& v) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] += v.m_ts[i];
        }
        return *this;
    }

    /**
     * @brief Subtracts another b3Vec12 vector from the current vector component-wise.
     * @param v A reference to another b3Vec12 vector.
     * @return A reference to the updated b3Vec12 vector.
     */
    inline b3Vec12& operator-=(const b3Vec12& v) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] -= v.m_ts[i];
        }
        return *this;
    }

    /**
     * @brief Multiplies each component of the vector by a scalar.
     * @param s A scalar of type U.
     * @return A reference to the updated b3Vec12 vector.
     */
    template <typename U>
    inline b3Vec12& operator*=(U s) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] *= s;
        }
        return *this;
    }

    /**
     * @brief Divides each component of the vector by a scalar.
     * @param s A scalar of type U.
     * @return A reference to the updated b3Vec12 vector.
     */
    template <typename U>
    inline b3Vec12& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    /**
     * @brief Computes the dot product with another b3Vec12 vector.
     * @param v A reference to another b3Vec12 vector.
     * @return The dot product of the two vectors.
     */
    inline T dot(const b3Vec12<T>& v) const {
        T sum = T(0);
        for (int i = 0; i < 12; ++i) {
            sum += m_ts[i] * v.m_ts[i];
        }
        return sum;
    }

    /**
     * @brief Computes the component-wise product with another b3Vec12 vector.
     * @param v A reference to another b3Vec12 vector.
     * @return A new b3Vec12 vector with each component being the product of the
     * corresponding components of the two vectors.
     */
    inline b3Vec12 cwise_product(const b3Vec12<T>& v) const {
        b3Vec12<T> result;
        for (int i = 0; i < 12; ++i) {
            result.m_ts[i] = m_ts[i] * v.m_ts[i];
        }
        return result;
    }

    /**
     * @brief Checks if all components of the vector are zero.
     * @return A boolean value indicating whether all components are zero.
     */
    inline bool is_zero() const{
        for (int i = 0; i < 12; ++i) {
            if (m_ts[i] != T(0)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Returns a normalized version of the current vector.
     * @return A new b3Vec12 vector that is the normalized version of the current vector.
     */
    inline b3Vec12 normalized() const {
        return *this / length();
    }

    /**
     * @brief Computes the absolute value of each component of the vector.
     * @return A new b3Vec12 vector with each component being the absolute value of the
     * corresponding component of the current vector.
     */
    inline b3Vec12 abs() const {
        b3Vec12<T> result;
        for (int i = 0; i < 12; ++i) {
            result.m_ts[i] = b3_abs(m_ts[i]);
        }
        return result;
    }

    /**
     * @brief Computes the Euclidean length (magnitude) of the vector.
     * @return The length of the vector, of type T.
     */
    inline T length() const {
        return b3_sqrt<T>(length2());
    }

    /**
     * @brief Computes the squared length of the vector.
     * @return The squared length of the vector, of type T.
     */
    inline T length2() const {
        return dot(*this);
    }

    /**
     * @brief Provides access to the vector components by index.
     * @param i The index of the component to access.
     * @return The value of the component at index i (const version) or a reference to
     * the component at index i (non-const version).
     */
    inline T operator[](int i) const {
        return m_ts[i];
    }

    /**
     * @brief Provides access to the vector components by index.
     * @param i The index of the component to access.
     * @return The value of the component at index i (const version) or a reference to
     * the component at index i (non-const version).
     */
    inline T& operator[](int i) {
        return m_ts[i];
    }

    /**
     * @brief Sets all components of the vector to zero
     */
    inline void set_zero() {
        memset(m_ts, 0, sizeof(m_ts));
    }

    /**
     * @brief Returns a b3Vec12 vector with all components set to zero.
     * @return A b3Vec12 vector with all components set to zero.
     */
    static b3Vec12 zero() {
        return b3Vec12();
    }

    /**
     * @brief Provides access to the underlying array of component
     * @return A pointer to the first element of the array m_ts.
     */
    inline T* data() {
        return m_ts;
    }

    /**
     * @brief Sets a segment of the vector using a b3Vec3 vector starting from a specified index.
     * @param b A reference to a b3Vec3 vector.
     * @param I The starting index in the b3Vec12 vector to set the segment.
     */
    void set_segment(const b3Vec3<T>& b, const int32 I) {
        memcpy(m_ts + I, b.m_ts, sizeof(T) * 3);
    }
};


//////////////////////////////////////////////////////

using b3Vec12d = b3Vec12<double>; ///< Creates an alias b3Vec12d for the b3Vec12 class template instantiated with double as the template parameter.
using b3Vec12f = b3Vec12<float>; ///< Creates an alias b3Vec12f for the b3Vec12 class template instantiated with float as the template parameter.
using b3Vec12r = b3Vec12<real>; ///< Creates an alias b3Vec12r for the b3Vec12 class template instantiated with real as the template parameter.

//////////////////////////////////////////////////////

/**
 * @brief Adds two b3Vec12 vectors element-wise.
 * @param v1 A constant reference to the first b3Vec12 vector.
 * @param v2 A constant reference to the second b3Vec12 vector.
 * @return A new b3Vec12 vector that is the element-wise sum of v1 and v2.
 */
template <typename T>
inline b3Vec12<T> operator+(const b3Vec12<T>& v1, const b3Vec12<T>& v2) {
    return b3Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

/**
 * @brief Subtracts one b3Vec12 vector from another element-wise.
 * @param v1 A constant reference to the first b3Vec12 vector.
 * @param v2 A constant reference to the second b3Vec12 vector.
 * @return A new b3Vec12 vector that is the element-wise difference of v1 and v2.
 */
template <typename T>
inline b3Vec12<T> operator-(const b3Vec12<T>& v1, const b3Vec12<T>& v2) {
    return b3Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

/**
 * @brief Negates a b3Vec12 vector element-wise.
 * @param v A constant reference to the b3Vec12 vector to be negated.
 * @return A new b3Vec12 vector that is the element-wise negation of v.
 */
template <typename T>
inline b3Vec12<T> operator-(const b3Vec12<T>& v) {
    return b3Vec3(-v.x, -v.y, -v.z);
}

/**
 * @brief Multiplies a b3Vec12 vector by a scalar.
 * @param s The scalar value to multiply by.
 * @param v A constant reference to the b3Vec12 vector.
 * @return A new b3Vec12 vector that is the result of element-wise multiplication of v by s.
 */
template <typename T, typename U>
inline b3Vec12<T> operator*(U s, const b3Vec12<T>& v) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result.m_ts[i] = s * v.m_ts[i];
    }
    return result;
}

/**
 * @brief Multiplies a b3Vec12 vector by a scalar
 * @param v A constant reference to the b3Vec12 vector.
 * @param s The scalar value to multiply by.
 * @return A new b3Vec12 vector that is the result of element-wise multiplication of v by s.
 */
template <typename T, typename U>
inline b3Vec12<T> operator*(const b3Vec12<T>& v, U s) {
    return s * v;
}

/**
 * @brief Divides a b3Vec12 vector by a scalar.
 * @param v A constant reference to the b3Vec12 vector.
 * @param s The scalar value to divide by.
 * @return A new b3Vec12 vector that is the result of element-wise division of v by s.
 */
template <typename T>
inline b3Vec12<T> operator/(const b3Vec12<T>& v, T s) {
    return v * (T(1.0) / s) ;
}

/**
 * @brief Computes the minimum coefficient-wise values between two b3Vec12 vectors.
 * @param a A constant reference to the first b3Vec12 vector.
 * @param b A constant reference to the second b3Vec12 vector.
 * @return A new b3Vec12 vector that contains the minimum values of each corresponding component from a and b.
 */
template <typename T>
inline b3Vec12<T> b3_min_coeff(const b3Vec12<T>& a, const b3Vec12<T>& b){
    return b3Vec3(b3_min(a.x, b.x), b3_min(a.y, b.y), b3_min(a.z, b.z));
}

/**
 * @brief Computes the maximum coefficient-wise values between two b3Vec12 vectors.
 * @param a A constant reference to the first b3Vec12 vector.
 * @param b A constant reference to the second b3Vec12 vector.
 * @return A new b3Vec12 vector that contains the maximum values of each corresponding component from a and b.
 */
template <typename T>
inline b3Vec12<T> b3_max_coeff(const b3Vec12<T>& a, const b3Vec12<T>& b){
    return b3Vec3(b3_max(a.x, b.x), b3_max(a.y, b.y), b3_max(a.z, b.z));
}


#endif //BOX3D_B3_VEC12_HPP
