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


#ifndef BOX3D_B3_MAT33_HPP
#define BOX3D_B3_MAT33_HPP


#include <cstring>
#include "math/b3_vec3.hpp"
#include "common/b3_common.hpp"

/**
 * @brief The struct template represents a 3x3 matrix and provides various functionalities
 * for matrix operations
 */
template <typename T>
struct b3Mat33 {

    union {
        /**
         * @brief m_ts is col major;
         */
        T m_ts[3][3];

        /**
         * @brief Provides a convenient way to access individual elements of the matrix.
         */
        struct {
            T m_11, m_21, m_31;
            T m_12, m_22, m_32;
            T m_13, m_23, m_33;
        };

        /**
         * @brief Provides a way to access the columns of the matrix as vectors.
         */
        struct {
            b3Vec3<T> m_col1, m_col2, m_col3;
        };
    };

public:

    /**
     * @brief the constructor of b3Mat33
     */
    b3Mat33() {
        memset(m_ts, 0, sizeof(T) * 9);
    }

    /**
     * @brief  the constructor of b3Mat33
     * @param other  The matrix to copy from.
     */
    b3Mat33(const b3Mat33 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 9);
    }

    /**
     * @brief  the constructor of b3Mat33
     * @param col1 The first column.
     * @param col2 The second column.
     * @param col3 The third column.
     */
    b3Mat33(const b3Vec3<T>& col1, const b3Vec3<T>& col2, const b3Vec3<T>& col3) {
        m_col1 = col1;
        m_col2 = col2;
        m_col3 = col3;
    }

    /**
     * @brief gets the i-th column of the matrix as a vector.
     * @param i The index of the column.
     * @return Returns the i-th column of the matrix as a vector.
     */
    inline b3Vec3<T> col(const int i) const {
        return b3Vec3<T>(m_ts[i][0], m_ts[i][1], m_ts[i][2]);
    }

    /**
     * @brief gets the i-th row of the matrix as a vector.
     * @param i The index of the row.
     * @return Returns the i-th row of the matrix as a vector.
     */
    inline b3Vec3<T> row(const int i) const {
        return b3Vec3<T>(m_ts[0][i], m_ts[1][i], m_ts[2][i]);
    }

    /**
     * @brief gets the transpose of the matrix.
     * @return Returns the transpose of the matrix.
     */
    inline b3Mat33<T> transpose() const {
        b3Mat33<T> m;

        for (int32 i = 0; i < 3; i++) {
            for (int32 j = 0; j < 3; j++) {
                m.m_ts[i][j] = m_ts[j][i];
            }
        }
        return m;
    }

    /**
     * @brief Sets all elements of the matrix to zero.
     */
    inline void set_zero() {
        m_ts = {T(0)};
    }

    /**
     * @brief Sets the matrix to the identity matrix.
     */
    inline void set_identity() {
        memset(m_ts, 0, sizeof(T) * 9);
        m_ts[0][0] = T(1);
        m_ts[1][1] = T(1);
        m_ts[2][2] = T(1);
    }

    /**
     * @brief Computes and returns the determinant of the matrix.
     * @return  The determinant as a T.
     */
    inline T determinant() const {
        return m_11 * (m_22 * m_33 - m_32 * m_23) -
               m_21 * (m_12 * m_33 - m_32 * m_13) +
               m_31 * (m_12 * m_23 - m_22 * m_13);
    }

    /**
     * @brief Computes and returns the inverse of the matrix.
     * @return The inverse matrix.
     */
    inline b3Mat33 inverse() {
        double det = determinant();
        b3_assert(det > 0);
        double inv_det = 1.0 / det;

        b3Mat33 inv;
        inv.m_11 =  inv_det * (m_22 * m_33 - m_23 * m_32);
        inv.m_21 = -inv_det * (m_12 * m_33 - m_13 * m_32);
        inv.m_31 =  inv_det * (m_12 * m_23 - m_22 * m_13);
        inv.m_12 = -inv_det * (m_21 * m_33 - m_23 * m_31);
        inv.m_22 =  inv_det * (m_11 * m_33 - m_13 * m_31);
        inv.m_32 = -inv_det * (m_11 * m_23 - m_21 * m_13);
        inv.m_13 =  inv_det * (m_21 * m_32 - m_22 * m_31);
        inv.m_23 = -inv_det * (m_11 * m_32 - m_12 * m_31);
        inv.m_33 =  inv_det * (m_11 * m_22 - m_12 * m_21);

        return inv;
    }

    /**
     * @brief Computes and returns the trace of the matrix (sum of diagonal elements).
     * @return The trace as a real.
     */
    inline real trace() const {
        return m_11 + m_22 + m_33;
    }

    /**
     * @brief Creates a skew-symmetric matrix from a vector.
     * @param v The vector.
     * @return The skew-symmetric matrix.
     */
    static b3Mat33<T> skew_symmetric(const b3Vec3<T>& v) {
        return b3Mat33<T>(b3Vec3<T>(T(0), v.z, -v.y),
                          b3Vec3<T>(-v.z, T(0), v.x),
                          b3Vec3<T>(v.y, -v.x, T(0)));
    }

    /**
     * @brief Computes and returns the eigenvalues of the matrix.
     * @return The eigenvalues as a b3Vec3<T>.
     */
    inline b3Vec3<T> eigen_values() const {
        T e1 = trace();
        T e2 = T(0.5) * (e1 * e1 + ((*this) * (*this)).trace());
        T e3 = determinant();
        return b3Vec3<T>(e1, e2, e3);
    }

    /**
     * @brief Subtracts another matrix from this matrix.
     * @param M The matrix to subtract.
     * @return A reference to the updated matrix.
     */
    inline b3Mat33& operator-=(const b3Mat33<T>& M) {
        m_col1 -= M.m_col1;
        m_col2 -= M.m_col2;
        m_col3 -= M.m_col3;
        return *this;
    }

    /**
     * @brief Adds another matrix to this matrix.
     * @param M The matrix to add.
     * @return A reference to the updated matrix.
     */
    inline b3Mat33& operator+=(const b3Mat33<T>& M) {
        m_col1 += M.m_col1;
        m_col2 += M.m_col2;
        m_col3 += M.m_col3;
        return *this;
    }

    /**
     * @brief Provides access to the element at the i-th row and j-th column.
     * @param i The row index.
     * @param j The column index.
     * @return A reference to the element at the specified position.
     */
    inline T& operator()(const int i, const int j) {
        b3_assert(0 <= i && i < 3 && 0 <= j && j < 3);
        return m_ts[j][i];
    }

    /**
     * @brief Returns the element at the i-th row and j-th column.
     * @param i  The row index.
     * @param j The column index.
     * @return The element at the specified position.
     */
    inline T get(int32 i ,int32 j) const {
        return m_ts[j][i];
    }

    /**
     * @brief gets the identity matrix.
     * @return Returns the identity matrix.
     */
    static constexpr b3Mat33 identity() {
        return b3Mat33(b3Vec3<T>(T(1), T(0), T(0)),
                       b3Vec3<T>(T(0), T(1), T(0)),
                       b3Vec3<T>(T(0), T(0), T(1)));
    }

    /**
     * @brief Returns the zero matrix.
     * @return The zero matrix.
     */
    static constexpr b3Mat33 zero() {
        return b3Mat33(b3Vec3<T>::zero(), b3Vec3<T>::zero(), b3Vec3<T>::zero());
    }

};


//////////////////////////////////////////////////////

using b3Mat33d = b3Mat33<double>; ///< Creates a type alias b3Mat33d for b3Mat33<double>.
using b3Mat33f = b3Mat33<float>; ///< Creates a type alias b3Mat33f for b3Mat33<float>
using b3Mat33r = b3Mat33<real>; ///< Creates a type alias b3Mat33r for b3Mat33<real>.

//////////////////////////////////////////////////////

/**
 * @brief Multiplies each element of the matrix M by the scalar s.
 * @param M The matrix to be multiplied.
 * @param s The scalar value to multiply each element of the matrix by.
 * @return A new b3Mat33<T> matrix, where each element of M has been multiplied by s
 */
template <typename T>
inline b3Mat33<T> operator*(const b3Mat33<T>& M, const T& s) {
    return b3Mat33(M.col(0) * s, M.col(1) * s, M.col(2) * s);
}

/**
 * @brief Adds two matrices M and N.
 * @param M The first matrix.
 * @param N The second matrix.
 * @return A new b3Mat33<T> matrix, where each element is the sum of the corresponding
 * elements of M and N.
 */
template <typename T>
inline b3Mat33<T> operator+(const b3Mat33<T>& M, const b3Mat33<T>& N) {
    return b3Mat33(M.col(0) + N.col(0), M.col(1) + N.col(1), M.col(2) + N.col(2));
}

/**
 * @brief Multiplies two matrices M1 and M2.
 * @param M1 The first matrix.
 * @param M2 The second matrix.
 * @return
 */
template <typename T>
inline b3Mat33<T> operator*(const b3Mat33<T>& M1, const b3Mat33<T>& M2) {

    b3Vec3<T> col1 = {M1.row(0).dot(M2.col(0)),
                      M1.row(1).dot(M2.col(0)),
                      M1.row(2).dot(M2.col(0))};

    b3Vec3<T> col2 = {M1.row(0).dot(M2.col(1)),
                      M1.row(1).dot(M2.col(1)),
                      M1.row(2).dot(M2.col(1))};

    b3Vec3<T> col3 = {M1.row(0).dot(M2.col(2)),
                      M1.row(1).dot(M2.col(2)),
                      M1.row(2).dot(M2.col(2))};

    return b3Mat33<T>(col1, col2, col3);
}

/**
 * @brief Multiplies each element of the matrix M by the scalar s.
 * @param s  The scalar value to multiply each element of the matrix by.
 * @param M The matrix to be multiplied.
 * @return A new b3Mat33<T> matrix, where each element of M has been multiplied by s.
 */
template <typename T>
inline b3Mat33<T> operator*(const T& s, const b3Mat33<T>& M) {
    return M * s;
}

/**
 * @brief Multiplies the matrix M by the vector v.
 * @param M  The matrix.
 * @param v The vector to be multiplied by the matrix.
 * @return A new b3Vec3<T> vector, which is the result of the matrix-vector multiplication.
 */
template <typename T>
inline b3Vec3<T> operator*(b3Mat33<T> M, const b3Vec3<T>& v) {
    return M.col(0) * v.x + M.col(1) * v.y + M.col(2) * v.z;
}

/**
 * @brief Multiplies the vector v by the matrix M.
 * @param v The vector.
 * @param M The matrix to be multiplied by the vector.
 * @return A new b3Vec3<T> vector, which is the result of the vector-matrix multiplication.
 */
template <typename T>
inline b3Vec3<T> operator*(const b3Vec3<T>& v, b3Mat33<T> M) {
    return b3Vec3<T>(v.dot(M.col(0)), v.dot(M.col(1)), v.dot(M.col(2)));
}

#endif //BOX3D_B3_MAT33_HPP
