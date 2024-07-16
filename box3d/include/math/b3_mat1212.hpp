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


#ifndef BOX3D_B3_MAT1212_HPP
#define BOX3D_B3_MAT1212_HPP


#include <cstring>
#include "math/b3_vec12.hpp"
#include "common/b3_common.hpp"

/**
 * @brief b3Mat1212 is a template structure representing a 12x12 matrix with various
 * utility functions for matrix operations.
 */
template <typename T>
struct b3Mat1212 {

    // m_ts is col major;
    /**
     * @brief A 12x12 matrix stored in column-major order.
     */
    union {
        T m_ts[12][12];
    };

public:

    /**
     * @brief constructor of b3Mat1212. Initializes a 12x12 matrix with all elements set to zero.
     */
    b3Mat1212() {
        memset(m_ts, 0, sizeof(T) * 144);
    }

    /**
     * @brief constructor of b3Mat1212. Initializes a new matrix as a copy of another matrix.
     * @param other The matrix to copy from.
     */
    b3Mat1212(const b3Mat1212 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 144);
    }

    /**
     * @brief Retrieves the i-th column of the matrix.
     * @param i The index of the column to retrieve.
     * @return A b3Vec12<T> vector representing the i-th column of the matrix.
     */
    inline b3Vec12<T> col(const int i) const {
        b3Vec12<T> col;
        memcpy(col.m_ts, &m_ts[i], sizeof(T) * 12);
        return col;
    }

    /**
     * @brief Retrieves the i-th row of the matrix.
     * @param i The index of the row to retrieve.
     * @return A b3Vec12<T> vector representing the i-th row of the matrix.
     */
    inline b3Vec12<T> row(const int i) const {
        b3Vec12<T> row;
        for (int j = 0; j < 12; ++j) {
            row.m_ts[j] = m_ts[j][i];
        }
        return row;
    }

    /**
     * @brief Computes the transpose of the matrix.
     * @return A new b3Mat1212<T> matrix that is the transpose of the original matrix.
     */
    inline b3Mat1212<T> transpose() const {
        b3Mat1212<T> m;

        for (int32 i = 0; i < 12; i++) {
            for (int32 j = 0; j < 12; j++) {
                m.m_ts[i][j] = m_ts[j][i];
            }
        }
        return m;
    }

    /**
     * @brief Sets all elements of the matrix to zero.
     */
    inline void set_zero() {
        memset(m_ts, 0, sizeof(T) * 144);
    }

    /**
     * @brief Sets the matrix to the identity matrix.
     */
    inline void set_identity() {
        memset(m_ts, 0, sizeof(T) * 144);
        m_ts[0][0] = T(1);
        m_ts[1][1] = T(1);
        m_ts[2][2] = T(1);
    }

    /**
     * @brief Provides access to the element at the i-th row and j-th column of the matrix.
     * @param i The row index.
     * @param j The column index.
     * @return A reference to the element at the specified position.
     */
    inline T& operator()(const int i, const int j) {
        b3_assert(0 <= i && i < 12 && 0 <= j && j < 12);
        return m_ts[j][i];
    }

    /**
     * @brief Sets a 3x3 block of the matrix starting at position (i, j) with the values
     * from the b3Mat33 matrix M.
     * @param M The 3x3 matrix to copy values from.
     * @param i The starting row index for the block.
     * @param j The starting column index for the block.
     */
    inline void set_block(const b3Mat33<T>& M, const int& i, const int& j) {
        b3_assert(0 <= i && i <= 9 && 0 <= j && j <= 9);
        for (int k = 0; k < 3; ++k) {
            for (int l = 0; l < 3; ++l) {
                m_ts[j + l][i + k] = M.get(k, l);
            }
        }
    }

};


//////////////////////////////////////////////////////

using b3Mat1212d = b3Mat1212<double>;
using b3Mat1212f = b3Mat1212<float>;
using b3Mat1212r = b3Mat1212<real>;

//////////////////////////////////////////////////////

/**
 * @brief Multiplies each element of the matrix M by the scalar s.
 * @param M  The matrix to be multiplied.
 * @param s  The scalar value to multiply each element of the matrix by.
 * @return A new b3Mat1212<T> matrix with each element multiplied by s.
 */
template <typename T>
inline b3Mat1212<T> operator*(const b3Mat1212<T>& M, const T& s) {
    b3Mat1212<T> result;
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 12; ++j) {
            result(i, j) = M(i, j) * s;
        }
    }
    return result;
}

/**
 * @brief Multiplies each element of the matrix M by the scalar s.
 * @param s  The scalar value to multiply each element of the matrix by.
 * @param M The matrix to be multiplied.
 * @return A new b3Mat1212<T> matrix with each element multiplied by s.
 */
template <typename T>
inline b3Mat1212<T> operator*(const T& s, const b3Mat1212<T>& M) {
    return M * s;
}

/**
 * @brief Multiplies the matrix M by the vector v.
 * @param M The matrix.
 * @param v The vector to be multiplied by the matrix.
 * @return A new b3Vec12<T> vector, which is the result of the matrix-vector multiplication.
 */
template <typename T>
inline b3Vec12<T> operator*(b3Mat1212<T> M, const b3Vec12<T>& v) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result += M.col(i) * v[i];
    }
    return result;
}

/**
 * @brief Multiplies the vector v by the matrix M.
 * @param v The vector.
 * @param M The matrix to be multiplied by the vector.
 * @return A new b3Vec12<T> vector, which is the result of the vector-matrix
 * multiplication.
 */
template <typename T>
inline b3Vec12<T> operator*(const b3Vec12<T>& v, b3Mat1212<T> M) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result[i] = v.dot(M.col(i));
    }
    return result;
}


#endif //BOX3D_B3_MAT1212_HPP
