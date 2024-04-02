
#ifndef BOX3D_B3_MATRIX_HPP
#define BOX3D_B3_MATRIX_HPP


#include <cstring>
#include "math/b3_vector.hpp"
#include "common/b3_common.hpp"


template <typename T>
struct b3Matrix3 {

    // m_ts is col major;
    union {
        T m_ts[9];

        struct {
            T m_11, m_21, m_31;
            T m_12, m_22, m_32;
            T m_13, m_23, m_33;
        };

        struct {
            b3Vector3<T> m_col1, m_col2, m_col3;
        };
    };

public:

    b3Matrix3() {
        memset(m_ts, 0, sizeof(T) * 9);
    }

    b3Matrix3(const b3Matrix3 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 9);
    }

    b3Matrix3(const b3Vector3<T>& col1, const b3Vector3<T>& col2, const b3Vector3<T>& col3) {
        m_col1 = col1;
        m_col2 = col2;
        m_col3 = col3;
    }

    inline b3Vector3<T> col(const int i) const {
        return b3Vector3<T>(m_ts[i * 3], m_ts[i * 3 + 1], m_ts[i * 3 + 2]);
    }

    inline b3Vector3<T> row(const int i) const {
        return b3Vector3<T>(m_ts[i], m_ts[i + 3], m_ts[i + 6]);
    }

    inline b3Matrix3<T> transpose() const {
        b3Matrix3<T> m;
        m.m_11 = m_11;
        m.m_21 = m_12;
        m.m_31 = m_13;
        m.m_12 = m_21;
        m.m_22 = m_22;
        m.m_32 = m_23;
        m.m_13 = m_31;
        m.m_23 = m_32;
        m.m_33 = m_33;
        return m;
    }

    inline void set_zero() {
        m_11 = T(0);
        m_21 = T(0);
        m_31 = T(0);
        m_12 = T(0);
        m_22 = T(0);
        m_32 = T(0);
        m_13 = T(0);
        m_23 = T(0);
        m_33 = T(0);
    }

    inline void set_identity() {
        m_11 = T(1);
        m_21 = T(0);
        m_31 = T(0);
        m_12 = T(0);
        m_22 = T(1);
        m_32 = T(0);
        m_13 = T(0);
        m_23 = T(0);
        m_33 = T(1);
    }

    inline T determinant() const {
        return m_11 * (m_22 * m_33 - m_32 * m_23) -
               m_21 * (m_12 * m_33 - m_32 * m_13) +
               m_31 * (m_12 * m_23 - m_22 * m_13);
    }

    inline b3Matrix3 inverse() {

        double det = determinant();
        b3_assert(det > 0);
        double inv_det = 1.0 / det;

        b3Matrix3 inv;
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

    inline real trace() const {
        return m_11 + m_22 + m_33;
    }

    static b3Matrix3<T> skew_symmetric(const b3Vector3<T>& v) {

        return b3Matrix3<T>(b3Vector3<T>(T(0), v.z(), -v.y()),
                            b3Vector3<T>(-v.z(), T(0), v.x()),
                            b3Vector3<T>(v.y(), -v.x(), T(0)));

    }

    inline b3Vector3<T> eigen_values() const {
        T e1 = trace();
        T e2 = T(0.5) * (e1 * e1 + ((*this) * (*this)).trace());
        T e3 = determinant();

        return b3Vector3<T>(e1, e2, e3);
    }

    inline b3Matrix3& operator-=(const b3Matrix3<T>& M) {
        m_col1 -= M.m_col1;
        m_col2 -= M.m_col2;
        m_col3 -= M.m_col3;
        return *this;
    }

    inline b3Matrix3& operator+=(const b3Matrix3<T>& M) {
        m_col1 += M.m_col1;
        m_col2 += M.m_col2;
        m_col3 += M.m_col3;
        return *this;
    }

    inline T& operator()(const int i, const int j) {
        b3_assert(0 <= i && i < 3 && 0 <= j && j < 3);
        return m_ts[3 * j + i];
    }

    static b3Matrix3 identity() {
        return b3Matrix3(b3Vector3<T>(T(1), T(0), T(0)),
                         b3Vector3<T>(T(0), T(1), T(0)),
                         b3Vector3<T>(T(0), T(0), T(1)));
    }

    static b3Matrix3 zero() {
        return b3Matrix3(b3Vector3<T>::zero(), b3Vector3<T>::zero(), b3Vector3<T>::zero());
    }

};

//////////////////////////////////////////////////////

using b3Matrix3d = b3Matrix3<double>;
using b3Matrix3f = b3Matrix3<float>;
using b3Matrix3r = b3Matrix3<real>;

//////////////////////////////////////////////////////

template <typename T>
inline b3Matrix3<T> operator*(const b3Matrix3<T>& M, const T& s) {
    return b3Matrix3(M.col(0) * s, M.col(1) * s, M.col(2) * s);
}

template <typename T>
inline b3Matrix3<T> operator*(const b3Matrix3<T>& M1, const b3Matrix3<T>& M2) {

    b3Vector3<T> col1 = {M1.row(0).dot(M2.col(0)),
                         M1.row(1).dot(M2.col(0)),
                         M1.row(2).dot(M2.col(0))};

    b3Vector3<T> col2 = {M1.row(0).dot(M2.col(1)),
                         M1.row(1).dot(M2.col(1)),
                         M1.row(2).dot(M2.col(1))};

    b3Vector3<T> col3 = {M1.row(0).dot(M2.col(2)),
                         M1.row(1).dot(M2.col(2)),
                         M1.row(2).dot(M2.col(2))};

    return b3Matrix3<T>(col1, col2, col3);
}


template <typename T>
inline b3Matrix3<T> operator*(const T& s, const b3Matrix3<T>& M) {
    return M * s;
}


template <typename T>
inline b3Vector3<T> operator*(b3Matrix3<T> M, const b3Vector3<T>& v) {
    return M.col(0) * v.x() + M.col(1) * v.y() + M.col(2) * v.z();
}


template <typename T>
inline b3Vector3<T> operator*(const b3Vector3<T>& v, b3Matrix3<T> M) {
    return M * v;
}

#endif //BOX3D_B3_MATRIX_HPP
