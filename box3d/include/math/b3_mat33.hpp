
#ifndef BOX3D_B3_MAT33_HPP
#define BOX3D_B3_MAT33_HPP


#include <cstring>
#include "math/b3_vec3.hpp"
#include "common/b3_common.hpp"


template <typename T>
struct b3Mat33 {

    // m_ts is col major;
    union {
        T m_ts[3][3];

        struct {
            T m_11, m_21, m_31;
            T m_12, m_22, m_32;
            T m_13, m_23, m_33;
        };

        struct {
            b3Vec3<T> m_col1, m_col2, m_col3;
        };
    };

public:

    b3Mat33() {
        memset(m_ts, 0, sizeof(T) * 9);
    }

    b3Mat33(const b3Mat33 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 9);
    }

    b3Mat33(const b3Vec3<T>& col1, const b3Vec3<T>& col2, const b3Vec3<T>& col3) {
        m_col1 = col1;
        m_col2 = col2;
        m_col3 = col3;
    }

    inline b3Vec3<T> col(const int i) const {
        return b3Vec3<T>(m_ts[i][0], m_ts[i][1], m_ts[i][2]);
    }

    inline b3Vec3<T> row(const int i) const {
        return b3Vec3<T>(m_ts[0][i], m_ts[1][i], m_ts[2][i]);
    }

    inline b3Mat33<T> transpose() const {
        b3Mat33<T> m;

        for (int32 i = 0; i < 3; i++) {
            for (int32 j = 0; j < 3; j++) {
                m.m_ts[i][j] = m_ts[j][i];
            }
        }
        return m;
    }

    inline void set_zero() {
        m_ts = {T(0)};
    }

    inline void set_identity() {
        memset(m_ts, 0, sizeof(T) * 9);
        m_ts[0][0] = T(1);
        m_ts[1][1] = T(1);
        m_ts[2][2] = T(1);
    }

    inline T determinant() const {
        return m_11 * (m_22 * m_33 - m_32 * m_23) -
               m_21 * (m_12 * m_33 - m_32 * m_13) +
               m_31 * (m_12 * m_23 - m_22 * m_13);
    }

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

    inline real trace() const {
        return m_11 + m_22 + m_33;
    }

    static b3Mat33<T> skew_symmetric(const b3Vec3<T>& v) {
        return b3Mat33<T>(b3Vec3<T>(T(0), v.z, -v.y),
                          b3Vec3<T>(-v.z, T(0), v.x),
                          b3Vec3<T>(v.y, -v.x, T(0)));
    }

    inline b3Vec3<T> eigen_values() const {
        T e1 = trace();
        T e2 = T(0.5) * (e1 * e1 + ((*this) * (*this)).trace());
        T e3 = determinant();
        return b3Vec3<T>(e1, e2, e3);
    }

    inline b3Mat33& operator-=(const b3Mat33<T>& M) {
        m_col1 -= M.m_col1;
        m_col2 -= M.m_col2;
        m_col3 -= M.m_col3;
        return *this;
    }

    inline b3Mat33& operator+=(const b3Mat33<T>& M) {
        m_col1 += M.m_col1;
        m_col2 += M.m_col2;
        m_col3 += M.m_col3;
        return *this;
    }

    inline T& operator()(const int i, const int j) {
        b3_assert(0 <= i && i < 3 && 0 <= j && j < 3);
        return m_ts[j][i];
    }

    static constexpr b3Mat33 identity() {
        return b3Mat33(b3Vec3<T>(T(1), T(0), T(0)),
                       b3Vec3<T>(T(0), T(1), T(0)),
                       b3Vec3<T>(T(0), T(0), T(1)));
    }

    static constexpr b3Mat33 zero() {
        return b3Mat33(b3Vec3<T>::zero(), b3Vec3<T>::zero(), b3Vec3<T>::zero());
    }

};


//////////////////////////////////////////////////////

using b3Mat33d = b3Mat33<double>;
using b3Mat33f = b3Mat33<float>;
using b3Mat33r = b3Mat33<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Mat33<T> operator*(const b3Mat33<T>& M, const T& s) {
    return b3Mat33(M.col(0) * s, M.col(1) * s, M.col(2) * s);
}

template <typename T>
inline b3Mat33<T> operator+(const b3Mat33<T>& M, const b3Mat33<T>& N) {
    return b3Mat33(M.col(0) + N.col(0), M.col(1) + N.col(1), M.col(2) + N.col(2));
}

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


template <typename T>
inline b3Mat33<T> operator*(const T& s, const b3Mat33<T>& M) {
    return M * s;
}


template <typename T>
inline b3Vec3<T> operator*(b3Mat33<T> M, const b3Vec3<T>& v) {
    return M.col(0) * v.x + M.col(1) * v.y + M.col(2) * v.z;
}


template <typename T>
inline b3Vec3<T> operator*(const b3Vec3<T>& v, b3Mat33<T> M) {
    return M * v;
}

#endif //BOX3D_B3_MAT33_HPP
