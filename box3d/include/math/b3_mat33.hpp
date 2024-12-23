
#ifndef BOX3D_B3_MAT33_HPP
#define BOX3D_B3_MAT33_HPP


#include <cstring>
#include "math/b3_vec3.hpp"
#include "common/b3_common.hpp"
#include "b3_quat.hpp"


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

    b3Mat33(T xx, T xy, T xz, T yx, T yy, T yz, T zx, T zy, T zz) {
        m_11 = xx;
        m_12 = xy;
        m_13 = xz;
        m_21 = yx;
        m_22 = yy;
        m_23 = yz;
        m_31 = zx;
        m_32 = zy;
        m_33 = zz;
    }

    void set_skew_symmetric_matrix(const b3Vec3<T>& v) {
        m_11 = 0;
        m_12 = -v.z;
        m_13 = v.y;
        m_21 = v.z;
        m_22 = 0;
        m_23 = -v.x;
        m_31 = -v.y;
        m_32 = v.x;
        m_33 = 0;
    }

    inline b3Vec3<T> col(const int i) const {
        return b3Vec3<T>(m_ts[i][0], m_ts[i][1], m_ts[i][2]);
    }

    b3Mat33 transpose_times(const b3Mat33& m) const {
        return b3Mat33(
            m_11 * m.m_11 + m_21 * m.m_21 + m_31 * m.m_31,
            m_11 * m.m_12 + m_21 * m.m_22 + m_31 * m.m_32,
            m_11 * m.m_13 + m_21 * m.m_23 + m_31 * m.m_33,
            m_12 * m.m_11 + m_22 * m.m_21 + m_32 * m.m_31,
            m_12 * m.m_12 + m_22 * m.m_22 + m_32 * m.m_32,
            m_12 * m.m_13 + m_22 * m.m_23 + m_32 * m.m_33,
            m_13 * m.m_11 + m_23 * m.m_21 + m_33 * m.m_31,
            m_13 * m.m_12 + m_23 * m.m_22 + m_33 * m.m_32,
            m_13 * m.m_13 + m_23 * m.m_23 + m_33 * m.m_33);
    }

    b3Mat33 absolute() const {
        return b3Mat33(
            b3_abs(m_11), b3_abs(m_12), b3_abs(m_13),
            b3_abs(m_21), b3_abs(m_22), b3_abs(m_23),
            b3_abs(m_31), b3_abs(m_32), b3_abs(m_33)
            );
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

    void set_zero() {
        // m_ts = {T(0)};
        memset(m_ts, 0, sizeof(T) * 9);
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

    void set_rotation(const b3Quat<T>& q) {
        T d = q.length2();
        T s = T(2.0) / d;

        T xs = q.m_x * s, ys = q.m_y * s, zs = q.m_z * s;
        T wx = q.m_w * xs, wy = q.m_w * ys, wz = q.m_w * zs;
        T xx = q.m_x * xs, xy = q.m_x * ys, xz = q.m_x * zs;
        T yy = q.m_y * ys, yz = q.m_y * zs, zz = q.m_z * zs;

        m_11 = T(1.0) - (yy + zz);
        m_12 = xy - wz;
        m_13 = xz + wy;
        m_21 = xy + wz;
        m_22 = T(1.0) - (xx + zz);
        m_23 = yz - wx;
        m_31 = xz - wy;
        m_32 = yz + wx;
        m_33 = T(1.0) - (xx + yy);
    }

    void get_rotation(b3Quat<T>& q) const {
        T trace = m_11 + m_22 + m_33;

        if (trace > T(0.0)) {
            real s = b3_sqrt(trace + real(1.0));
            q.m_w = s * T(0.5);
            s = T(0.5) / s;

            q.m_x = (m_32 - m_23) * s;
            q.m_y = (m_13 - m_31) * s;
            q.m_z = (m_21 - m_12) * s;
        } else {
            int i = m_11 < m_22 ? (m_22 < m_33 ? 2 : 1) : (m_11 < m_33 ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            real s = b3_sqrt(m_ts[i][i] - m_ts[j][j] - m_ts[k][k] + T(1.0));
            q.m_ts[i + 1] = s * T(0.5);
            s = T(0.5) / s;

            q.m_w = (m_ts[j][k] - m_ts[k][j]) * s;
            q.m_ts[j + 1] = (m_ts[i][j] + m_ts[j][i]) * s;
            q.m_ts[k + 1] = (m_ts[i][k] + m_ts[k][i]) * s;
        }
    }

    // solve Ax = b
    b3Vec3<T> solve33(const b3Vec3r& b) const {
        real det = m_col1.dot(m_col2.cross(m_col3));
        if (b3_abs(det) > b3_real_epsilon) {
            det = 1.f / det;
        }
        b3Vec3<T> x;
        x[0] = det * b.dot(m_col2.cross(m_col3));
        x[1] = det * m_col1.dot(b.cross(m_col3));
        x[2] = det * m_col1.dot(m_col2.cross(b));
        return x;
    }

    b3Mat33 scaled(const b3Vec3<T>& s) const {
        return b3Mat33<T>(
            m_11 * s.x, m_12 * s.y, m_13 * s.z,
            m_21 * s.x, m_22 * s.y, m_23 * s.z,
            m_31 * s.x, m_32 * s.y, m_33 * s.z
            );
    }

    inline real trace() const {
        return m_11 + m_22 + m_33;
    }

    inline b3Vec3<T> to_euler_angles() const {
        b3Vec3<T> angles;
        angles.x = atan2(-m_23, m_33);
        angles.y = atan2(m_13, sqrt(m_11 * m_11 + m_12 * m_12));
        angles.z = atan2(-m_12, m_11);
        return angles;
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
b3Mat33<T> operator-(const b3Mat33<T>& m1, const b3Mat33<T>& m2) {
    return b3Mat33<T>(
        m1.m_11 - m2.m_11,
        m1.m_12 - m2.m_12,
        m1.m_13 - m2.m_13,
        m1.m_21 - m2.m_21,
        m1.m_22 - m2.m_22,
        m1.m_23 - m2.m_23,
        m1.m_31 - m2.m_31,
        m1.m_32 - m2.m_32,
        m1.m_33 - m2.m_33
        );
}

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
    auto col1 = M.col(0);
    auto col2 = M.col(1);
    auto col3 = M.col(2);
    auto x1 = M.col(0) * v.x;
    auto x2 = M.col(1) * v.y;
    auto x3 = M.col(2) * v.z;
    return M.col(0) * v.x + M.col(1) * v.y + M.col(2) * v.z;
}


template <typename T>
inline b3Vec3<T> operator*(const b3Vec3<T>& v, b3Mat33<T> M) {
    return M * v;
}

#endif //BOX3D_B3_MAT33_HPP
