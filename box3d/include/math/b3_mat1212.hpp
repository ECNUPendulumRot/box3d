
#ifndef BOX3D_B3_MAT1212_HPP
#define BOX3D_B3_MAT1212_HPP


#include <cstring>
#include "math/b3_vec12.hpp"
#include "common/b3_common.hpp"


template <typename T>
struct b3Mat1212 {

    // m_ts is col major;
    union {
        T m_ts[12][12];
    };

public:

    b3Mat1212() {
        memset(m_ts, 0, sizeof(T) * 144);
    }

    b3Mat1212(const b3Mat1212 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 144);
    }

    inline b3Vec12<T> col(const int i) const {
        b3Vec12<T> col;
        memset(col.m_ts, &m_ts[i], sizeof(T) * 12);
        return col;
    }

    inline b3Vec12<T> row(const int i) const {
        b3Vec12<T> row;
        for (int j = 0; j < 12; ++j) {
            row.m_ts[j] = m_ts[j][i];
        }
        return row;
    }

    inline b3Mat1212<T> transpose() const {
        b3Mat1212<T> m;

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

    inline T& operator()(const int i, const int j) {
        b3_assert(0 <= i && i < 3 && 0 <= j && j < 3);
        return m_ts[j][i];
    }

};


//////////////////////////////////////////////////////

using b3Mat1212d = b3Mat1212<double>;
using b3Mat1212f = b3Mat1212<float>;
using b3Mat1212r = b3Mat1212<real>;

//////////////////////////////////////////////////////


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


template <typename T>
inline b3Mat1212<T> operator*(const T& s, const b3Mat1212<T>& M) {
    return M * s;
}


template <typename T>
inline b3Vec12<T> operator*(b3Mat1212<T> M, const b3Vec12<T>& v) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result[i] = M.col(i).dot(v);
    }
    return result;
}


template <typename T>
inline b3Vec12<T> operator*(const b3Vec12<T>& v, b3Mat1212<T> M) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result[i] = T(0);
        for (int j = 0; j < 12; ++j) {
            result[i] += M(j, i) * v[j];
        }
    }
    return result;
}


#endif //BOX3D_B3_MAT1212_HPP
