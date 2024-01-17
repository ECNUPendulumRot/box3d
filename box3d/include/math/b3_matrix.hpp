
#ifndef BOX3D_B3_MATRIX_HPP
#define BOX3D_B3_MATRIX_HPP


#include <memory>

#include "math/b3_vector.hpp"


namespace box3d {

    template <typename T>
    class b3Matrix3;
}


template <typename T>
inline box3d::b3Vector3<T> operator*(box3d::b3Matrix3<T> M, const box3d::b3Vector3<T>& v) {

    return M.col(0) * v.x() + M.col(1) * v.y() + M.col(2) * v.z();

}


template <typename T>
class box3d::b3Matrix3 {

    // m_ts is col major;
    union {
        T m_ts[9];
        struct {
            T m_11, m_21, m_31;
            T m_12, m_22, m_32;
            T m_13, m_23, m_33;
        };
    };

public:

    b3Matrix3() {
        memset(m_ts, 0, sizeof(T) * 9);
    }

    b3Matrix3(const b3Matrix3 &other) {
        memcpy(m_ts, other.m_ts, sizeof(T) * 9);
    }

    explicit b3Matrix3(const Eigen::Matrix3<T>& m) {
        m_11 = m(0, 0);
        m_21 = m(1, 0);
        m_31 = m(2, 0);
        m_12 = m(0, 1);
        m_22 = m(1, 1);
        m_32 = m(2, 1);
        m_13 = m(0, 2);
        m_23 = m(1, 2);
        m_33 = m(2, 2);
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
        m_ts = {T(0)};
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

};




#endif //BOX3D_B3_MATRIX_HPP
