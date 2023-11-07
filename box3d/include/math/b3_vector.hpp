
#ifndef BOX3D_B3_VECTOR_HPP
#define BOX3D_B3_VECTOR_HPP

#include <Eigen/Core>

#include "math/b3_min_max.hpp"


namespace box3d {

    template <typename T>
    class b3Vector3;

}


template <typename T>
class box3d::b3Vector3 {

    union {
        T m_ts[3];
        struct {
            T m_x, m_y, m_z;
        };
    };

public:

    b3Vector3() {
        m_x = m_y = m_z = T(0);
    }

    explicit inline b3Vector3(const Eigen::Vector3<T>& v) {
        m_x = v.x();
        m_y = v.y();
        m_z = v.z();
    }

    inline b3Vector3(T x, T y, T z) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline b3Vector3& operator=(const Eigen::Vector3<T>& v) {
        m_x = v.x();
        m_y = v.y();
        m_z = v.z();
        return *this;
    }

    inline b3Vector3& operator+=(const b3Vector3& v) {
        m_x += v.x;
        m_y += v.y;
        m_z += v.z;
        return *this;
    }

    inline b3Vector3& operator-=(const b3Vector3& v) {
        m_x -= v.x;
        m_y -= v.y;
        m_z -= v.z;
        return *this;
    }

    template <typename U>
    inline b3Vector3& operator*=(U s) {
        m_x *= s;
        m_y *= s;
        m_z *= s;
        return *this;
    }

    template <typename U>
    inline b3Vector3& operator/=(U s) {
        return *this *= (U(1.0) / s);
    }

    inline T dot(const b3Vector3<T>& v) const {
        return m_x * v.x + m_y * v.y + m_z * v.z;
    }

    inline b3Vector3 cross(const b3Vector3<T>& v) const {
        return b3Vector3(m_y * v.z - m_z * v.y,
                         m_z * v.x - m_x * v.z,
                         m_x * v.y - m_y * v.x);
    }

    inline void set_zero() {
        m_x = m_y = m_z = T(0);
    }

    static b3Vector3 zero() {
        return b3Vector3(T(0), T(0), T(0));
    }

    inline T x() const {
        return m_x;
    }

    inline T y() const {
        return m_y;
    }

    inline T z() const {
        return m_z;
    }
};


template <typename T>
inline box3d::b3Vector3<T> operator+(const box3d::b3Vector3<T>& v1,
                                     const box3d::b3Vector3<T>& v2) {
    return box3d::b3Vector3(v1.x() + v2.x(),
                            v1.y() + v2.y(),
                            v1.z() + v2.z());
}

template <typename T>
inline box3d::b3Vector3<T> operator-(const box3d::b3Vector3<T>& v1,
                                     const box3d::b3Vector3<T>& v2) {
    return box3d::b3Vector3(v1.x() - v2.x(),
                            v1.y() - v2.y(),
                            v1.z() - v2.z());
}

template <typename T>
inline box3d::b3Vector3<T> operator-(const box3d::b3Vector3<T>& v) {
    return box3d::b3Vector3(-v.x(),
                            -v.y(),
                            -v.z());
}

template <typename T>
inline box3d::b3Vector3<T> operator*(T s, const box3d::b3Vector3<T>& v) {
    return box3d::b3Vector3(s * v.x(),
                            s * v.y(),
                            s * v.z());
}

template <typename T>
inline box3d::b3Vector3<T> operator*(const box3d::b3Vector3<T>& v, T s) {
    return s * v;
}

template <typename T>
inline box3d::b3Vector3<T> b3_min_coeff(const box3d::b3Vector3<T>& a, const box3d::b3Vector3<T>& b){
    return box3d::b3Vector3(b3_min(a.x(), b.x()), b3_min(a.y(), b.y()), b3_min(a.z(), b.z()));
}

template <typename T>
inline box3d::b3Vector3<T> b3_max_coeff(const box3d::b3Vector3<T>& a, const box3d::b3Vector3<T>& b){
    return box3d::b3Vector3(b3_max(a.x(), b.x()), b3_max(a.y(), b.y()), b3_max(a.z(), b.z()));
}

#endif //BOX3D_B3_VECTOR_HPP
