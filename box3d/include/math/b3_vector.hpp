
#ifndef BOX3D_B3_VECTOR_HPP
#define BOX3D_B3_VECTOR_HPP


#include <initializer_list>

#include "math/b3_math.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"


template <typename T>
struct b3Vector3 {

    union {
        T m_ts[3];
        struct {
            T m_x, m_y, m_z;
        };
    };

    b3Vector3() {
        m_x = m_y = m_z = T(0);
    }

    b3Vector3(const b3Vector3& other) {
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }

    inline b3Vector3& operator=(std::initializer_list<double> list) {
        b3_assert(list.size() == 3);
        auto it = list.begin();
        m_x = *it++;
        m_y = *it++;
        m_z = *it;

        return *this;
    }

    inline b3Vector3(T x, T y, T z) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline void set(T x, T y, T z) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline b3Vector3& operator+=(const b3Vector3& v) {
        m_x += v.m_x;
        m_y += v.m_y;
        m_z += v.m_z;
        return *this;
    }

    inline b3Vector3& operator-=(const b3Vector3& v) {
        m_x -= v.m_x;
        m_y -= v.m_y;
        m_z -= v.m_z;
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
        return *this *= (1.0 / (U)s);
    }

    inline T dot(const b3Vector3<T>& v) const {
        return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
    }

    inline b3Vector3 cross(const b3Vector3<T>& v) const {
        return b3Vector3(m_y * v.m_z - m_z * v.m_y,
                         m_z * v.m_x - m_x * v.m_z,
                         m_x * v.m_y - m_y * v.m_x);
    }

    inline b3Vector3 cwise_product(const b3Vector3<T>& v) const {
        return b3Vector3(m_x * v.m_x, m_y * v.m_y, m_z * v.m_z);
    }

    inline bool is_zero() const{
        return b3_abs(m_x) < b3_real_epsilon && b3_abs(m_y) < b3_real_epsilon && b3_abs(m_z) < b3_real_epsilon;
    }

    inline b3Vector3 normalized() const {
        return *this / length();
    }

    inline b3Vector3 abs() const {
        return b3Vector3(b3_abs(m_x), b3_abs(m_y), b3_abs(m_z));
    }

    inline b3Vector3 array_dot(const b3Vector3<T>& v) const {
        return b3Vector3(m_x * v.m_x, m_y * v.m_y, m_z * v.m_z);
    }

    inline T length() const {
        return b3_sqrt<T>(length2());
    }

    inline T length2() const {
        return dot(*this);
    }

    inline T operator[](int i) const {
        return m_ts[i];
    }

    inline T& operator[](int i) {
        return m_ts[i];
    }

    inline void set_zero() {
        m_x = m_y = m_z = T(0);
    }

    static b3Vector3 zero() {
        return b3Vector3(T(0), T(0), T(0));
    }

    inline const T& x() const {
        return m_x;
    }

    inline const T& y() const {
        return m_y;
    }

    inline const T& z() const {
        return m_z;
    }

};


//////////////////////////////////////////////////////

using b3Vector3d = b3Vector3<double>;
using b3Vector3f = b3Vector3<float>;
using b3Vector3r = b3Vector3<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Vector3<T> operator+(const b3Vector3<T>& v1, const b3Vector3<T>& v2) {
    return b3Vector3(v1.x() + v2.x(),
                     v1.y() + v2.y(),
                     v1.z() + v2.z());
}


template <typename T>
inline b3Vector3<T> operator-(const b3Vector3<T>& v1, const b3Vector3<T>& v2) {
    return b3Vector3(v1.x() - v2.x(),
                     v1.y() - v2.y(),
                     v1.z() - v2.z());
}


template <typename T>
inline b3Vector3<T> operator-(const b3Vector3<T>& v) {
    return b3Vector3(-v.x(),
                     -v.y(),
                     -v.z());
}


template <typename T, typename U>
inline b3Vector3<T> operator*(U s, const b3Vector3<T>& v) {
    return b3Vector3<T>(s * v.x(),
                        s * v.y(),
                        s * v.z());
}


template <typename T, typename U>
inline b3Vector3<T> operator*(const b3Vector3<T>& v, U s) {
    return s * v;
}


template <typename T>
inline b3Vector3<T> operator/(const b3Vector3<T>& v, T s) {
    return v * (T(1.0) / s) ;
}


template <typename T>
inline b3Vector3<T> b3_min_coeff(const b3Vector3<T>& a, const b3Vector3<T>& b){
    return b3Vector3(b3_min(a.x(), b.x()), b3_min(a.y(), b.y()), b3_min(a.z(), b.z()));
}


template <typename T>
inline b3Vector3<T> b3_max_coeff(const b3Vector3<T>& a, const b3Vector3<T>& b){
    return b3Vector3(b3_max(a.x(), b.x()), b3_max(a.y(), b.y()), b3_max(a.z(), b.z()));
}


#endif //BOX3D_B3_VECTOR_HPP
