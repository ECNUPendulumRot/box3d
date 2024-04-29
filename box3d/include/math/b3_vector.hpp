
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
            T x, y, z;
        };
    };

    b3Vector3() {
        x = y = z = T(0);
    }

    b3Vector3(const b3Vector3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    inline b3Vector3& operator=(std::initializer_list<double> list) {
        b3_assert(list.size() == 3);
        auto it = list.begin();
        x = *it++;
        y = *it++;
        z = *it;

        return *this;
    }

    inline b3Vector3(T x, T y, T z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    inline void set(T x, T y, T z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    inline b3Vector3& operator+=(const b3Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    inline b3Vector3& operator-=(const b3Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    template <typename U>
    inline b3Vector3& operator*=(U s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    template <typename U>
    inline b3Vector3& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    inline T dot(const b3Vector3<T>& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    inline b3Vector3 cross(const b3Vector3<T>& v) const {
        return b3Vector3(y * v.z - z * v.y,
                         z * v.x - x * v.z,
                         x * v.y - y * v.x);
    }

    inline b3Vector3 cwise_product(const b3Vector3<T>& v) const {
        return b3Vector3(x * v.x, y * v.y, z * v.z);
    }

    inline bool is_zero() const{
        return b3_abs(x) < b3_real_epsilon && b3_abs(y) < b3_real_epsilon && b3_abs(z) < b3_real_epsilon;
    }

    inline b3Vector3 normalized() const {
        return *this / length();
    }

    inline b3Vector3 abs() const {
        return b3Vector3(b3_abs(x), b3_abs(y), b3_abs(z));
    }

    inline b3Vector3 array_dot(const b3Vector3<T>& v) const {
        return b3Vector3(x * v.x, y * v.y, z * v.z);
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
        x = y = z = T(0);
    }

    static b3Vector3 zero() {
        return b3Vector3(T(0), T(0), T(0));
    }

    inline T* data() {
        return m_ts;
    }
};


//////////////////////////////////////////////////////

using b3Vector3d = b3Vector3<double>;
using b3Vector3f = b3Vector3<float>;
using b3Vector3r = b3Vector3<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Vector3<T> operator+(const b3Vector3<T>& v1, const b3Vector3<T>& v2) {
    return b3Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}


template <typename T>
inline b3Vector3<T> operator-(const b3Vector3<T>& v1, const b3Vector3<T>& v2) {
    return b3Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}


template <typename T>
inline b3Vector3<T> operator-(const b3Vector3<T>& v) {
    return b3Vector3(-v.x, -v.y, -v.z);
}


template <typename T, typename U>
inline b3Vector3<T> operator*(U s, const b3Vector3<T>& v) {
    return b3Vector3<T>(s * v.x, s * v.y, s * v.z);
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
    return b3Vector3(b3_min(a.x, b.x), b3_min(a.y, b.y), b3_min(a.z, b.z));
}


template <typename T>
inline b3Vector3<T> b3_max_coeff(const b3Vector3<T>& a, const b3Vector3<T>& b){
    return b3Vector3(b3_max(a.x, b.x), b3_max(a.y, b.y), b3_max(a.z, b.z));
}


#endif //BOX3D_B3_VECTOR_HPP
