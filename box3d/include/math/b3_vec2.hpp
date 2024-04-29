
#ifndef BOX3D_B3_VEC2_HPP
#define BOX3D_B3_VEC2_HPP


#include <initializer_list>

#include "math/b3_math.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"


template <typename T>
struct b3Vec2 {

    union {
        T m_ts[2];
        struct {
            T x, y;
        };
    };

    b3Vec2() {
        x = y = T(0);
    }

    b3Vec2(const b3Vec2& other) {
        x = other.x;
        y = other.y;
    }

    inline b3Vec2& operator=(std::initializer_list<T> list) {
        b3_assert(list.size() == 2);
        auto it = list.begin();
        x = *it++;
        y = *it;
        return *this;
    }

    inline b3Vec2(T x, T y) {
        this->x = x;
        this->y = y;
    }

    inline void set(T x, T y) {
        this->x = x;
        this->y = y;
    }

    inline b3Vec2& operator+=(const b3Vec2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    inline b3Vec2& operator-=(const b3Vec2& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    template <typename U>
    inline b3Vec2& operator*=(U s) {
        x *= s;
        y *= s;
        return *this;
    }

    template <typename U>
    inline b3Vec2& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    inline T dot(const b3Vec2<T>& v) const {
        return x * v.x + y * v.y;
    }

    inline b3Vec2 cwise_product(const b3Vec2<T>& v) const {
        return b3Vec2(x * v.x, y * v.y);
    }

    inline bool is_zero() const{
        return b3_abs(x) < b3_real_epsilon && b3_abs(y) < b3_real_epsilon < b3_real_epsilon;
    }

    inline b3Vec2 normalized() const {
        return *this / length();
    }

    inline b3Vec2 abs() const {
        return b3Vec2(b3_abs(x), b3_abs(y));
    }

    inline b3Vec2 array_dot(const b3Vec2<T>& v) const {
        return b3Vec2(x * v.x, y * v.y);
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
        x = y = T(0);
    }

    static b3Vec2 zero() {
        return b3Vec2(T(0), T(0), T(0));
    }

    inline T* data() {
        return m_ts;
    }
};


//////////////////////////////////////////////////////

using b3Vec2d = b3Vec2<double>;
using b3Vec2f = b3Vec2<float>;
using b3Vec2r = b3Vec2<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Vec2<T> operator+(const b3Vec2<T>& v1, const b3Vec2<T>& v2) {
    return b3Vec2(v1.x + v2.x, v1.y + v2.y);
}


template <typename T>
inline b3Vec2<T> operator-(const b3Vec2<T>& v1, const b3Vec2<T>& v2) {
    return b3Vec2(v1.x - v2.x, v1.y - v2.y);
}


template <typename T>
inline b3Vec2<T> operator-(const b3Vec2<T>& v) {
    return b3Vec2(-v.x, -v.y);
}


template <typename T, typename U>
inline b3Vec2<T> operator*(U s, const b3Vec2<T>& v) {
    return b3Vec2<T>(s * v.x, s * v.y);
}


template <typename T, typename U>
inline b3Vec2<T> operator*(const b3Vec2<T>& v, U s) {
    return s * v;
}


template <typename T>
inline b3Vec2<T> operator/(const b3Vec2<T>& v, T s) {
    return v * (T(1.0) / s) ;
}



#endif //BOX3D_B3_VEC2_HPP
