
#ifndef BOX3D_B3_VEC12_HPP
#define BOX3D_B3_VEC12_HPP


#include <initializer_list>

#include "math/b3_math_op.hpp"
#include "math/b3_min_max.hpp"
#include "common/b3_common.hpp"
#include <memory>


template <typename T>
struct b3Vec12 {

    union {
        T m_ts[12];
    };

    b3Vec12() {
        memset(m_ts, 0, sizeof(m_ts));
    }

    b3Vec12(const b3Vec12& other) {
        memcpy(m_ts, other.m_ts, sizeof(m_ts));
    }

    inline b3Vec12& operator=(std::initializer_list<double> list) {
        b3_assert(list.size() == 3);
        auto it = list.begin();
        for (int i = 0; i < 12; ++i) {
            m_ts[i] = *(it++);
        }

        return *this;
    }

    inline b3Vec12& operator+=(const b3Vec12& v) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] += v.m_ts[i];
        }
        return *this;
    }

    inline b3Vec12& operator-=(const b3Vec12& v) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] -= v.m_ts[i];
        }
        return *this;
    }

    template <typename U>
    inline b3Vec12& operator*=(U s) {
        for (int i = 0; i < 12; ++i) {
            m_ts[i] *= s;
        }
        return *this;
    }

    template <typename U>
    inline b3Vec12& operator/=(U s) {
        return *this *= (1.0 / (U)s);
    }

    inline T dot(const b3Vec12<T>& v) const {
        T sum = T(0);
        for (int i = 0; i < 12; ++i) {
            sum += m_ts[i] * v.m_ts[i];
        }
        return sum;
    }

    inline b3Vec12 cwise_product(const b3Vec12<T>& v) const {
        b3Vec12<T> result;
        for (int i = 0; i < 12; ++i) {
            result.m_ts[i] = m_ts[i] * v.m_ts[i];
        }
        return result;
    }

    inline bool is_zero() const{
        for (int i = 0; i < 12; ++i) {
            if (m_ts[i] != T(0)) {
                return false;
            }
        }
        return true;
    }

    inline b3Vec12 normalized() const {
        return *this / length();
    }

    inline b3Vec12 abs() const {
        b3Vec12<T> result;
        for (int i = 0; i < 12; ++i) {
            result.m_ts[i] = b3_abs(m_ts[i]);
        }
        return result;
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
        memset(m_ts, 0, sizeof(m_ts));
    }

    static b3Vec12 zero() {
        return b3Vec12();
    }

    inline T* data() {
        return m_ts;
    }

    void set_segment(const b3Vec3<T>& b, const int32 I) {
        memcpy(m_ts + I, b.m_ts, sizeof(T) * 3);
    }
};


//////////////////////////////////////////////////////

using b3Vec12d = b3Vec12<double>;
using b3Vec12f = b3Vec12<float>;
using b3Vec12r = b3Vec12<real>;

//////////////////////////////////////////////////////


template <typename T>
inline b3Vec12<T> operator+(const b3Vec12<T>& v1, const b3Vec12<T>& v2) {
    return b3Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}


template <typename T>
inline b3Vec12<T> operator-(const b3Vec12<T>& v1, const b3Vec12<T>& v2) {
    return b3Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}


template <typename T>
inline b3Vec12<T> operator-(const b3Vec12<T>& v) {
    return b3Vec3(-v.x, -v.y, -v.z);
}


template <typename T, typename U>
inline b3Vec12<T> operator*(U s, const b3Vec12<T>& v) {
    b3Vec12<T> result;
    for (int i = 0; i < 12; ++i) {
        result.m_ts[i] = s * v.m_ts[i];
    }
    return result;
}


template <typename T, typename U>
inline b3Vec12<T> operator*(const b3Vec12<T>& v, U s) {
    return s * v;
}


template <typename T>
inline b3Vec12<T> operator/(const b3Vec12<T>& v, T s) {
    return v * (T(1.0) / s) ;
}


template <typename T>
inline b3Vec12<T> b3_min_coeff(const b3Vec12<T>& a, const b3Vec12<T>& b){
    return b3Vec3(b3_min(a.x, b.x), b3_min(a.y, b.y), b3_min(a.z, b.z));
}


template <typename T>
inline b3Vec12<T> b3_max_coeff(const b3Vec12<T>& a, const b3Vec12<T>& b){
    return b3Vec3(b3_max(a.x, b.x), b3_max(a.y, b.y), b3_max(a.z, b.z));
}


#endif //BOX3D_B3_VEC12_HPP
