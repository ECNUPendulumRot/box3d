
#ifndef BOX3D_B3_MATH_OP_HPP
#define BOX3D_B3_MATH_OP_HPP


#include <cmath>
#include "common/b3_common.hpp"


template <typename T>
inline T b3_sqrt(T x) {
    if constexpr (std::is_same_v<T, double>)
        return sqrt(x);
    else
        return sqrtf(x);
}


template <typename T>
inline T b3_abs(T x) {
    return x > 0 ? x : -x;
}

template <typename T>
inline T b3_sin(T x) {
    if constexpr (std::is_same_v<T, double>)
        return sin(x);
    else
        return sinf(x);
}

template <typename T>
inline T b3_cos(T x) {
    if constexpr (std::is_same_v<T, double>)
        return cos(x);
    else
        return cosf(x);
}

inline void b3_round_to_zero(real& x) {
    x = b3_abs(x) < b3_real_epsilon ? real(0) : x;
}

#endif //BOX3D_B3_MATH_OP_HPP
