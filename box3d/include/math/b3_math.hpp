
#ifndef BOX3D_B3_MATH_HPP
#define BOX3D_B3_MATH_HPP


#include <cmath>

template <typename T>
inline T b3_sqrt(T x) {
    if constexpr (std::is_same_v<T, double>)
        return sqrt(x);
    else
        return sqrtf(x);
}

#endif //BOX3D_B3_MATH_HPP