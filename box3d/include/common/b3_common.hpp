
#ifndef BOX3D_B3_COMMON_HPP
#define BOX3D_B3_COMMON_HPP


#include <cassert>
#include <cfloat>
#include <cmath>


#define b3_assert(A) assert(A)


#define b3_pi 3.14159265359

#define b3_close_to_zero_epsilon 1e-5

#define b3_NOT_USED(x) ((void)(x))

#define b3_length_units_per_meter 1.0

#define b3_linear_slop  (0.005 * b3_length_units_per_meter)

#define b3_polygon_radius (2.0 * b3_linear_slop)

#define b3_aabb_extension (0.1 * b3_length_units_per_meter)


using real = float;

template<typename T>
struct b3RealLimits;

template<>
struct b3RealLimits<float> {
    static constexpr float Max = FLT_MAX;
    static constexpr float Min = FLT_MIN;
    static constexpr float Epsilon = FLT_EPSILON;
};

template<>
struct b3RealLimits<double> {
    static constexpr double Max = DBL_MAX;
    static constexpr double Min = DBL_MIN;
    static constexpr double Epsilon = DBL_EPSILON;
};

constexpr auto b3_real_max = b3RealLimits<real>::Max;
constexpr auto b3_real_min = b3RealLimits<real>::Min;
constexpr auto b3_real_epsilon = b3RealLimits<real>::Epsilon;

inline bool b3_is_zero(real x) {
    return fabs(x) < b3_real_epsilon;
}

inline bool b3_close_to_zero(real x) {
    return fabs(x) < b3_close_to_zero_epsilon;
}

#endif //BOX3D_B3_COMMON_HPP
