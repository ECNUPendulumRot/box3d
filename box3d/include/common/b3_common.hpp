
#ifndef BOX3D_B3_COMMON_HPP
#define BOX3D_B3_COMMON_HPP


#include <cassert>
#include <cfloat>
#include <cmath>

#define b3_assert(A) assert(A)

#define b3_double_epsilon DBL_EPSILON
#define b3_float_epsilon FLT_EPSILON

#define b3_pi 3.14159265359

#define b3_close_to_zero 1e-5

#define b3_NOT_USED(x) ((void)(x))

#define b3_length_units_per_meter 1.0

#define b3_linear_slop  (0.005 * b3_length_units_per_meter)

#define b3_polygon_radius (2.0 * b3_linear_slop)

#define b3_aabb_extension (0.1 * b3_length_units_per_meter)

#define b3_max_double DBL_MAX

#endif //BOX3D_B3_COMMON_HPP
