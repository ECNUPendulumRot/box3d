
#ifndef BOX3D_B3_TYPES_HPP
#define BOX3D_B3_TYPES_HPP

#include <Eigen/Core>

#include "math/b3_vector.hpp"

using int8   = signed char;

using int16  = signed short;

using int32  = signed int;

using uint8  = unsigned char;

using uint16 = unsigned short;

using uint32 = unsigned int ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
using b3Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using b3MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using b3Vector12 = Eigen::Matrix<T, 12, 1>;

using b3Vector3d = box3d::b3Vector3<double>;
using b3Vector3f = box3d::b3Vector3<float>;

using b3Matrix3d = b3Matrix3<double>;
using b3Matrix3f = b3Matrix3<float>;

using b3MatrixXi = b3MatrixX<int>;
using b3MatrixXd = b3MatrixX<double>;
using b3MatrixXf = b3MatrixX<float>;

using b3Vector12d = b3Vector12<double>;
using b3Vector12f = b3Vector12<float>;


#endif //BOX3D_B3_TYPES_HPP
