
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

template <typename T>
using b3Vector9 = Eigen::Matrix<T, 9, 1>;

template <typename T>
using b3Matrix9 = Eigen::Matrix<T, 9, 9>;

template <typename T>
using b3Matrix12 = Eigen::Matrix<T, 12, 12>;

using b3Vector3d = box3d::b3Vector3<double>;
using b3Vector3f = box3d::b3Vector3<float>;

using b3Vector12d = b3Vector12<double>;
using b3Vector12f = b3Vector12<float>;

using b3Matrix3d = b3Matrix3<double>;
using b3Matrix3f = b3Matrix3<float>;

using b3Vector9d = b3Vector9<double>;
using b3Vector9f = b3Vector9<float>;


using b3Matrix9d = b3Matrix9<double>;
using b3Matrix9f = b3Matrix9<float>;

using b3Matrix12d = b3Matrix12<double>;
using b3Matrix12f = b3Matrix12<float>;

using b3MatrixXi = b3MatrixX<int>;
using b3MatrixXd = b3MatrixX<double>;
using b3MatrixXf = b3MatrixX<float>;

#endif //BOX3D_B3_TYPES_HPP
