
#ifndef BOX3D_B3_TYPES_HPP
#define BOX3D_B3_TYPES_HPP

#include <Eigen/Core>

#include "math/b3_vector.hpp"
#include "math/b3_matrix.hpp"


using int8   = signed char;

using int16  = signed short;

using int32  = signed int;

using uint8  = unsigned char;

using uint16 = unsigned short;

using uint32 = unsigned int ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
using E3Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using E3MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using E3Vector12 = Eigen::Matrix<T, 12, 1>;

template <typename T>
using E3Vector9 = Eigen::Matrix<T, 9, 1>;

template <typename T>
using E3Matrix9 = Eigen::Matrix<T, 9, 9>;

template <typename T>
using E3Matrix12 = Eigen::Matrix<T, 12, 12>;

using E3Vector12d = E3Vector12<double>;
using E3Vector12f = E3Vector12<float>;

using E3Matrix3d = E3Matrix3<double>;
using E3Matrix3f = E3Matrix3<float>;

using E3Vector9d = E3Vector9<double>;
using E3Vector9f = E3Vector9<float>;

using E3Matrix9d = E3Matrix9<double>;
using E3Matrix9f = E3Matrix9<float>;

using E3Matrix12d = E3Matrix12<double>;
using E3Matrix12f = E3Matrix12<float>;

using E3MatrixXi = E3MatrixX<int>;
using E3MatrixXd = E3MatrixX<double>;
using E3MatrixXf = E3MatrixX<float>;

using b3Vector3d = box3d::b3Vector3<double>;
using b3Vector3f = box3d::b3Vector3<float>;

using b3Matrix3d = box3d::b3Matrix3<double>;
using b3Matrix3f = box3d::b3Matrix3<float>;


#endif //BOX3D_B3_TYPES_HPP
