
#ifndef BOX3D_B3_TYPES_HPP
#define BOX3D_B3_TYPES_HPP

#include <Eigen/Core>

template <typename T>
using b3Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using b3Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using b3MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

using b3Vector3d = b3Vector3<double>;
using b3Vector3f = b3Vector3<float>;

using b3Matrix3d = b3Matrix3<double>;
using b3Matrix3f = b3Matrix3<float>;

using b3MatrixXi = b3MatrixX<int>;
using b3MatrixXd = b3MatrixX<double>;
using b3MatrixXf = b3MatrixX<float>;

#endif //BOX3D_B3_TYPES_HPP
