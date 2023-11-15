//
// Created by sherman on 23-11-15.
//
#include <iostream>
#include <Eigen/Core>

int main(int argc, char* argv[]) {

    Eigen::MatrixXd m = Eigen::Matrix3d();

    std::cout << sizeof(Eigen::MatrixXd) << std::endl;
    std::cout << sizeof(Eigen::MatrixXi) << std::endl;
    std::cout << sizeof(Eigen::MatrixXf) << std::endl;


    std::cout << sizeof(decltype(m)::Scalar) * m.size() << std::endl;


    return 0;
}
