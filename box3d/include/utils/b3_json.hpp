
#ifndef BOX3D_B3_JSON_HPP
#define BOX3D_B3_JSON_HPP


#include <nlohmann/json.hpp>

#include <Eigen/Core>

template <typename T, int dim, int max_dim>
void from_json(const nlohmann::json& json, Eigen::Matrix<T, dim, 1, Eigen::ColMajor, max_dim, 1>& vector)
{
    typedef Eigen::Matrix<T, dim, 1, Eigen::ColMajor, max_dim, 1> Vector;
    typedef std::vector<T> L;
    L list = json.template get<L>();
    vector = Eigen::Map<Vector>(list.data(), long(list.size()));
}


#endif //BOX3D_B3_JSON_HPP
