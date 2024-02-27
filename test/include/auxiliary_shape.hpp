
#ifndef BOX3D_AUXILIARY_SHAPE
#define BOX3D_AUXILIARY_SHAPE

#include "Eigen/Core"

template <typename T, int Major>
using ConstMapMatrixX = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Major>>;

template <typename T, int Major>
using MapMatrixX = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Major>>;

class b3AABB;

class b3AuxiliaryShape {

    b3AuxiliaryShape* m_next = nullptr;

    std::vector<double> edge_left_v;
    std::vector<double> edge_right_v;

    std::vector<double> point_v;

    Eigen::RowVector3d color = Eigen::RowVector3d(0, 0, 0);

public:

    b3AuxiliaryShape() = default;

    void set_color(const Eigen::RowVector3d& color) {
        this->color = color;
    }

    void add_line(const double& x1, const double& y1, const double& z1,
                  const double& x2, const double& y2, const double& z2) {
        edge_left_v.emplace_back(x1);
        edge_left_v.emplace_back(y1);
        edge_left_v.emplace_back(z1);
        edge_right_v.emplace_back(x2);
        edge_right_v.emplace_back(y2);
        edge_right_v.emplace_back(z2);
    }

    void add_point(const double& x, const double& y, const double& z) {
        point_v.emplace_back(x);
        point_v.emplace_back(y);
        point_v.emplace_back(z);
    }

    void set_next(b3AuxiliaryShape* next) {
        m_next = next;
    }

    b3AuxiliaryShape* next() {
        return m_next;
    }

    Eigen::RowVector3d get_color() const {
        return color;
    }

    Eigen::MatrixXd get_edges_left_m() const {
        return ConstMapMatrixX<double, Eigen::RowMajor>(edge_left_v.data(), edge_left_v.size() / 3, 3);
    }

    Eigen::MatrixXd get_edges_right_m() const {
        return ConstMapMatrixX<double, Eigen::RowMajor>(edge_right_v.data(), edge_right_v.size() / 3, 3);
    }

    Eigen::MatrixXd get_points_m() const {
        return ConstMapMatrixX<double, Eigen::RowMajor>(point_v.data(), point_v.size() / 3, 3);
    }

};


#endif // BOX3D_AUXILIARY_SHAPE