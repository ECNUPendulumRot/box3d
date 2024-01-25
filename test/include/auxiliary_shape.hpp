
#ifndef BOX3D_AUXILIARY_SHAPE
#define BOX3D_AUXILIARY_SHAPE

#include "Eigen/Core"

class b3AABB;

class b3AuxiliaryShape {

    b3AuxiliaryShape* m_next = nullptr;

    Eigen::MatrixXd edges_left;
    Eigen::MatrixXd edges_right;
    Eigen::RowVector3d color;

public:

    b3AuxiliaryShape() = default;

    void init(b3AABB* aabb, int height);

    void set_next(b3AuxiliaryShape* next) {
        m_next = next;
    }

    b3AuxiliaryShape* next() {
        return m_next;
    }

    Eigen::MatrixXd get_edges_left() const {
        return edges_left;
    }

    Eigen::MatrixXd get_edges_right() const {
        return edges_right;
    }

    Eigen::RowVector3d get_color() const {
        return color;
    }
};


#endif // BOX3D_AUXILIARY_SHAPE