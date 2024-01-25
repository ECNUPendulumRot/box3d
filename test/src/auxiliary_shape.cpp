
#include "auxiliary_shape.hpp"
#include "box3d.hpp"


static Eigen::RowVector3d colors[10] = {
        Eigen::RowVector3d(0, 0, 0),
        Eigen::RowVector3d(252.0 / 255, 230.0 / 255, 202.0 / 255),
        Eigen::RowVector3d(1, 0, 0),
        Eigen::RowVector3d(156.0 / 255, 102.0 / 255, 31.0 / 255),
        Eigen::RowVector3d(0, 1, 0),
        Eigen::RowVector3d(0, 1, 1),
        Eigen::RowVector3d(0, 0, 1),
        Eigen::RowVector3d(128.0 / 255, 42.0 / 255, 42.0 / 255),
        Eigen::RowVector3d(160.0 / 255, 82.0 / 255, 45.0 / 255),
        Eigen::RowVector3d(153.0 / 255, 51.0 / 255, 250.0 / 255)
};


void b3AuxiliaryShape::init(b3AABB *aabb, int height) {
    color = colors[height % 10];


    b3Vector3d min = aabb->min();
    b3Vector3d max = aabb->max();

    edges_left.resize(12, 3);
    edges_right.resize(12, 3);

    edges_left.row(0) << min.x(), min.y(), min.z(),
    edges_left.row(1) << max.x(), min.y(), min.z(),
    edges_left.row(2) << max.x(), max.y(), min.z(),
    edges_left.row(3) << min.x(), max.y(), min.z(),
    edges_left.row(4) << min.x(), min.y(), min.z(),
    edges_left.row(5) << max.x(), min.y(), min.z(),
    edges_left.row(6) << max.x(), max.y(), min.z(),
    edges_left.row(7) << min.x(), max.y(), min.z(),
    edges_left.row(8) << min.x(), min.y(), max.z(),
    edges_left.row(9) << max.x(), min.y(), max.z(),
    edges_left.row(10) << max.x(), max.y(), max.z(),
    edges_left.row(11) << min.x(), max.y(), max.z();

    edges_right.row(0) << max.x(), min.y(), min.z(),
    edges_right.row(1) << max.x(), max.y(), min.z(),
    edges_right.row(2) << min.x(), max.y(), min.z(),
    edges_right.row(3) << min.x(), min.y(), min.z(),
    edges_right.row(4) << min.x(), min.y(), max.z(),
    edges_right.row(5) << max.x(), min.y(), max.z(),
    edges_right.row(6) << max.x(), max.y(), max.z(),
    edges_right.row(7) << min.x(), max.y(), max.z(),
    edges_right.row(8) << max.x(), min.y(), max.z(),
    edges_right.row(9) << max.x(), max.y(), max.z(),
    edges_right.row(10) << min.x(), max.y(), max.z(),
    edges_right.row(11) << min.x(), min.y(), max.z();
}