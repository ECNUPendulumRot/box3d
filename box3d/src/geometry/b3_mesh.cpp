
#include "geometry/b3_mesh.hpp"

#include <iterator>

// This must be included because the 'cross' method is used in this header
#include <Eigen/Dense>

#include "geometry/b3_geometry.hpp"

#include "math/b3_matrix.hpp"

#include "utils/b3_io.hpp"
#include "common/b3_types.hpp"
#include "common/b3_common.hpp"
#include "common/b3_allocator.hpp"

std::vector<box3d::b3Mesh*> box3d::b3Mesh::s_meshes = std::vector<box3d::b3Mesh*>();


bool compute_mass_properties_3D(const b3MatrixXd& vertices,
                                const b3MatrixXi& faces,
                                double& mass,
                                b3Vector3d& center,
                                b3Matrix3d& inertia);

box3d::b3Mesh::b3Mesh():
    m_V(b3MatrixXd(0, 0)),
    m_E(b3MatrixXi(0, 0)),
    m_F(b3MatrixXi(0, 0))
{
    ;
}


box3d::b3Mesh::b3Mesh(const std::string &obj_file_name)
{
    read_obj(obj_file_name);
}


bool box3d::b3Mesh::read_obj(const std::string &obj_file_name) {


    // 0 0 1
    // 1 0 0
    // 0 1 0
    static b3Matrix3d transform = [](){
        b3Matrix3d m;
        m.setIdentity();
        m.col(0).swap(m.col(1));
        m.col(1).swap(m.col(2));
        return m;
    }();

    std::vector<std::vector<double>> vV, vTC, vN;
    std::vector<std::vector<int>> vF, vFTC, vFN, vL;

    bool success = ::read_obj(obj_file_name, vV, vTC, vN, vF, vFTC, vFN, vL);

    if (!success) {
        // read_obj(str,vV,vTC,vN,vF,vFTC,vFN) should have already printed
        // an error message
        return false;
    }

    bool V_rect = b3_list_to_matrix(vV, m_V);
    if (!V_rect) {
        // igl::list_to_matrix(vV,V) already printed error message
        return false;
    }

    bool F_rect = b3_list_to_matrix(vF, m_F);
    if (!F_rect) {
        // igl::list_to_matrix(vF,F) already printed error message
        return false;
    }

    std::vector<std::vector<int>> vE;
    for (const std::vector<int>& polyline : vL) {
        for (int i = 1; i < polyline.size(); i++) {
            vE.push_back({ { polyline[i - 1], polyline[i] } });
        }
    }
    bool E_rect = b3_list_to_matrix(vE, m_E);
    if (!E_rect) {
        return false;
    }
    if (m_F.size()) {
        Eigen::MatrixXi faceE;
        b3_edges(m_F, faceE);
        m_E.conservativeResize(m_E.rows() + faceE.rows(), 2);
        m_E.bottomRows(faceE.rows()) = faceE;
    }

    // transform to x-forward, y-left, z-up
    m_V = (transform * m_V.transpose()).transpose().eval();

    return true;
}


bool box3d::b3Mesh::mesh_properties(double& volume, b3PoseD& CoG, b3Inertia& Inertia) const
{
    b3Vector3d tmp_center; b3Matrix3d tmp_inertia;

    bool success = compute_mass_properties_3D(m_V, m_F, volume, tmp_center, tmp_inertia);

    if (!success)
        return false;

    CoG.set_linear(tmp_center);
    Inertia.set_inertia(tmp_inertia);

    return true;
}


void box3d::b3Mesh::recenter(const b3PoseD &new_center)
{
    // TODO: do a transform in this function
    auto eigen_new_center = new_center.linear().eigen_vector3();
    m_V.rowwise() -= eigen_new_center.transpose();
}


//void box3d::b3Mesh::transform()
//{
//    b3_assert(m_rel_pose != nullptr);
//
//    for (int32 i = 0; i < m_V.rows(); i++) {
//        auto transformed = m_rel_pose->transform(m_V.row(i).transpose());
//        m_V.row(i) = transformed.transpose();
//    }
//}

b3MatrixXd box3d::b3Mesh::transform() const
{
    return transform(m_rel_pose);
}

b3MatrixXd box3d::b3Mesh::transform(const b3PoseD* pose) const
{
    b3_assert(pose != nullptr);

    // Because the mesh vertices are rowwise, we need to transpose the matrix
    b3Matrix3d R_T = pose->rotation_matrix().transpose();
    Eigen::RowVector3d p_T = pose->linear().eigen_vector3().transpose();

    return (m_V * R_T + p_T.replicate(m_V.rows(), 1)).eval();
}


Eigen::Vector3d box3d::b3Mesh::get_support(const Eigen::Vector3d wd) {
    
    // transform d to body coordinate, and don not care p
    b3Matrix3d R_T = m_rel_pose->rotation_matrix().transpose();
    Eigen::Vector3d p = m_rel_pose->linear().eigen_vector3();

    Eigen::Vector3d local_d = R_T * wd;

    double max_value = m_V.row(0).dot(local_d);
    int max_index = 0;

    b3_assert(m_V.rows() >= 1);
    double value = 0;

    for(int i = 1; i < m_V.rows(); ++i) {
        value = m_V.row(i).dot(local_d);
        if(value > max_value) {
            max_value = value;
            max_index = i;
        }
    }

    // transform vertex to world coordinate
    return m_V.row(max_index) * R_T + p.transpose();
}


box3d::b3AABB box3d::b3Mesh::get_bounding_aabb() const
{
    auto v_trans = transform();
    Eigen::Vector3d aabb_min = v_trans.colwise().minCoeff();
    Eigen::Vector3d aabb_max = v_trans.colwise().maxCoeff();

    return b3AABB{aabb_min, aabb_max};
}


// The method is introduced by Mirtich and utilized by Geometric Tools Engine
// For more details please access:
// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
bool compute_mass_properties_3D(const b3MatrixXd& vertices,
                                const b3MatrixXi& faces,
                                double& mass,
                                b3Vector3d& center,
                                b3Matrix3d& inertia)
{
    if (faces.size() == 0 || faces.cols() != 3) {
        return false;
    }
    assert(vertices.cols() == 3);
    assert(faces.cols() == 3);

    // order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
    Eigen::Matrix<double, 10, 1> integral =
            Eigen::Matrix<double, 10, 1>::Zero();

    for (int i = 0; i < faces.rows(); i++) {
        // Get vertices of triangle i.
        const Eigen::Vector3d& v0 = vertices.row(faces(i, 0));
        const Eigen::Vector3d& v1 = vertices.row(faces(i, 1));
        const Eigen::Vector3d& v2 = vertices.row(faces(i, 2));

        // Get cross product of edges and normal vector.
        const Eigen::Vector3d& V1mV0 = v1 - v0;
        const Eigen::Vector3d& V2mV0 = v2 - v0;
        const Eigen::Vector3d& N = V1mV0.cross(V2mV0);

        // Compute integral terms.
        double tmp0, tmp1, tmp2;
        double f1x, f2x, f3x, g0x, g1x, g2x;
        tmp0 = v0.x() + v1.x();
        f1x = tmp0 + v2.x();
        tmp1 = v0.x() * v0.x();
        tmp2 = tmp1 + v1.x() * tmp0;
        f2x = tmp2 + v2.x() * f1x;
        f3x = v0.x() * tmp1 + v1.x() * tmp2 + v2.x() * f2x;
        g0x = f2x + v0.x() * (f1x + v0.x());
        g1x = f2x + v1.x() * (f1x + v1.x());
        g2x = f2x + v2.x() * (f1x + v2.x());

        double f1y, f2y, f3y, g0y, g1y, g2y;
        tmp0 = v0.y() + v1.y();
        f1y = tmp0 + v2.y();
        tmp1 = v0.y() * v0.y();
        tmp2 = tmp1 + v1.y() * tmp0;
        f2y = tmp2 + v2.y() * f1y;
        f3y = v0.y() * tmp1 + v1.y() * tmp2 + v2.y() * f2y;
        g0y = f2y + v0.y() * (f1y + v0.y());
        g1y = f2y + v1.y() * (f1y + v1.y());
        g2y = f2y + v2.y() * (f1y + v2.y());

        double f1z, f2z, f3z, g0z, g1z, g2z;
        tmp0 = v0.z() + v1.z();
        f1z = tmp0 + v2.z();
        tmp1 = v0.z() * v0.z();
        tmp2 = tmp1 + v1.z() * tmp0;
        f2z = tmp2 + v2.z() * f1z;
        f3z = v0.z() * tmp1 + v1.z() * tmp2 + v2.z() * f2z;
        g0z = f2z + v0.z() * (f1z + v0.z());
        g1z = f2z + v1.z() * (f1z + v1.z());
        g2z = f2z + v2.z() * (f1z + v2.z());

        // Update integrals.
        integral[0] += N.x() * f1x;
        integral[1] += N.x() * f2x;
        integral[2] += N.y() * f2y;
        integral[3] += N.z() * f2z;
        integral[4] += N.x() * f3x;
        integral[5] += N.y() * f3y;
        integral[6] += N.z() * f3z;
        integral[7] += N.x() * (v0.y() * g0x + v1.y() * g1x + v2.y() * g2x);
        integral[8] += N.y() * (v0.z() * g0y + v1.z() * g1y + v2.z() * g2y);
        integral[9] += N.z() * (v0.x() * g0z + v1.x() * g1z + v2.x() * g2z);
    }

    integral[0] /= 6;
    integral[1] /= 24;
    integral[2] /= 24;
    integral[3] /= 24;
    integral[4] /= 60;
    integral[5] /= 60;
    integral[6] /= 60;
    integral[7] /= 120;
    integral[8] /= 120;
    integral[9] /= 120;

    // mass
    mass = integral[0];
    if (mass <= 0 || !std::isfinite(mass)) {
        return false;
    }
    assert(mass > 0);

    // center of mass
    center = Eigen::Vector3d(integral[1], integral[2], integral[3]) / mass;

    // inertia relative to world origin
    inertia.resize(3, 3);
    inertia(0, 0) = integral[5] + integral[6];
    inertia(0, 1) = -integral[7];
    inertia(0, 2) = -integral[9];
    inertia(1, 0) = inertia(0, 1);
    inertia(1, 1) = integral[4] + integral[6];
    inertia(1, 2) = -integral[8];
    inertia(2, 0) = inertia(0, 2);
    inertia(2, 1) = inertia(1, 2);
    inertia(2, 2) = integral[4] + integral[5];

    // inertia relative to center of mass
    inertia(0, 0) -= mass * (center.y() * center.y() + center.z() * center.z());
    inertia(0, 1) += mass * center.x() * center.y();
    inertia(0, 2) += mass * center.z() * center.x();
    inertia(1, 0) = inertia(0, 1);
    inertia(1, 1) -= mass * (center.z() * center.z() + center.x() * center.x());
    inertia(1, 2) += mass * center.y() * center.z();
    inertia(2, 0) = inertia(0, 2);
    inertia(2, 1) = inertia(1, 2);
    inertia(2, 2) -= mass * (center.x() * center.x() + center.y() * center.y());

    return true;
}


box3d::b3Mesh*  box3d::b3Mesh::create_mesh(const std::filesystem::path &file_path)
{
    std::string fs_string = file_path.string();

    void* memory = b3_alloc(sizeof(b3Mesh));

    auto* mesh = new(memory) b3Mesh(fs_string);

    s_meshes.push_back(mesh);

    return mesh;
}





