//
// Created by sherman on 23-11-15.
//
#include "utils/b3_igl_viewer_ext.hpp"

#include <igl/colormap.h>
#include <igl/isolines.h>
#include <igl/slice.h>
#include <igl/slice_mask.h>

namespace igl {
    namespace opengl {
        ViewerDataExt::ViewerDataExt(
                igl::opengl::glfw::Viewer* _viewer, const Eigen::RowVector3d& color)
                : m_viewer(_viewer)
                , m_color(color)
                , show_vertex_data(false)
                , m_use_log_scale(true)
                , m_normalized(false)
                , m_scaling(1.0)
        {
            data_id = m_viewer->data_list.size() - 1;
        }
        ViewerDataExt::~ViewerDataExt() {}

        Eigen::MatrixXd ViewerDataExt::set_vertices(const Eigen::MatrixXd& V)
        {
            Eigen::MatrixXd V_temp;

            // If P only has two columns, pad with a column of zeros
            if (V.cols() == 2) {
                V_temp = Eigen::MatrixXd::Zero(V.rows(), 3);
                V_temp.block(0, 0, V.rows(), 2) = V;
            } else {
                V_temp = V;
            }
            data().set_vertices(V_temp);

            Eigen::MatrixXd nz = Eigen::MatrixXd::Zero(V.rows(), 3);
            nz.col(2).setConstant(1.0);
            data().set_normals(nz);
            return V_temp;
        }

        void ViewerDataExt::set_points(
                const Eigen::MatrixXd& V, const Eigen::MatrixXd& color)
        {
            Eigen::MatrixXd V_temp;

            // If P only has two columns, pad with a column of zeros
            if (V.cols() == 2) {
                V_temp = Eigen::MatrixXd::Zero(V.rows(), 3);
                V_temp.block(0, 0, V.rows(), 2) = V;
            } else {
                V_temp = V;
            }
            data().set_points(V_temp, color);
        }
        void ViewerDataExt::set_edges(
                const Eigen::MatrixXd& V,
                const Eigen::MatrixXi& E,
                const Eigen::MatrixXd& color)
        {
            Eigen::MatrixXd V_temp;

            // If P only has two columns, pad with a column of zeros
            if (V.cols() == 2) {
                V_temp = Eigen::MatrixXd::Zero(V.rows(), 3);
                V_temp.block(0, 0, V.rows(), 2) = V;
            } else {
                V_temp = V;
            }
            data().set_edges(V_temp, E, color);
        }
        void ViewerDataExt::set_faces(
                const Eigen::MatrixXd& V,
                const Eigen::MatrixXi& F,
                const Eigen::MatrixXd& color)
        {
            data().V = Eigen::MatrixXd(0, 3);
            data().F = Eigen::MatrixXi(0, 3);
            data().set_mesh(V, F);
            data().set_colors(color);
            data().show_lines = false;
        }

        ///////////////////////////////////////////////////////////////////////////
        /// \brief MeshData::MeshData
        /// \param _viewer
        ////////////////////////////////////////////////////////////////////////
        MeshData::MeshData(
                igl::opengl::glfw::Viewer* _viewer, const Eigen::RowVector3d& color)
                : ViewerDataExt(_viewer, color)
        {
            m_edge_color = Eigen::RowVector3d::Zero();                    // #000000
            m_static_color = Eigen::RowVector3d(0xB3, 0xB3, 0xB3) / 0xFF; // #B3B3B3
            m_kinematic_color =
                    Eigen::RowVector3d(0xFF, 0x80, 0x00) / 0xFF; // #FF8000
        }

        void MeshData::set_mesh(
                const Eigen::MatrixXd& V,
                const Eigen::MatrixXi& E,
                const Eigen::MatrixXi& F)
        {
            data().clear();
            mV = set_vertices(V);
            set_points(V, F.size() ? m_edge_color : m_color);
            set_edges(V, E, F.size() ? m_edge_color : m_color);
            if (F.size()) {
                set_faces(V, F, m_color);
            }

            mE = E;
            mF = F;
        }

        void MeshData::update_vertices(const Eigen::MatrixXd& V)
        {
            mV = set_vertices(V);
            recolor();
            data().labels_positions = mV;
        }

        void MeshData::recolor()
        {
            assert(mV.rows() == m_vertex_type.rows());

            Eigen::MatrixXd vertex_colors(mV.rows(), 3);
            Eigen::MatrixXd edge_vertex_colors(mE.rows(), 3);
            Eigen::MatrixXd face_vertex_colors(mV.rows(), 3);

            auto type_color = [&](int type) {
                return type == 0 ? m_static_color
                                 : (type == 1 ? m_kinematic_color : m_color);
            };

            for (int i = 0; i < mV.rows(); i++) {
                vertex_colors.row(i) =
                        mF.size() ? m_edge_color : type_color(m_vertex_type(i));
                face_vertex_colors.row(i) = type_color(m_vertex_type(i));
            }

            for (int i = 0; i < mE.rows(); i++) {
                edge_vertex_colors.row(i) = mF.size()
                                            ? m_edge_color
                                            : type_color(std::min(
                                m_vertex_type(mE(i, 0)), m_vertex_type(mE(i, 1))));
            }

            set_points(mV, vertex_colors);
            if (mE.size()) {
                set_edges(mV, mE, edge_vertex_colors);
            }
            if (mF.size()) {
                set_faces(mV, mF, face_vertex_colors);
            }
        }

        void MeshData::set_vertex_data(
                const Eigen::MatrixX<bool>& vtx_data, const Eigen::VectorXi& vertex_type)
        {
            assert(vtx_data.rows() == mV.rows());

            vertex_data_labels.clear();
            vertex_data_labels.resize(size_t(vtx_data.rows()));
            for (size_t i = 0; i < vtx_data.rows(); i++) {
                std::string data_i = std::to_string(i) + ":";
                for (size_t j = 0; j < vtx_data.cols(); j++) {
                    if (vtx_data(i, j)) {
                        data_i += std::to_string(j) + ",";
                    }
                }
                vertex_data_labels[i] = data_i.substr(0, data_i.size() - 1);
            }
            show_vertex_data = true;
            data().labels_positions = mV;
            data().labels_strings = vertex_data_labels;

            m_vertex_type = vertex_type;
            recolor();
        }

        void MeshData::update_vertex_data()
        {
            if (show_vertex_data) {
                data().labels_positions = mV;
                data().labels_strings = vertex_data_labels;
            } else {
                data().labels_positions.resize(0, 0);
            }
        }

    } // namespace opengl
} // namespace igl

