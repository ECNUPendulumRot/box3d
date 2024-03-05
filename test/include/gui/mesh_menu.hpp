
#ifndef BOX3D_MESH_MENU_HPP
#define BOX3D_MESH_MENU_HPP


#include "igl/opengl/glfw/Viewer.h"

#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"

#include "spdlog/spdlog.h"

#include "test.hpp"
#include "imgui_ext.hpp"


struct MeshViewObject {

    int shape_id;
    ImVec4 color;

    MeshViewObject(int shape_id, ImVec4 color) : shape_id(shape_id), color(color) {}
};


enum class UserShapeType {
    Point,
    Line,
    Plane
};


struct UserShape {
    UserShapeType type;
    char name[20];
    float data[6];
    float size;
    ImVec4 color;

    void get_point(Eigen::RowVector3d& point, Eigen::RowVector3d& color) {
        point = Eigen::RowVector3d(data[0], data[1], data[2]);
        color = Eigen::RowVector3d(this->color.x, this->color.y, this->color.z);
    }

    void get_plane(Eigen::MatrixXd& vertex, Eigen::MatrixXi& face) {
        vertex.resize(4, 3);
        face.resize(2, 3);

        vertex << data[0], data[1], data[2],
                data[0] + data[3], data[1], data[2],
                data[0], data[1] + data[3], data[2],
                data[0], data[1], data[2] + data[3];

        face << 0, 1, 2,
                0, 2, 3;
    }
};

class MeshMenu: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

    int m_selected_test = -1;

    float list_width;

    std::vector<MeshViewObject> m_index_colors;

    TestBase* m_test;

    std::vector<UserShape> m_user_shapes;



public:

    void draw_viewer_window() override {
        int width, height;

        glfwGetWindowSize(viewer->window, &width, &height);

        ///////////////////////// The Mesh List View /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(width - list_width, ImGui::GetFrameHeight()));
        float list_height = (float(height) - ImGui::GetFrameHeight()) / 2;
        ImGui::SetNextWindowSizeConstraints(ImVec2(300.0f, list_height), ImVec2(350.0f, list_height));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Mesh List", &_viewer_menu_visible,
                ImGuiWindowFlags_NoMove
        );
        bool collapsed = ImGui::IsWindowCollapsed();
        for (int i = 0; i < m_index_colors.size(); i++) {
            if (ImGui::Selectable(std::to_string(i).c_str(), m_selected_test == i, ImGuiSelectableFlags_SpanAllColumns)) {
                m_selected_test = i;

                if (m_test != nullptr)
                    m_test->selected_object(m_selected_test);
            }

            if (m_selected_test == i) {
                ImGui::ColorEdit3("Color", (float*)&m_index_colors[i].color);
                ImGui::SliderFloat("Alpha", &m_index_colors[i].color.w, 0.0f, 1.0f);
            }
        }
        list_width = ImGui::GetWindowWidth();
        ImGui::End();

        ///////////////////////// User-added Mesh /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(width - list_width, ImGui::GetFrameHeight() + list_height));
        ImGui::SetNextWindowSize(ImVec2(list_width, list_height));
        ImGui::Begin(
                "User-added Shapes", &_viewer_menu_visible,
                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
        );
        ImGui::SetWindowCollapsed(collapsed);
        for (int i = 0; i < m_user_shapes.size(); i++) {
            auto& shape = m_user_shapes[i];

            ImGui::PushID(i);
            ImGui::Separator();
            ImGui::InputText("Name", shape.name, 20);

            switch (m_user_shapes[i].type) {
                case UserShapeType::Point:
                    show_point(shape);
                    break;
//                case UserShapeType::Line:
//                    show_line(shape);
//                    break;
                case UserShapeType::Plane:
                    show_plane(shape);
                    break;
            }

            if (ImGui::Button("Delete")) {
                m_user_shapes.erase(m_user_shapes.begin() + i);
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
        ImGui::Separator();

        user_added_mesh();

        ImGui::End();
    }

    void add_object(const int& shape_id) {
        m_index_colors.emplace_back(shape_id, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    }

    void clear() {
        m_index_colors.clear();
    }

    void set_test(TestBase* test) {
        m_test = test;
    }

    void check_delete_shape(const int& index) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && ImGui::IsItemHovered()) {
            ImGui::OpenPopup("MyPopup");
        }
        if (ImGui::BeginPopup("MyPopup")) {
            // 在弹出式菜单中添加选项
            ImGui::Text("Add a shape:");
            ImGui::Separator();

            do {
                if (ImGui::MenuItem("Delete")) {
                    m_user_shapes.erase(m_user_shapes.begin() + index);
                    break;
                }
            } while (false);

            ImGui::EndPopup();
        }
    }

    void user_added_mesh() {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && ImGui::IsWindowHovered()) {
            // 打开一个上下文菜单
            ImGui::OpenPopup("MyPopup");
        }

        if (ImGui::BeginPopup("MyPopup")) {
            // 在弹出式菜单中添加选项
            ImGui::Text("Add a shape:");
            ImGui::Separator();

            do {
                if (ImGui::MenuItem("Point")) {
                    m_user_shapes.push_back({UserShapeType::Point, "Point",
                                             {0.0f, 0.0f, 0.0f}, 5.0f});
                    break;
                }
//                if (ImGui::MenuItem("Line")) {
//                    m_user_shapes.push_back({UserShapeType::Line, "Line",
//                                             {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f}, 5.0f});
//                    break;
//                }
                if (ImGui::MenuItem("Plane")) {
                    m_user_shapes.push_back({UserShapeType::Plane, "Plane",
                                             {0.0f, 0.0f, 0.0f, 1.0f}, 0.1f});
                    break;
                }
            } while (false);

            ImGui::EndPopup();
        }
    }

    void show_point(UserShape& shape) {

        ImGui::ColorEdit3WithPalette("", (float*)&shape.color);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Size", &shape.size, 1.0f, 10.0f);
        ImGui::InputFloat3("Position", &shape.data[0],"%.2f");

    }

    void show_line(UserShape& shape) {
        // TODO:
    }

    void show_plane(UserShape& shape) {

        ImGui::ColorEdit3WithPalette("", (float*)&shape.color);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Alpha", &shape.color.w, 0.0f, 1.0f);

        ImGui::SetNextItemWidth(40);
        ImGui::InputFloat("x", &shape.data[0], 0.0f, 0.0f, "%.1f");
        ImGui::SameLine(); // 保持在同一行
        ImGui::Text("+");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(40);
        ImGui::InputFloat("y", &shape.data[1], 0.0f, 0.0f, "%.1f");
        ImGui::SameLine(); // 保持在同一行
        ImGui::Text("+");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(40);
        ImGui::InputFloat("z", &shape.data[2], 0.0f, 0.0f, "%.1f");
        ImGui::SameLine(); // 保持在同一行
        ImGui::Text("=");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(40);
        ImGui::InputFloat(" ", &shape.data[3], 0.0f, 0.0f, "%.1f");

    }

    void draw_shape_in_viewer(igl::opengl::glfw::Viewer& viewer, const int& index) {

        for (int i = 0; i < m_user_shapes.size(); i++) {
            auto& shape = m_user_shapes[i];
            switch (shape.type) {
                case UserShapeType::Point:
                    Eigen::RowVector3d point;
                    Eigen::RowVector3d color;
                    shape.get_point(point, color);
                    viewer.data(index + i).add_points(point, color);
                    viewer.data(index + i).point_size = shape.size;
                    break;
            }
        }

    }
};


#endif //BOX3D_MESH_MENU_HPP
