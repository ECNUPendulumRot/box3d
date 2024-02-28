
#ifndef BOX3D_MESH_LIST_VIEW_HPP
#define BOX3D_MESH_LIST_VIEW_HPP


#include "igl/opengl/glfw/Viewer.h"

#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"

#include "spdlog/spdlog.h"

#include "test.hpp"


struct MeshViewObject {
    int shape_id;
    ImVec4 color;

    MeshViewObject(int shape_id, ImVec4 color) : shape_id(shape_id), color(color) {}
};

class MeshListView: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

    int m_selected_test = -1;

    float list_width;

    std::vector<MeshViewObject> m_index_colors;

    TestBase* m_test;

public:

    void draw_viewer_window() override {
        int width, height;

        static bool first_time = true;

        glfwGetWindowSize(viewer->window, &width, &height);

        ///////////////////////// The Test Menu /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(width - list_width, ImGui::GetFrameHeight()));
        float list_height = float(height) - ImGui::GetFrameHeight();
        ImGui::SetNextWindowSizeConstraints(ImVec2(250.0f, list_height), ImVec2(350.0f, list_height));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Mesh List", &_viewer_menu_visible,
                ImGuiWindowFlags_NoMove
        );
        if (first_time) {
            ImGui::SetWindowCollapsed(true);
            first_time = false;
        }
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
    }

    void add_object(const int& shape_id) {
        spdlog::info("Adding object to list");

        m_index_colors.emplace_back(shape_id, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    }

    void clear() {
        m_index_colors.clear();
    }

    void set_test(TestBase* test) {
        m_test = test;
    }
};


#endif //BOX3D_MESH_LIST_VIEW_HPP
