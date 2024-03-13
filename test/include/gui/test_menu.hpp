
#ifndef BOX3D_TEST_MENU_HPP
#define BOX3D_TEST_MENU_HPP

#include "igl/opengl/glfw/Viewer.h"

#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"

#include "scene_test.hpp"
#include "unit_test.hpp"
#include "imgui_ext.hpp"


class TestMenu: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

    int m_selected_test = -1;

    bool m_enable_depth_test = true;
    bool m_show_auxiliary_shapes = true;
    bool m_show_ground = true;
    bool m_show_faces = true;
    bool m_show_edges = true;
    ImVec4 m_line_color = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    float m_line_width = 1.0f;

public:

    void draw_viewer_window() override {
        static int width, height;
        static ImVec2 test_list_pos, test_list_size;
        static float utility_height = 300.0f;

        glfwGetWindowSize(viewer->window, &width, &height);
        ///////////////////// The Main Menu Bar /////////////////////
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open", "Ctrl+O")) {
                }
                if (ImGui::MenuItem("Save", "Ctrl+S")) {
                }
                if (ImGui::MenuItem("Exit", "Alt+F4")) {
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Undo", "Ctrl+Z")) {
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ///////////////////////// The Test Menu /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(0.0f, ImGui::GetFrameHeight()));
        ImGui::SetNextWindowSize(ImVec2(300.0f, height - ImGui::GetFrameHeight() - utility_height), ImGuiCond_Once);
        ImGui::SetNextWindowSizeConstraints(ImVec2(250.0f, 0),
                                            ImVec2(350.0f, FLT_MAX));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Tests", &_viewer_menu_visible,
                 ImGuiWindowFlags_NoMove| ImGuiWindowFlags_NoCollapse
        );
        test_list_pos = ImGui::GetWindowPos();
        test_list_size = ImGui::GetWindowSize();
        utility_height = height - test_list_size.y - ImGui::GetFrameHeight();
        ImGuiTreeNodeFlags leaf_node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
        leaf_node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

        int category_index = 0;
        const char* category = g_test_entries[category_index].category;
        int i = 0;
        while (i < g_test_count) {
            bool category_selected = m_selected_test != -1 && strcmp(category, g_test_entries[m_selected_test].category) == 0;
            ImGuiTreeNodeFlags node_selection_flags = category_selected ? ImGuiTreeNodeFlags_Selected : 0;
            bool node_open = ImGui::TreeNodeEx(category, node_flags | node_selection_flags);

            if (node_open) {
                while (i < g_test_count && strcmp(category, g_test_entries[i].category) == 0) {
                    ImGuiTreeNodeFlags selection_flags = 0;
                    if (m_selected_test == i) {
                        selection_flags = ImGuiTreeNodeFlags_Selected;
                    }
                    ImGui::TreeNodeEx((void*)(intptr_t)i, leaf_node_flags | selection_flags, "%s", g_test_entries[i].name);
                    if (ImGui::IsItemClicked()) {
                        m_selected_test = i;
                    }
                    ++i;
                }
                ImGui::TreePop();
            }
            else {
                while (i < g_test_count && strcmp(category, g_test_entries[i].category) == 0) {
                    ++i;
                }
            }
            if (i < g_test_count) {
                category = g_test_entries[i].category;
                category_index = i;
            }
        }
        ImGui::End();

        ///////////////////////// Some Useful Tools /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(0.0f, test_list_pos.y + test_list_size.y));
        ImGui::SetNextWindowSize(ImVec2(test_list_size.x, utility_height));
        ImGui::Begin(
                "Utilities", &_viewer_menu_visible,
                ImGuiWindowFlags_NoMove| ImGuiWindowFlags_NoCollapse
        );
        ImGui::Checkbox("Enable Depth Test", &m_enable_depth_test);
        ImGui::Checkbox("Show Ground", &m_show_ground);
        ImGui::Checkbox("Show Auxiliary Shapes", &m_show_auxiliary_shapes);
        ImGui::Checkbox("Show Mesh Faces", &m_show_faces);
        ImGui::Checkbox("Show Mesh Edges", &m_show_edges);
        ImGui::ColorEdit3WithPalette("Edge Color", (float*)&m_line_color);
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("Line Width", &m_line_width, 0.5f, 3.0f);
        ImGui::PopItemWidth();
        ImGui::End();
    }

};

#endif //BOX3D_TEST_MENU_HPP
