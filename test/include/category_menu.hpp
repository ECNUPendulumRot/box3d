
#ifndef BOX3D_CATEGORY_MENU_HPP
#define BOX3D_CATEGORY_MENU_HPP

#include "igl/opengl/glfw/Viewer.h"

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include "scene_test.hpp"
#include "unit_test.hpp"

class CategoryMenu: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

    int m_app_window_size[2] = {0, 0};

    int m_selected_scene_test = -1;

    int m_selected_unit_test = -1;

public:

    void draw_viewer_window() override {

        glfwGetWindowSize(viewer->window, &m_app_window_size[0], &m_app_window_size[1]);

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

        ///////////////////////// The Scene Menu /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(0.0f, ImGui::GetFrameHeight()), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(250.0f, 350.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(250.0f, 0.0f), ImVec2(350.0f, FLT_MAX));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Scenes", &_viewer_menu_visible,
                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove| ImGuiWindowFlags_NoCollapse
        );
        ImVec2 scene_window_pos = ImGui::GetWindowPos();
        ImVec2 scene_window_size = ImGui::GetWindowSize();
        ImGuiTreeNodeFlags leaf_node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
        leaf_node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

        int category_index = 0;
        const char* category = g_scene_test_entries[category_index].category;
        int i = 0;
        while (i < g_scene_test_count) {
            bool category_selected = m_selected_scene_test != -1 && strcmp(category, g_scene_test_entries[m_selected_scene_test].category) == 0;
            ImGuiTreeNodeFlags node_selection_flags = category_selected ? ImGuiTreeNodeFlags_Selected : 0;
            bool node_open = ImGui::TreeNodeEx(category, node_flags | node_selection_flags);

            if (node_open) {
                while (i < g_scene_test_count && strcmp(category, g_scene_test_entries[i].category) == 0) {
                    ImGuiTreeNodeFlags selection_flags = 0;
                    if (m_selected_scene_test == i) {
                        selection_flags = ImGuiTreeNodeFlags_Selected;
                    }
                    ImGui::TreeNodeEx((void*)(intptr_t)i, leaf_node_flags | selection_flags, "%s", g_scene_test_entries[i].name);
                    if (ImGui::IsItemClicked()) {
                        m_selected_scene_test = i;
                        m_selected_unit_test = -1;
                    }
                    ++i;
                }
                ImGui::TreePop();
            }
            else {
                while (i < g_scene_test_count && strcmp(category, g_scene_test_entries[i].category) == 0) {
                    ++i;
                }
            }
            if (i < g_scene_test_count) {
                category = g_scene_test_entries[i].category;
                category_index = i;
            }
        }
        ImGui::End();

        ///////////////////////// The Unit Test Menu /////////////////////////
        ImGui::SetNextWindowPos(ImVec2(0.0f, scene_window_pos.y + scene_window_size.y));
        ImGui::SetNextWindowSize(ImVec2(scene_window_size.x, m_app_window_size[1] - (ImGui::GetFrameHeight() + scene_window_size.y)));
        ImGui::SetNextWindowSizeConstraints(ImVec2(250.0f, 0.0f), ImVec2(350.0f, FLT_MAX));
        ImGui::Begin(
                "Unit Tests", &_viewer_menu_visible,
                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize
        );

        for (int i = 0; i < g_unit_test_count; ++i) {
            ImGuiTreeNodeFlags selection_flags = 0;
            if (m_selected_unit_test == i) {
                selection_flags = ImGuiTreeNodeFlags_Selected;
            }
            ImGui::TreeNodeEx((void*)(intptr_t)i, leaf_node_flags | selection_flags, "%s", g_unit_test_entries[i].name);
            if (ImGui::IsItemClicked()) {
                m_selected_unit_test = i;
                m_selected_scene_test = -1;
            }
        }
        ImGui::End();
    }

};

#endif //BOX3D_CATEGORY_MENU_HPP
