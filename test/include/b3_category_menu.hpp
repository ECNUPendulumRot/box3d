
#ifndef BOX3D_B3_CATEGORY_MENU_HPP
#define BOX3D_B3_CATEGORY_MENU_HPP

#include "igl/opengl/glfw/Viewer.h"

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include "b3_test.hpp"

class CategoryMenu: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

    int m_selected_test = -1;

public:

    void draw_viewer_window() override {

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Tests", &_viewer_menu_visible,
                ImGuiWindowFlags_NoSavedSettings
        );
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
        if (callback_draw_viewer_menu) { callback_draw_viewer_menu(); }
        else { draw_viewer_menu(); }
        ImGui::PopItemWidth();
        ImGui::End();
    }

    void draw_viewer_menu() override {

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

    }

};

#endif //BOX3D_B3_CATEGORY_MENU_HPP
