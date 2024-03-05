
#ifndef BOX3D_TITLE_BAR_HPP
#define BOX3D_TITLE_BAR_HPP

#include "igl/opengl/glfw/Viewer.h"

#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"


class TitleBar: public igl::opengl::glfw::imgui::ImGuiMenu {

    friend class b3GUIViewer;

public:

    void draw_viewer_window() override {
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
    }

};


#endif //BOX3D_TITLE_BAR_HPP
