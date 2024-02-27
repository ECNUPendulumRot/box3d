
#ifndef BOX3D_IMGUI_EXT_HPP
#define BOX3D_IMGUI_EXT_HPP

#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuiHelpers.h"


namespace ImGui {

// A color picker created by Geogram
bool ColorEdit3or4WithPalette(const char* label, float* color_in, bool with_alpha);


bool ColorEdit3WithPalette(const char* label, float* color_in);

} // namespace ImGui



#endif //BOX3D_IMGUI_EXT_HPP
