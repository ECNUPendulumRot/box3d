// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BOX3D_IMGUI_EXT_HPP
#define BOX3D_IMGUI_EXT_HPP


#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

/**
 * @brief Renders a toggle button with the specified label.
 * 
 * @param str_id The label for the toggle button.
 * @param v Pointer to the boolean value representing the toggle state.
 * @return true if the button was clicked, false otherwise.
 */
bool ToggleButton(const char* str_id, bool* v)
{
    bool clicked = false;
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float width = height * 3.f;
    float radius = height * 0.50f;

    if (ImGui::InvisibleButton(str_id, ImVec2(width, height))) {
        *v = !*v;
        clicked = true;
    }

    ImU32 col_bg;
    if (ImGui::IsItemHovered())
        col_bg = *v ? IM_COL32(145+20, 211, 68+20, 255) : IM_COL32(218-20, 218-20, 218-20, 255);
    else
        col_bg = *v ? IM_COL32(145, 211, 68, 255) : IM_COL32(218, 218, 218, 255);

    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);
    draw_list->AddCircleFilled(ImVec2(*v ? (p.x + width - radius) : (p.x + radius), p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));

    ImGui::SameLine();
    ImGui::Text("%s", str_id);

    return clicked;
}

#endif //BOX3D_IMGUI_EXT_HPP
