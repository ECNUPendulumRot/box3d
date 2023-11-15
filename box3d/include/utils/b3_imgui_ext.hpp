//
// Created by sherman on 23-11-15.
//

#ifndef BOX3D_B3_IMGUI_EXT_HPP
#define BOX3D_B3_IMGUI_EXT_HPP

#include <Eigen/Core>

#include "imgui.h"
#include <limits>
#include "nlohmann/json.hpp"

#define CCD_IM_ARRAYSIZE(_ARR) (int(sizeof(_ARR) / sizeof(*_ARR)))

namespace ImGui {

    bool InputIntBounded(
            const char* label,
            int* val,
            int lower_bound = std::numeric_limits<int>::min(),
            int upper_bound = std::numeric_limits<int>::max(),
            int step = 1,
            int step_fast = 100,
            ImGuiInputTextFlags flags = 0);

    bool InputDoubleBounded(
            const char* label,
            double* val,
            double lower_bound = -std::numeric_limits<double>::infinity(),
            double upper_bound = std::numeric_limits<double>::infinity(),
            double step = 1,
            double step_fast = 100,
            const char* format = "%.6f",
            ImGuiInputTextFlags flags = 0);

    bool DragDouble(
            const char* label,
            double* v,
            double v_speed = 1.0,
            double v_min = 0.0,
            double v_max = 0.0,
            const char* format = "%.3f",
            float power = 1.0f); // If v_min >= v_max we have no bound

    void TreeNodeJson(const nlohmann::json json);
    bool DoubleColorEdit3(const char* label, Eigen::RowVector3d& color);
    void HelpMarker(const char* desc);
} // namespace ImGui


#endif //BOX3D_B3_IMGUI_EXT_HPP
