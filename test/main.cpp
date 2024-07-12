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
// The MIT License

// history
//
// author           date                description
// ----------------------------------------------------------------------------
// sherman          2024-4-25           created

#include <spdlog/spdlog.h>

#include "box3d.hpp"

#include "draw.hpp"
#include "settings.hpp"
#include "test.hpp"
#include "imgui_ext.hpp"


GLFWwindow *g_main_window = nullptr;
static Settings s_settings;
static int32 s_test_selection = 0;
static Test* s_test = nullptr;
static float s_display_scale = 1.0f;
static bool s_mid_mouse_down = false;
static bool s_left_mouse_down = false;

static b3Vec3f s_click_point_ws = b3Vec3f::zero();
static b3Vec2f s_click_ss = {0, 0};

// The callback function receives the new size, in screen coordinates
// when the window is resized
static void resize_window_callback(GLFWwindow *, int width, int height) {
    g_camera.m_width = width;
    g_camera.m_height = height;
    s_settings.m_window_width = width;
    s_settings.m_window_height = height;
}


static void create_ui(GLFWwindow *window, const char *glslVersion = NULL) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    bool success;
    success = ImGui_ImplGlfw_InitForOpenGL(window, false);
    if (!success) {
        spdlog::error("ImGui_ImplGlfw_InitForOpenGL failed\n");
        assert(false);
    }

    success = ImGui_ImplOpenGL3_Init(glslVersion);
    if (!success) {
        spdlog::error("ImGui_ImplOpenGL3_Init failed\n");
        assert(false);
    }

    // TODO: add font
}


static inline bool compare_tests(const TestEntry& a, const TestEntry& b)
{
    int result = strcmp(a.category, b.category);
    if (result == 0)
    {
        result = strcmp(a.name, b.name);
    }

    return result < 0;
}


static void restart_test()
{
    delete s_test;
    s_test = g_test_entries[s_settings.m_test_index].create_fcn();
}

// The callback function receives the mouse button, button action and modifier bits.
// If you wish to be notified when a mouse button is pressed or released, set a mouse button callback.
static void mouse_button_callback(GLFWwindow* window, int32 button, int32 action, int32 mods) {

    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    // spdlog::info("mouse button callback: {}, {}, {}", xd, yd, button);

    if (action == GLFW_PRESS) {
        s_click_ss = { float(xd), float(yd)};
    }

    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            s_mid_mouse_down = true;
        } else if (action == GLFW_RELEASE) {
            s_mid_mouse_down = false;
        }
    } else if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            s_left_mouse_down = true;
        } else if (action == GLFW_RELEASE) {
            s_left_mouse_down = false;
        }
    }
}

// The callback functions receives the cursor position, measured in screen coordinates
// but relative to the top-left corner of the window content area.
// If you wish to be notified when the cursor moves over the window, set this callback.
static void mouse_motion_call_back(GLFWwindow*, double xd, double yd) {

    b3Vec2f ss = {float(xd), float(yd)};
    if (s_mid_mouse_down) {
        // this operation will not change the camera rotation
        b3Vec2f diff_ss = ss - s_click_ss;
        b3Vec3f diff_camera_r = -g_camera.m_r * diff_ss.x * 0.01f;
        b3Vec3f diff_camera_u = b3Vec3f(0, 1, 0) * diff_ss.y * 0.02f;
        g_camera.m_position += diff_camera_r;
        g_camera.m_position += diff_camera_u;
        g_camera.m_lookat += diff_camera_r;
        g_camera.m_lookat += diff_camera_u;
    } else if (s_left_mouse_down) {
        float diff_sx = ss.x - s_click_ss.x;
        g_camera.m_position += -g_camera.m_r * diff_sx * 0.04f;
        g_camera.build_up_camera_coordinate();
    }
    s_click_ss = ss;
}

// The callback function receives two-dimensional scroll offsets.
// If you wish to be notified when the user scrolls, 
// whether with a mouse wheel or touchpad gesture, set a scroll callback.
static void scroll_callback(GLFWwindow* window, double dx, double dy) {

    ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    g_camera.m_position += g_camera.m_d * float(dy) * 0.8f;
}


static void sort_tests() {
    std::sort(g_test_entries, g_test_entries + g_test_count, compare_tests);
}

// update ImGUI for the given tests
void update_ui() {

    float menuWidth = 180.0f * s_display_scale;

    ImGui::SetNextWindowPos({0.0f, 10.0f});
    ImGui::SetNextWindowSize({menuWidth, g_camera.m_height - 20.0f});

    /////////////////// Test List ///////////////////

    ImGui::Begin("Tests", &g_debug_draw.m_show_ui,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGuiTreeNodeFlags leaf_node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
    leaf_node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

    ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

    int categoryIndex = 0;
    const char *category = g_test_entries[categoryIndex].category;
    int i = 0;
    while (i < g_test_count) {
        bool categorySelected = strcmp(category, g_test_entries[s_settings.m_test_index].category) == 0;
        ImGuiTreeNodeFlags nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags_Selected : 0;
        bool nodeOpen = ImGui::TreeNodeEx(category, node_flags | nodeSelectionFlags);

        if (nodeOpen) {
            while (i < g_test_count && strcmp(category, g_test_entries[i].category) == 0) {
                ImGuiTreeNodeFlags selectionFlags = 0;
                if (s_settings.m_test_index == i) {
                    selectionFlags = ImGuiTreeNodeFlags_Selected;
                }
                ImGui::TreeNodeEx((void *) (intptr_t) i, leaf_node_flags | selectionFlags, "%s",
                                  g_test_entries[i].name);
                if (ImGui::IsItemClicked()) {
                    s_test_selection = i;
                }
                ++i;
            }
            ImGui::TreePop();
        } else {
            while (i < g_test_count && strcmp(category, g_test_entries[i].category) == 0) {
                ++i;
            }
        }

        if (i < g_test_count) {
            category = g_test_entries[i].category;
            categoryIndex = i;
        }
    }

    ImGui::End();

    /////////////////// Option List ///////////////////

    ImGui::SetNextWindowPos({g_camera.m_width - menuWidth - 10.0f, 10.0f});
    ImGui::SetNextWindowSize({menuWidth, g_camera.m_height - 20.0f});
    ImGui::Begin("Tools", &g_debug_draw.m_show_ui, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::SliderInt("Velocity Iteration", &s_settings.m_velocity_iteration, 0, 100);
    ImGui::SliderInt("Position Iteration", &s_settings.m_position_iteration, 0, 100);
    ImGui::SliderFloat("Hertz", &s_settings.m_hertz, 5.0f, 144.0f, "%.0f hz");

    ImGui::Separator();

    ImGui::Checkbox("Enable Sleep", &s_settings.m_enable_sleep);
    ImGui::Checkbox("Enable Continuous Physics", &s_settings.m_enable_continuous_physics);

    ImGui::Separator();

    ImGui::Checkbox("Shapes", &s_settings.m_draw_shapes);
    ImGui::Checkbox("Frame Only", &s_settings.m_draw_frame_only);
    ImGui::Checkbox("Contact Points", &s_settings.m_draw_contact_points);

    ImVec2 button_sz = ImVec2(-1, 0);

    if (ImGui::Button("Pause (P)", button_sz)) {
        s_settings.m_pause = !s_settings.m_pause;
        s_settings.m_generate_json = false;
    }
    if (ImGui::Button("Single Step (O)", button_sz)) {
        s_settings.m_single_step = !s_settings.m_single_step;
    }
    if (ImGui::Button("Restart (R)", button_sz)) {
        restart_test();
        s_settings.m_generate_json = false;
    }

    if (ToggleButton("Record Frame to JSON", &s_settings.m_generate_json)) {
        restart_test();
    }

    if (ToggleButton("Record Body Info to CSV", &s_settings.m_output_bodies_info)) {
        restart_test();
    }

    ImGui::End();

}


int main(int argc, char *argv[]) {

    // TODO: load settings;
    g_camera.m_width = s_settings.m_window_width;
    g_camera.m_height = s_settings.m_window_height;

    s_settings.load();
    sort_tests();

    // init glfw
    if (glfwInit() == 0) {
        spdlog::error("Failed to initialize GLFW\n");
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Title
    char buffer[128];
    sprintf(buffer, "Box3D Testbed");
    g_main_window = glfwCreateWindow(g_camera.m_width, g_camera.m_height, buffer, NULL, NULL);

    if (g_main_window == NULL) {
        fprintf(stderr, "Failed to open GLFW g_main_window.\n");
        glfwTerminate();
        return -1;
    }

    glfwGetWindowContentScale(g_main_window, &s_display_scale, &s_display_scale);

    glfwMakeContextCurrent(g_main_window);

    // If do not write these three lines, glViewPort will be wrong
    int version = gladLoadGL(glfwGetProcAddress);
    spdlog::info("GL {}.{}", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

    glfwSetWindowSizeCallback(g_main_window, resize_window_callback);
    glfwSetMouseButtonCallback(g_main_window, mouse_button_callback);
    glfwSetCursorPosCallback(g_main_window, mouse_motion_call_back);
    glfwSetScrollCallback(g_main_window, scroll_callback);
    g_debug_draw.create();

    create_ui(g_main_window, nullptr);

    // load first test
    s_settings.m_test_index = b3_clamp(s_settings.m_test_index, 0, g_test_count - 1);
    s_test_selection = s_settings.m_test_index;
    s_test = g_test_entries[s_settings.m_test_index].create_fcn();

    glClearColor(0.4f, 0.4f, 0.4f, 1.0f);

    std::chrono::duration<double> frame_time(0.0);
    std::chrono::duration<double> sleep_adjust(0.0);

    while (!glfwWindowShouldClose(g_main_window)) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        glfwGetWindowSize(g_main_window, &g_camera.m_width, &g_camera.m_height);

        int bufferWidth, bufferHeight;
        glfwGetFramebufferSize(g_main_window, &bufferWidth, &bufferHeight);
        glViewport(0, 0, bufferWidth, bufferHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(float(g_camera.m_width), float(g_camera.m_height)));
        ImGui::SetNextWindowBgAlpha(0.0f);
        ImGui::Begin("Overlay", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoScrollbar);
        ImGui::End();

        s_test->step(s_settings);

        update_ui();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(g_main_window);

        if (s_test_selection != s_settings.m_test_index) {
            s_settings.m_test_index = s_test_selection;
            s_settings.m_generate_json = false;
            delete s_test;
            s_test = g_test_entries[s_settings.m_test_index].create_fcn();
            g_camera.reset_view();
        }

        glfwPollEvents();

        // Throttle to cap at 60Hz. This adaptive using a sleep adjustment. This could be improved by
        // using mm_pause or equivalent for the last millisecond.
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> target(1.0 / 60.0);
        std::chrono::duration<double> timeUsed = t2 - t1;
        std::chrono::duration<double> sleep_time = target - timeUsed + sleep_adjust;
        if (sleep_time > std::chrono::duration<double>(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        frame_time = t3 - t1;
        sleep_adjust = 0.9 * sleep_adjust + 0.1 * (target - frame_time);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();

    s_settings.save();
}
