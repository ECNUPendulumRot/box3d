
#include <spdlog/spdlog.h>

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "box3d.hpp"

#include "draw.hpp"
#include "settings.hpp"
#include "test.hpp"

GLFWwindow *g_mainWindow = nullptr;
static Settings s_settings;
static int32 s_test_selection = 0;
static Test* s_test = nullptr;
static float s_display_scale = 1.0f;

static bool s_mid_mouse_down = false;
static bool s_left_mouse_down = false;

static b3Vec3f s_click_point_ws = b3Vec3f::zero();
static b3Vec2f s_click_ss = {0, 0};

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


static void mouse_button_callback(GLFWwindow* window, int32 button, int32 action, int32 mods) {

    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    spdlog::info("mouse button callback: {}, {}, {}", xd, yd, button);

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

}


int main(int argc, char *argv[]) {

    // TODO: load settings;
    g_camera.m_width = s_settings.m_window_width;
    g_camera.m_height = s_settings.m_window_height;

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
    g_mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, buffer, NULL, NULL);

    if (g_mainWindow == NULL) {
        fprintf(stderr, "Failed to open GLFW g_mainWindow.\n");
        glfwTerminate();
        return -1;
    }

    glfwGetWindowContentScale(g_mainWindow, &s_display_scale, &s_display_scale);

    glfwMakeContextCurrent(g_mainWindow);

    // If do not write these three lines, glViewPort will be wrong
    int version = gladLoadGL(glfwGetProcAddress);
    spdlog::info("GL {}.{}", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));
    spdlog::info("OpenGL {}, GLSL {}", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glfwSetWindowSizeCallback(g_mainWindow, resize_window_callback);
    glfwSetMouseButtonCallback(g_mainWindow, mouse_button_callback);
    glfwSetCursorPosCallback(g_mainWindow, mouse_motion_call_back);
    glfwSetScrollCallback(g_mainWindow, scroll_callback);
    g_debug_draw.create();

    create_ui(g_mainWindow, nullptr);

    // load first test
    s_settings.m_test_index = b3_clamp(s_settings.m_test_index, 0, g_test_count - 1);
    s_test_selection = s_settings.m_test_index;
    s_test = g_test_entries[s_settings.m_test_index].create_fcn();

    glClearColor(0.4f, 0.4f, 0.4f, 1.0f);
    while (!glfwWindowShouldClose(g_mainWindow)) {

        glfwGetWindowSize(g_mainWindow, &g_camera.m_width, &g_camera.m_height);

        int bufferWidth, bufferHeight;
        glfwGetFramebufferSize(g_mainWindow, &bufferWidth, &bufferHeight);
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

        glfwSwapBuffers(g_mainWindow);
        glfwPollEvents();

    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();

}