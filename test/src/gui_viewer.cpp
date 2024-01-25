
#include "../include/gui_viewer.hpp"
#include "utils/b3_log.hpp"

#include "scene_test.hpp"

namespace {

    void dpi_aware(int* width, int* height)
    {
        // Get the DPI of the primary monitor
        using namespace igl::opengl::glfw;
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        if (monitor == nullptr)
            return;
        float xscale, yscale;
        glfwGetMonitorContentScale(monitor, &xscale, &yscale);
        (*width)  *= xscale;
        (*height) *= yscale;
    }

    static const Eigen::IOFormat RowVectorFmt(Eigen::FullPrecision,
                                              Eigen::DontAlignCols,
                                              ", ",
                                              ",\n",
                                              "[",
                                              "]",
                                              "",
                                              "");

    static const Eigen::IOFormat MatrixFmt(Eigen::FullPrecision,
                                           Eigen::DontAlignCols,
                                           ", ",
                                           ",\n",
                                           "[",
                                           "]",
                                           "[",
                                           "]");

    std::string matrix_str(const Eigen::MatrixXd& x, const int precision)
    {
        std::stringstream ssx;
        Eigen::MatrixXd m = x;
        if (m.cols() == 1) {
            m.transposeInPlace();
        }
        ssx << std::setprecision(precision);
        if (m.rows() == 1) {
            ssx << m.format(RowVectorFmt);
        } else {
            ssx << m.format(MatrixFmt);
        }
        return ssx.str();
    }

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


static void sort_tests()
{
    std::sort(g_test_entries, g_test_entries + g_test_count, compare_tests);
}


b3GUIViewer::b3GUIViewer()
{
    // Set up the transformation matrix
    // 0 1 0
    // 0 0 1
    // 1 0 0
    m_transform.setIdentity();
    m_transform.col(0).swap(m_transform.col(2));
    m_transform.col(1).swap(m_transform.col(2));
    m_transform.transposeInPlace();

    sort_tests();

    // set up the camera
    m_viewer.core().viewport = Eigen::Vector4f(0, 0, 1280, 720);
    m_viewer.core().camera_eye = Eigen::Vector3f(4, 4, 4);
    m_viewer.core().camera_center = Eigen::Vector3f(-3, 0, 0);
    //m_viewer.core().camera_view_angle = 95;
}


void b3GUIViewer::launch()
{
    // m_viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION);
    // m_viewer.core().orthographic = true;
    m_viewer.core().is_animating = true;
    // m_viewer.core().lighting_factor = 0.0;
    m_viewer.core().animation_max_fps = 60.0;
    m_viewer.core().background_color << 0.6, 0.6, 0.6, 0.6;

    // lighting
    m_viewer.core().lighting_factor = 0.4;
    m_viewer.core().light_position = Eigen::Vector3f(50, 50, 50);

    // add GUI plugin
    m_viewer.plugins.push_back(&m_plugin);
    m_plugin.widgets.push_back(&m_menu);

    // set up the callback for pre_draw
    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&) {
        return pre_draw_loop();
    };

    // Draw the axis at the origin
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 0, 0), Eigen::RowVector3d(1, 0, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 1, 0), Eigen::RowVector3d(0, 1, 0));
    m_viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1), Eigen::RowVector3d(0, 0, 1));

    add_ground();

    m_viewer.launch(false, "Simulation");
}


void b3GUIViewer::add_ground() {
    int g_s = 10;

    Eigen::RowVector3d c = Eigen::RowVector3d(0, 0, 0);

    Eigen::RowVector3d v1;
    Eigen::RowVector3d v2;
    Eigen::RowVector3d v3;
    Eigen::RowVector3d v4;

    for (int i = 0; i <= 2 * g_s; i++) {
        v1 = Eigen::RowVector3d(-g_s, -1, i - g_s);
        v2 = Eigen::RowVector3d(g_s, -1, i - g_s);

        v3 = Eigen::RowVector3d(i - g_s, -1, -g_s);
        v4 = Eigen::RowVector3d(i - g_s, -1, g_s);

        m_viewer.data().add_edges(v1, v2, c);
        m_viewer.data().add_edges(v3, v4, c);
    }
}


bool b3GUIViewer::check_test_index() {
    if (m_menu.m_selected_test == -1)
        return false;

    if (m_menu.m_selected_test != -1 && m_menu.m_selected_test == m_current_test)
        return false;

    m_current_test = m_menu.m_selected_test;
    if (m_test != nullptr) {
        delete m_test;
    }
    m_test = g_test_entries[m_menu.m_selected_test].create_fcn();
    return true;
}

bool b3GUIViewer::pre_draw_loop()
{
    // make sure that two variables are initialized to -1
    if (check_test_index()) {
        clear_meshes();
        add_meshes();
    }
    if (m_test != nullptr)
        m_test->step();

    redraw_mesh();

    return false;
}


void b3GUIViewer::add_meshes() {

    m_shape_count = m_test->get_shape_count();
    m_shape_list = m_test->get_shape_list();

    b3_assert(m_shape_list != nullptr);
    b3_assert(m_shape_count != -1);

    b3Shape* shape = m_shape_list;
    for (int i = 0; i < m_shape_count; i++) {
        int id;
        if (i >= m_viewer_used_count) {
            id = m_viewer.append_mesh(true);
            m_viewer_used_count++;
        } else {
            id = i + 1;
            m_viewer.data(id).clear();
        }

        b3ViewData view_data;
        auto p = shape->get_body()->get_pose();
        view_data = shape->get_view_data(shape->get_body()->get_pose());

        E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data.m_V, view_data.m_vertex_count, 3);
        E3MapMatrixX<int, Eigen::RowMajor> faces(view_data.m_F, view_data.m_face_count, 3);

        vertices *= m_transform;

        std::string s = matrix_str(vertices, 3);
        std::cout << s <<std::endl;
        m_viewer.data(id).set_mesh(vertices, faces);
        shape = shape->next();
    }
}


void b3GUIViewer::clear_meshes() {
    for (int i = 1; i < m_viewer.data_list.size(); i++) {
        m_viewer.data(i).clear();
    }
}


void b3GUIViewer::redraw_mesh() {

    if(m_shape_list == nullptr) {
        return;
    }

    b3Shape* shape = m_shape_list;

    int index = 0;

    while (shape != nullptr) {
        b3ViewData view_data = shape->get_view_data(shape->get_body()->get_pose());
        E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data.m_V, view_data.m_vertex_count, 3);
        E3MapMatrixX<int, Eigen::RowMajor> faces(view_data.m_F, view_data.m_face_count, 3);
        vertices *= m_transform;
        m_viewer.data(index + 1).set_mesh(vertices, faces);

        index++;

        shape = shape->next();
    }

}





