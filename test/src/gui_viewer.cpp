
#include "gui/gui_viewer.hpp"

#include <spdlog/spdlog.h>

#include "scene_test.hpp"
#include "unit_test.hpp"

template <typename T, int Major>
using MapMatrixX = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Major>>;


namespace {

    void dpi_aware(int *width, int *height)
    {
        // Get the DPI of the primary monitor
        using namespace igl::opengl::glfw;
        GLFWmonitor *monitor = glfwGetPrimaryMonitor();

        if (monitor == nullptr)
          return;
        float xscale, yscale;
        glfwGetMonitorContentScale(monitor, &xscale, &yscale);
        (*width) *= xscale;
        (*height) *= yscale;
    }

    static const Eigen::IOFormat RowVectorFmt(Eigen::FullPrecision, Eigen::DontAlignCols,
											    ", ", ",\n",
											    "[", "]", "", "");

     static const Eigen::IOFormat MatrixFmt(Eigen::FullPrecision, Eigen::DontAlignCols,
										    ", ", ",\n",
										    "[", "]", "[", "]");

    std::string matrix_str(const Eigen::MatrixXd &x, const int precision)
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

    bool same_position(const int &mouse_x, const int &mouse_y)
    {
        static int last_x = -1;
        static int last_y = -1;

        if (b3_abs(mouse_x - last_x) < 5 && b3_abs(mouse_y - last_y) < 5) {
            return true;
        }
        last_x = mouse_x;
        last_y = mouse_y;
        return false;
    }

    bool double_clicked(const int &mouse_x, const int &mouse_y, b3Timer &timer)
    {
        if (!same_position(mouse_x, mouse_y)) {
              timer.reset();
              return false;
        }

        bool is_double_clicking = timer.get_time_ms() < 200;
        timer.reset();
        return is_double_clicking;
    }

}


static inline bool compare_tests(const TestEntry &a, const TestEntry &b)
{
    int result = strcmp(a.category, b.category);
        if (result == 0) {
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

    m_viewer.core().depth_test = m_gui_test_menu.m_enable_depth_test;

    // add GUI plugin
    m_viewer.plugins.push_back(&m_gui_plugin);
    m_gui_plugin.widgets.push_back(&m_gui_title_bar);
    m_gui_plugin.widgets.push_back(&m_gui_test_menu);
    m_gui_plugin.widgets.push_back(&m_gui_mesh_menu);

      //////////////////////////// Set up Callbacks ////////////////////////////
    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) {
  	    return pre_draw_loop();
  	};

    m_viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers) {

	    if (m_test != nullptr) {
	        return m_test->key_pressed(viewer, key, modifiers);
	    }
        return false;
	};

    // Draw the axis at the origin

    m_viewer.data().add_edges(Eigen::Matrix3d::Identity() * m_transform, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity());

    m_viewer.data().line_width = 1.25f;

    m_viewer.launch(false, "Simulation");
}


// check whether there is a test chosen
// or whether the test is changed
// if there is a change, delete the old test and return true
// if there is no test or not changed, return false
bool b3GUIViewer::check_test_index()
{
    if (m_gui_test_menu.m_selected_test == -1)
        return false;

    if (m_gui_test_menu.m_selected_test != -1 && m_gui_test_menu.m_selected_test == m_current_test)
  	    return false;

    m_current_test = m_gui_test_menu.m_selected_test;

    if (m_test != nullptr) {
  	    delete m_test;
    }
    m_test = g_test_entries[m_gui_test_menu.m_selected_test].create_fcn();
    return true;
}


bool b3GUIViewer::pre_draw_loop()
{
    m_viewer.core().depth_test = m_gui_test_menu.m_enable_depth_test;

    // make sure that two variables are initialized to -1
    if (check_test_index()) {
        clear_viewer_data();
        add_meshes();
        m_gui_mesh_menu.set_test(m_test);
    }

    if (m_test == nullptr)
        return false;

    m_test->step();

    // reallocate the mesh index
    m_viewer_used_count = 0;

    draw_mesh();

    draw_auxiliary_shapes();

    if (m_gui_test_menu.m_show_local_axis)
        draw_body_axis();

    return false;
}


// Check whether the data list in igl is enough to keep new mesh
int b3GUIViewer::allocate_mesh_index()
{
    // the first mesh is used to draw the ground
    m_viewer_used_count++;
    int viewer_allocated_mesh = m_viewer.data_list.size() - 1;
    if (m_viewer_used_count > viewer_allocated_mesh)
  	    m_viewer.append_mesh(true);
    return m_viewer_used_count;
}


// add all address of shapes in the test into m_shapes
void b3GUIViewer::add_meshes()
{

    int shape_count = m_test->get_shape_count();
    b3Shape *shape_list = m_test->get_shape_list();

    b3_assert(shape_list != nullptr);
    b3_assert(shape_count != -1);

    b3Shape *shape = shape_list;

    for (int i = 0; i < shape_count; i++) {
        m_shapes.push_back(shape);
        m_gui_mesh_menu.add_object(i);
        shape = shape->next();
    }
}


// clear all view data in m_viewer
// also clear the mesh list and the shape list
void b3GUIViewer::clear_viewer_data()
{
    m_viewer_used_count = 0;
    for (int i = 1; i < m_viewer.data_list.size(); i++) {
  	    m_viewer.data(i).clear();
    }
    m_gui_mesh_menu.clear();
    m_shapes.clear();
}


void b3GUIViewer::draw_mesh()
{
    for (int index = 0; index < m_shapes.size(); index++) {
        int shape_index = allocate_mesh_index();

        ViewerData &data = m_viewer.data(shape_index);
        b3Shape *shape = m_shapes[index];

        // TODO: test affine
        // b3Transformr xf(shape->get_body()->get_position(), shape->get_body()->get_quaternion());
        b3Transformr xf(shape->get_body()->get_affine_q());
        b3ViewData view_data = shape->get_view_data(xf);
        MapMatrixX<double, Eigen::RowMajor> vertices(view_data.m_V, view_data.m_vertex_count, 3);

        vertices *= m_transform;


  	    data.line_color = Eigen::Vector4f(m_gui_test_menu.m_line_color.x, m_gui_test_menu.m_line_color.y, m_gui_test_menu.m_line_color.z, 1.0);

        ImVec4 &color = m_gui_mesh_menu.m_index_colors[index].color;
        Eigen::RowVector3d c;
        b3Vector3r shape_color = shape->get_color();

        if (shape_color.is_zero()) {
            c << color.x, color.y, color.z;
        } else {
            c << shape_color.x(), shape_color.y(), shape_color.z();
        }

        if (view_data.m_edge_count > 0) {
            MapMatrixX<int, Eigen::RowMajor> edges(view_data.m_E, view_data.m_edge_count, 2);
            data.set_edges(vertices, edges, c);
        }

        if (view_data.m_face_count > 0) {
            MapMatrixX<int, Eigen::RowMajor> faces(view_data.m_F, view_data.m_face_count, 3);
            data.set_mesh(vertices, faces);
            data.set_colors(Eigen::RowVector4d(c.x(), c.y(), c.z(), color.w));
        }

        data.line_width = m_gui_test_menu.m_line_width;

        data.show_faces = m_gui_test_menu.m_show_faces;
        data.show_lines = m_gui_test_menu.m_show_edges;
    }
}


void b3GUIViewer::draw_auxiliary_shapes()
{
    int auxiliary_index = allocate_mesh_index();

    m_viewer.data(auxiliary_index).clear_edges();
    m_viewer.data(auxiliary_index).clear_points();

    if (!m_gui_test_menu.m_show_auxiliary_shapes) {
  	    return;
    }

    if (m_test == nullptr)
  	    return;

    int auxiliary_shape_count = m_test->get_auxiliary_shape_count();
    b3AuxiliaryShape *auxiliary_shape_list = m_test->get_auxiliary_shape_list();

    if (auxiliary_shape_count == 0) {
  	    return;
    }

    b3_assert(auxiliary_shape_count != -1);
    b3_assert(auxiliary_shape_list != nullptr);

    b3AuxiliaryShape *auxiliary_shape = auxiliary_shape_list;

    if (auxiliary_shape->has_points()) {
        m_viewer.data(auxiliary_index).point_size = 8;
        Eigen::MatrixXd points = auxiliary_shape->get_points_m();
        points *= m_transform;
        m_viewer.data(auxiliary_index).add_points(points, Eigen::RowVector3d(1.0, 0.0, 0.0));
    }

    for (int i = 0; i < auxiliary_shape_count; i++) {
        Eigen::MatrixXd edges_left = auxiliary_shape->get_edges_left_m();
        Eigen::MatrixXd edges_right = auxiliary_shape->get_edges_right_m();
        Eigen::RowVector3d color = auxiliary_shape->get_color();
        edges_left *= m_transform;
        edges_right *= m_transform;
        m_viewer.data(auxiliary_index).add_edges(edges_left, edges_right, color);

        auxiliary_shape = auxiliary_shape->next();
    }
}


void b3GUIViewer::draw_body_axis() {

    int axis_index = allocate_mesh_index();

    m_viewer.data(axis_index).clear_edges();

    b3AuxiliaryShape* local_axis = m_test->get_local_axis();

    if (local_axis == nullptr)
        return;

    Eigen::MatrixXd edges_left = local_axis->get_edges_left_m();
    Eigen::MatrixXd edges_right = local_axis->get_edges_right_m();
    Eigen::MatrixXd color = local_axis->get_color_m();
    edges_left *= m_transform;
    edges_right *= m_transform;
    m_viewer.data(axis_index).add_edges(edges_left, edges_right, color);
}














