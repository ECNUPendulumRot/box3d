
#include "igl/opengl/glfw/Viewer.h"

#include "box3d.hpp"

#include "common/b3_types.hpp"

#include <iostream>

#include "utils/b3_timer.hpp"

b3BlockAllocator block_allocator;

igl::opengl::glfw::Viewer viewer;

b3TransformD xf[2];

b3SphereShape sphere[2];
double radius[2] = { 0.5, 0.5 };
b3Vector3d position[2] = {
        b3Vector3d(1, 1, 1),
        b3Vector3d(0, 0, 0)
};

b3Body body[2];

b3Manifold manifold;

E3Vector3d colors[3] = {
        E3Vector3d(1, 0, 0),
        E3Vector3d(0, 1, 0),
        E3Vector3d(0, 0, 1)
};

E3MatrixXd point;

int select_object_index = -1;

b3Timer timer;
int last_mouse_x = 0;
int last_mouse_y = 0;


bool same_click_position(int x, int y) {
    if(b3_abs(x - last_mouse_x) < 5 && b3_abs(y - last_mouse_y) < 5) {
        return true;
    }
    last_mouse_x = x;
    last_mouse_y = y;
    return false;
}

bool line_AABB_intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                       const b3Vector3d& min_corner, const b3Vector3d& max_corner) {

    double t_min = 0.0f;
    double t_max = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i) {
        if (b3_abs(dir[i]) < std::numeric_limits<double>::epsilon()) {
            // Ray is parallel to slab. If it is not within the slab, no intersection.
            if (origin[i] < min_corner[i] || origin[i] > max_corner[i]) {
                return false;
            }
        } else {
            // Compute intersection t value of the slab
            double t1 = (min_corner[i] - origin[i]) / dir[i];
            double t2 = (max_corner[i] - origin[i]) / dir[i];

            if (t1 > t2) {
                std::swap(t1, t2);
            }

            t_min = b3_max(t1, t_min);
            t_max = b3_min(t2, t_max);

            if (t_min > t_max) {
                return false;
            }
        }
    }

    // The line intersects the AABB if the final interval is not empty
    return true;
}


void interact_object(Eigen::Vector3d& origin, Eigen::Vector3d& point) {
    Eigen::Vector3d direction = (point - origin).normalized();

    b3AABB aabb;
    for(int i = 0; i < 2; ++i) {
        sphere[i].get_bound_aabb(&aabb, xf[i], sphere[i].get_child_count());
        const b3Vector3d min_a = aabb.min();
        const b3Vector3d max_a = aabb.max();
        if(line_AABB_intersect(origin, direction, min_a, max_a)) {
            select_object_index = i;
        }
    }
}


bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {

    // double click
    if(same_click_position(viewer.current_mouse_x, viewer.current_mouse_y)) {
        if(timer.get_time_ms() > 200) {
            return false;
        }
        timer.reset();
    } else {
        timer.reset();
        return false;
    }
    // init select object
    select_object_index = -1;

    // second click - first click time < 200ms
    float x = viewer.current_mouse_x;
    float y = viewer.current_mouse_y;
    float z = 0;

    // 获取鼠标点击位置的深度值
    glReadPixels((int)x, (int)(viewer.core().viewport[3] - y), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

    // ndc
    x = (2.0 * x) / viewer.core().viewport[2] - 1;
    y = 1.0 - (2.0 * y) / viewer.core().viewport[3];
    z = 2.0 * z - 1.0;
    Eigen::Matrix4f proj_inv = viewer.core().proj.inverse();
    Eigen::Matrix4f view = viewer.core().view;

    Eigen::Matrix3f view_r = view.block<3, 3>(0, 0).inverse();
    Eigen::Vector3f view_t = view.block<3, 1>(0, 3);

    // camera frame.
    Eigen::Vector4f ndc_point(x, y, z, 1.0f);
    Eigen::Vector4f proj_point = proj_inv * ndc_point;

    proj_point = proj_point / proj_point[3];

    Eigen::Vector3f world_point(proj_point.x(), proj_point.y(), proj_point.z());
    world_point = view_r * (world_point - view_t);

    Eigen::Vector3d point_b(world_point.x(), world_point.y(), world_point.z());
    Eigen::Vector3d point_a(viewer.core().camera_eye.x(),
                            viewer.core().camera_eye.y(),
                            viewer.core().camera_eye.z());

    interact_object(point_a, point_b);

    std::cout << "double click end, select object index = " << select_object_index << std::endl;

    return false; // 防止继续传播事件
}


void init() {

    for(int i = 0; i < 2; ++i) {
        sphere[i].set_block_allocator(&block_allocator);
        sphere[i].set_as_sphere(radius[i]);
        sphere[i].set_relative_body(&body[i]);
        xf[i].set_linear(position[i]);
        body[i].set_pose(xf[i]);
    }

    viewer.core().camera_eye = Eigen::Vector3f(0, 0, 5.0f);
    viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 0, 0), Eigen::RowVector3d(1, 0, 0));
    viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 1, 0), Eigen::RowVector3d(0, 1, 0));
    viewer.data().add_edges(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 5), Eigen::RowVector3d(0, 0, 1));
}

int igl_index[2] = { -1, -1 };
void add_mesh(int color_index, b3SphereShape& sphere) {
    b3ViewData view_data = sphere.get_view_data(body[color_index].get_pose());
    E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data.m_V, view_data.m_vertex_count, 3);
    E3MapMatrixX<int, Eigen::RowMajor> faces(view_data.m_F, view_data.m_face_count, 3);

    if(igl_index[color_index] == -1) {
        igl_index[color_index] = viewer.append_mesh(true);
    }

    viewer.data(igl_index[color_index]).set_mesh(vertices, faces);
    Eigen::MatrixXd color(1, 4);
    color << colors[color_index].x(), colors[color_index].y(), colors[color_index].z(), 0.2;
    viewer.data(igl_index[color_index]).set_colors(color);
    viewer.data(igl_index[color_index]).show_lines = false;
}



bool callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
    if(select_object_index >= 0) {
        b3TransformD pose = body[select_object_index].get_pose();
        b3Vector3d position = pose.linear();
        // b3Timer key_down_timer;
        // key_down_timer.reset();
        if(key == GLFW_KEY_W) {
            // x +
            position.m_x += 0.1;
        } else if(key == GLFW_KEY_S) {
            position.m_x -= 0.1;
        } else if(key == GLFW_KEY_A) {
            position.m_y += 0.1;
        } else if(key == GLFW_KEY_D) {
            position.m_y -= 0.1;
        } else if(key == GLFW_KEY_R) {
            position.m_z += 0.1;
        } else if(key == GLFW_KEY_F) {
            position.m_z -= 0.1;
        }
        pose.set_linear(position);
        body[select_object_index].set_pose(pose);
    }
    return false;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer) {
    for(int i = 0; i < 2; ++i) {
        add_mesh(i, sphere[i]);
    }

    const b3TransformD xf_a = body[0].get_pose();
    const b3TransformD xf_b = body[1].get_pose();
    b3_collide_spheres(&manifold, &sphere[0], xf_a, &sphere[1], xf_b);

    if(manifold.m_point_count > 0) {
        b3Vector3d contact_point = manifold.m_points[0].m_local_point;
        b3Vector3d contact_normal = manifold.m_local_normal;
        double penetration = manifold.m_penetration;

        Eigen::RowVector3d point(contact_point.x(), contact_point.y(), contact_point.z());
        Eigen::RowVector3d normal(contact_normal.x(), contact_normal.y(), contact_normal.z());
        Eigen::RowVector3d point_color(1, 1, 1);
        viewer.data().add_points(point, point_color);
        viewer.data().add_edges(point, point + 2 * normal, point_color);
        viewer.data().add_label(point + 2 * normal, "penetration = " + std::to_string(penetration));
    }

    return false;
}

int main() {

    init();

    add_mesh(0, sphere[0]);
    add_mesh(1, sphere[1]);

    viewer.callback_mouse_down = &callback_mouse_down;
    viewer.callback_key_down = &callback_key_down;

    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 60.0;


    viewer.callback_pre_draw = &pre_draw;


    viewer.launch();

}