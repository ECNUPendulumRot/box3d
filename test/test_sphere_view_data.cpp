

#include "spdlog/spdlog.h"
#include "igl/opengl/glfw/Viewer.h"

#include "box3d.hpp"

#include <iostream>

int main() {
  igl::opengl::glfw::Viewer viewer;

  b3World world;
  world.set_gravity(b3Vector3d(0, 0, 0));

  b3BodyDef body_def;
  body_def.m_type = b3BodyType::b3_dynamic_body;

  b3Body *body = world.create_body(body_def);

  b3SphereShape sphere_shape;
  sphere_shape.set_as_sphere(0.5);

  b3FixtureDef fixture_def;
  fixture_def.m_shape = &sphere_shape;
  fixture_def.m_density = 1.0;
  fixture_def.m_friction = 0.1;
  fixture_def.m_restitution = 1.0;

  body->create_fixture(fixture_def);

  b3ViewData *view_data = world.get_shape_list()->get_view_data();

  spdlog::info("sphere has {} vertices.", view_data->m_vertex_count);
  spdlog::info("sphere has {} faces.", view_data->m_face_count);

  E3MapMatrixX<double, Eigen::RowMajor> vertices(view_data->m_V, view_data->m_vertex_count, 3);
  E3MapMatrixX<int, Eigen::RowMajor> faces(view_data->m_F, view_data->m_face_count, 3);

  std::cout << vertices << std::endl;
  std::cout << faces << std::endl;

  viewer.data().set_mesh(vertices, faces);

  viewer.launch();
}