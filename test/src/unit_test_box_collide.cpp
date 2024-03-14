
#include "unit_test.hpp"

#include <spdlog/spdlog.h>

#define VERTEX(V, i) (*(V + 3 * i)), (*(V + 3 * i + 1)), (*(V + 3 * i + 2))

class UnitTestBoxCollide : public UnitTestBase {

  b3TransformD xf_A;

  b3TransformD xf_B;

  b3Body *body_A = nullptr;

  b3Body *body_B = nullptr;

  b3CubeShape *cube_A = nullptr;

  b3CubeShape *cube_B = nullptr;

  b3Vector3d hf_A = b3Vector3d(1, 1, 1);

  b3Vector3d hf_B = b3Vector3d(1, 1, 1);

  b3Manifold manifold;

  int selected_box = -1;
  b3Body *selected_body = nullptr;

public:

  UnitTestBoxCollide() {
	xf_A.set_linear(-1.7, 0.9, 0);
	xf_A.set_angular(0, 0.6, 1.1);
	cube_A = new b3CubeShape();
	cube_A->set_as_box(hf_A.m_x, hf_A.m_y, hf_A.m_z);
	body_A = new b3Body();
	body_A->set_pose(xf_A);
	cube_A->set_relative_body(body_A);
	cube_A->set_block_allocator(&m_allocator);


	xf_B.set_linear(b3Vector3d(0, 3, 0));
	xf_B.set_angular(0, 0, 0);
	cube_B = new b3CubeShape();
	cube_B->set_as_box(hf_B.m_x, hf_B.m_y, hf_B.m_z);
	body_B = new b3Body();
	body_B->set_pose(xf_B);
	cube_B->set_relative_body(body_B);
	cube_B->set_block_allocator(&m_allocator);

	cube_A->set_next(cube_B);
  }

  ~UnitTestBoxCollide() override {
	delete cube_A;
	delete cube_B;
	delete body_A;
	delete body_B;
  }

  void step() override {
	spdlog::log(spdlog::level::info, "step----------------------------------------");
	xf_A = body_A->get_pose();
	xf_B = body_B->get_pose();
	spdlog::log(spdlog::level::info, "body A position: {}, {}, {}", xf_A.linear().x(), xf_A.linear().y(), xf_A.linear().z());
	spdlog::log(spdlog::level::info, "body A rotation: {}, {}, {}", xf_A.angular().x(), xf_A.angular().y(), xf_A.angular().z());
	b3Manifold m;
	b3Timer timer;
	b3_collide_cube(&m, cube_A, xf_A, cube_B, xf_B);
	spdlog::log(spdlog::level::info, "time for checking collision: {}", timer.get_time_ms());
	spdlog::log(spdlog::level::info, "manifold point count: {}", m.m_point_count);

	switch (m.m_type) {
	case b3Manifold::e_circles:
	  spdlog::log(spdlog::level::info, "manifold type: circles");
	  break;
	case b3Manifold::e_face_A:
	  spdlog::log(spdlog::level::info, "manifold type: face A");
	  break;
	case b3Manifold::e_face_B:
	  spdlog::log(spdlog::level::info, "manifold type: face B");
	  break;
	case b3Manifold::e_edges:
	  spdlog::log(spdlog::level::info, "manifold type: edges");
	  break;
	}

	manifold = m;

	spdlog::log(spdlog::level::info, "step end------------------------------------");
  }

  int get_shape_count() const override {
	return 2;
  }

  b3Shape *get_shape_list() const override {
	return cube_A;
  }

  static TestBase *create() {
	return new UnitTestBoxCollide();
  }

  void selected_object(const int &index) override {
	if (index == 0) {
	  selected_body = body_A;
	} else if (index == 1) {
	  selected_body = body_B;
	}
  }

  bool key_pressed(Viewer &viewer, unsigned int key, int modifiers) override {
	if (selected_body == nullptr) {
	  return false;
	}
	// Ensure the key is in upper case
	if (key >= 97 && key <= 122) key -= 32;

	b3TransformD xf = selected_body->get_pose();
	switch (key) {
	case GLFW_KEY_W:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(-0.1, 0, 0));
	  break;
	}
	case GLFW_KEY_S:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(0.1, 0, 0));
	  break;
	}
	case GLFW_KEY_A:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(0, -0.1, 0));
	  break;
	}
	case GLFW_KEY_D:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(0, 0.1, 0));
	  break;
	}
	case GLFW_KEY_T:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(0, 0, 0.1));
	  break;
	}
	case GLFW_KEY_G:
	{
	  xf.set_linear(xf.linear() + b3Vector3d(0, 0, -0.1));
	  break;
	}
	case GLFW_KEY_Q:
	{
	  xf.set_angular(xf.angular() + b3Vector3d(0, 0, 0.1));
	  break;
	}
	case GLFW_KEY_E:
	{
	  xf.set_angular(xf.angular() + b3Vector3d(0, 0, -0.1));
	  break;
	}
	case GLFW_KEY_R:
	{
	  xf.set_angular(xf.angular() + b3Vector3d(0, 0.1, 0));
	  break;
	}
	case GLFW_KEY_F:
	{
	  xf.set_angular(xf.angular() + b3Vector3d(0, -0.1, 0));
	  break;
	}

	default:
	  return false;
	}
	selected_body->set_pose(xf);
	return true;
  }

  int get_auxiliary_shape_count() const override {
	return 2;
  }

  b3AuxiliaryShape *get_auxiliary_shape_list() const override {
	auto *edge_box_A = new b3AuxiliaryShape();
	auto *edge_box_B = new b3AuxiliaryShape();
	edge_box_A->set_next(edge_box_B);

	b3ViewData view_data_A = cube_A->get_view_data(body_A->get_pose());
	double *V = view_data_A.m_V;
	edge_box_A->add_line(VERTEX(V, 0), VERTEX(V, 1));
	edge_box_A->add_line(VERTEX(V, 1), VERTEX(V, 2));
	edge_box_A->add_line(VERTEX(V, 2), VERTEX(V, 3));
	edge_box_A->add_line(VERTEX(V, 3), VERTEX(V, 0));
	edge_box_A->add_line(VERTEX(V, 4), VERTEX(V, 5));
	edge_box_A->add_line(VERTEX(V, 5), VERTEX(V, 6));
	edge_box_A->add_line(VERTEX(V, 6), VERTEX(V, 7));
	edge_box_A->add_line(VERTEX(V, 7), VERTEX(V, 4));
	edge_box_A->add_line(VERTEX(V, 0), VERTEX(V, 4));
	edge_box_A->add_line(VERTEX(V, 1), VERTEX(V, 5));
	edge_box_A->add_line(VERTEX(V, 2), VERTEX(V, 6));
	edge_box_A->add_line(VERTEX(V, 3), VERTEX(V, 7));

	b3ViewData view_data_B = cube_B->get_view_data(body_B->get_pose());
	V = view_data_B.m_V;
	edge_box_B->add_line(VERTEX(V, 0), VERTEX(V, 1));
	edge_box_B->add_line(VERTEX(V, 1), VERTEX(V, 2));
	edge_box_B->add_line(VERTEX(V, 2), VERTEX(V, 3));
	edge_box_B->add_line(VERTEX(V, 3), VERTEX(V, 0));
	edge_box_B->add_line(VERTEX(V, 4), VERTEX(V, 5));
	edge_box_B->add_line(VERTEX(V, 5), VERTEX(V, 6));
	edge_box_B->add_line(VERTEX(V, 6), VERTEX(V, 7));
	edge_box_B->add_line(VERTEX(V, 7), VERTEX(V, 4));
	edge_box_B->add_line(VERTEX(V, 0), VERTEX(V, 4));
	edge_box_B->add_line(VERTEX(V, 1), VERTEX(V, 5));
	edge_box_B->add_line(VERTEX(V, 2), VERTEX(V, 6));
	edge_box_B->add_line(VERTEX(V, 3), VERTEX(V, 7));

	for (int i = 0; i < manifold.m_point_count; i++) {
	  const b3ManifoldPoint *mp = manifold.m_points + i;
	  edge_box_A->add_point(mp->m_local_point.x(), mp->m_local_point.y(), mp->m_local_point.z());
	}

	return edge_box_A;
  }

};

int register_index = register_test("unit test", "box collide", UnitTestBoxCollide::create);