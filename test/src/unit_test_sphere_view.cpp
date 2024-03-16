
#include "unit_test.hpp"

class UnitTestShpereView : public UnitTestBase {

  b3Transformr xf_A;

  b3Body *body_A = nullptr;

  b3SphereShape *sphere_A = nullptr;

public:

  UnitTestShpereView() {
	xf_A.set_linear(0, 0, 0);
	xf_A.set_angular(0, 0, 0);
	sphere_A = new b3SphereShape();
	sphere_A->set_as_sphere(2);
	body_A = new b3Body();
	body_A->set_pose(xf_A);
	sphere_A->set_relative_body(body_A);
	sphere_A->set_block_allocator(&m_allocator);
  }

  ~UnitTestShpereView() override {
	delete sphere_A;
	delete body_A;
  }

  void step() override {

  }

  int get_shape_count() const override {
	return 1;
  }

  b3Shape *get_shape_list() const override {
	return sphere_A;
  }

  static TestBase *create() {
	return new UnitTestShpereView();
  }

};

static int register_index = register_test("Unit Test", "Sphere View", UnitTestShpereView::create);