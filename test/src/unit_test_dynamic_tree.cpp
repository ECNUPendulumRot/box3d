
#include "unit_test.hpp"

#include <vector>

static double radius[5] = {
		0.1,
		0.2,
		0.3,
		0.4,
		0.5
};

static b3Vector3d hf_cube[5] = {
		b3Vector3d(0.1, 0.1, 0.1),
		b3Vector3d(0.1, 0.2, 0.2),
		b3Vector3d(0.3, 0.1, 0.1),
		b3Vector3d(0.2, 0.1, 0.3),
		b3Vector3d(0.4, 0.3, 0.3)
};


static b3Vector3d position[10] = {
		b3Vector3d(1, 0, 0),
		b3Vector3d(1, 2, 0),
		b3Vector3d(1, 1 ,1),
		b3Vector3d(2, 3, 1),
		b3Vector3d(2, 2, 2),
		b3Vector3d(3, 3, 3),
		b3Vector3d(2, 4, 2),
		b3Vector3d(2, 4, 0),
		b3Vector3d(2, 2, 4),
		b3Vector3d(5, 1, 3)
};


static Eigen::RowVector3d colors[10] = {
		Eigen::RowVector3d(0, 0, 0),
		Eigen::RowVector3d(252.0 / 255, 230.0 / 255, 202.0 / 255),
		Eigen::RowVector3d(1, 0, 0),
		Eigen::RowVector3d(156.0 / 255, 102.0 / 255, 31.0 / 255),
		Eigen::RowVector3d(0, 1, 0),
		Eigen::RowVector3d(0, 1, 1),
		Eigen::RowVector3d(0, 0, 1),
		Eigen::RowVector3d(128.0 / 255, 42.0 / 255, 42.0 / 255),
		Eigen::RowVector3d(160.0 / 255, 82.0 / 255, 45.0 / 255),
		Eigen::RowVector3d(153.0 / 255, 51.0 / 255, 250.0 / 255)
};

class UnitTestDynamicTree : public UnitTestBase {

  const int shape_count = 20;

  b3SphereShape *sphere_shape = nullptr;

  b3CubeShape *cube_shape = nullptr;

  b3Body *bodies = nullptr;

  b3BroadPhase broad_phase;

  std::vector<int32> tree_id_list;

  b3AuxiliaryShape *auxiliary_shape_list;
  int auxiliary_shape_count;

public:

  UnitTestDynamicTree();
  ~UnitTestDynamicTree();

  void step() override {

  }

  int get_shape_count() const override {
	return shape_count;
  }

  b3Shape *get_shape_list() const override {
	return sphere_shape;
  }

  static TestBase *create() {
	return new UnitTestDynamicTree();
  }

  int get_auxiliary_shape_count() const override {
	return auxiliary_shape_count;
  }

  b3AuxiliaryShape *get_auxiliary_shape_list() const override {
	return auxiliary_shape_list;
  }
};

int dynamic_tree_index = register_test("unit test", "dynamic tree", UnitTestDynamicTree::create);


void init_auxiliary_shape(b3AABB *aabb, int height, b3AuxiliaryShape *auxiliary_shape) {
  auxiliary_shape->set_color(colors[height]);
  b3Vector3d min = aabb->min();
  b3Vector3d max = aabb->max();
  auxiliary_shape->add_line(min.x(), min.y(), min.z(), max.x(), min.y(), min.z());
  auxiliary_shape->add_line(max.x(), min.y(), min.z(), max.x(), max.y(), min.z());
  auxiliary_shape->add_line(max.x(), max.y(), min.z(), min.x(), max.y(), min.z());
  auxiliary_shape->add_line(min.x(), max.y(), min.z(), min.x(), min.y(), min.z());
  auxiliary_shape->add_line(min.x(), min.y(), max.z(), max.x(), min.y(), max.z());
  auxiliary_shape->add_line(max.x(), min.y(), max.z(), max.x(), max.y(), max.z());
  auxiliary_shape->add_line(max.x(), max.y(), max.z(), min.x(), max.y(), max.z());
  auxiliary_shape->add_line(min.x(), max.y(), max.z(), min.x(), min.y(), max.z());
  auxiliary_shape->add_line(min.x(), min.y(), min.z(), min.x(), min.y(), max.z());
  auxiliary_shape->add_line(max.x(), min.y(), min.z(), max.x(), min.y(), max.z());
  auxiliary_shape->add_line(max.x(), max.y(), min.z(), max.x(), max.y(), max.z());
  auxiliary_shape->add_line(min.x(), max.y(), min.z(), min.x(), max.y(), max.z());
}


UnitTestDynamicTree::UnitTestDynamicTree() {

  ////////////////////// shape //////////////
  bodies = new b3Body[shape_count];

  b3TransformD xf;

  int body_index = 0;
  int shape_num = shape_count / 2;

  // first shape_num == 1
  sphere_shape = new b3SphereShape[shape_num];
  for (int i = 1; i < shape_num; ++i) {
	sphere_shape[i - 1].set_next(sphere_shape + i);
  }

  for (int i = 0; i < shape_num; ++i) {
	// set xf
	xf.set_linear(position[i % 10].x(), position[i % 10].y(), position[i % 10].z());
	xf.set_angular(0, 0, 0);
	sphere_shape[i].set_as_sphere(radius[i % 5]);
	sphere_shape[i].set_block_allocator(&m_allocator);
	bodies[body_index].set_pose(xf);
	sphere_shape[i].set_relative_body(bodies + body_index);
	body_index++;
  }

  cube_shape = new b3CubeShape[shape_num];
  sphere_shape[shape_num - 1].set_next(cube_shape);
  for (int i = 1; i < shape_num; ++i) {
	cube_shape[i - 1].set_next(cube_shape + i);
  }

  for (int i = 0; i < shape_num; ++i) {
	xf.set_linear(-position[i % 10].x(), -position[i % 10].y(), -position[i % 10].z());
	xf.set_angular(0, 0, 0);
	int index = i % 5;
	cube_shape[i].set_as_box(hf_cube[index].x(), hf_cube[index].y(), hf_cube[index].z());
	cube_shape[i].set_block_allocator(&m_allocator);
	bodies[body_index].set_pose(xf);
	cube_shape[i].set_relative_body(bodies + body_index);
	body_index++;
  }

  ///////////////// dynamic tree //////////////////////
  broad_phase.set_block_allocator(&m_allocator);
  b3AABB aabb;
  for (int i = 0; i < shape_num; ++i) {
	sphere_shape[i].get_bound_aabb(&aabb, bodies[i].get_pose(), 0);
	tree_id_list.push_back(broad_phase.create_proxy(aabb, nullptr));
  }
  for (int i = 0; i < shape_num; ++i) {
	cube_shape[i].get_bound_aabb(&aabb, bodies[i + shape_num].get_pose(), 0);
	tree_id_list.push_back(broad_phase.create_proxy(aabb, nullptr));
  }

  /////////////// generate auxiliary shape ///////////////////
  auxiliary_shape_count = broad_phase.get_dynamic_tree()->get_node_count();
  int n = broad_phase.get_dynamic_tree()->get_node_capacity();
  int height;

  auxiliary_shape_list = new b3AuxiliaryShape[auxiliary_shape_count];
  for (int i = 1; i < auxiliary_shape_count; ++i) {
	auxiliary_shape_list[i - 1].set_next(auxiliary_shape_list + i);
  }
  int index = 0;
  for (int i = 0; i < n; ++i) {
	broad_phase.get_dynamic_tree()->get_node_info(i, height, aabb);
	if (height != b3_NULL_HEIGHT) {
	  init_auxiliary_shape(&aabb, height, auxiliary_shape_list + index);
	  index++;
	}
  }
}


UnitTestDynamicTree::~UnitTestDynamicTree() {
  delete[] sphere_shape;
  delete[] cube_shape;
  delete[] bodies;
}