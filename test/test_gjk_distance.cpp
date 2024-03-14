#include "box3d.hpp"

#include <fstream>
#include <typeinfo>
#include <filesystem>


int main(int argc, char *argv[]) {

  namespace fs = std::filesystem;

  set_logger_level(spdlog::level::info);

  std::string cube_mesh_path("cube.obj");
  std::string sphere_mesh_path("sphere.obj");

  box3d::b3Mesh *cube_mesh_a = nullptr;
  box3d::b3Mesh *cube_mesh_b = nullptr;
  box3d::b3Mesh *sphere_mesh_a = nullptr;
  box3d::b3Mesh *sphere_mesh_b = nullptr;

  box3d::b3World *world = new box3d::b3World();

  {
	fs::path path(cube_mesh_path);
	path = fs::path(B3D_MESH_DIR) / path;

	spdlog::log(spdlog::level::info, "path: {}", path.string());

	if (!fs::exists(path)) {
	  spdlog::warn("The path do not exist");
	  return 0;
	}
	world->create_mesh(path);
	cube_mesh_a = world->create_mesh(path);
	cube_mesh_b = world->create_mesh(path);
  }

  {
	fs::path path(sphere_mesh_path);
	path = fs::path(B3D_MESH_DIR) / path;

	spdlog::log(spdlog::level::info, "path: {}", path.string());

	if (!fs::exists(path)) {
	  spdlog::warn("The path do not exist");
	  return 0;
	}

	sphere_mesh_a = world->create_mesh(path);
	sphere_mesh_b = world->create_mesh(path);
  }


  {
	// cube and cube 
	spdlog::info("test gjk between cube and cube");

	box3d::b3Fixture cube_fixture_a;
	box3d::b3Fixture cube_fixture_b;

	cube_fixture_a.set_mesh(cube_mesh_a);
	cube_fixture_b.set_mesh(cube_mesh_b);

	// set two object position
	box3d::b3Pose<double> cube_rel_pose_a;
	box3d::b3Pose<double> cube_rel_pose_b;

	cube_rel_pose_a.set_linear({ 1.8, 0, 0 });


	cube_mesh_a->set_relative_pose(&cube_rel_pose_a);
	cube_mesh_b->set_relative_pose(&cube_rel_pose_b);

	double distance = box3d::get_gjk_distance(&cube_fixture_a, &cube_fixture_b);

	spdlog::info("the distance between cube and cube is {}", distance);
  }

  {
	spdlog::info("test gjk between cube and sphere");
	// cube and sphere
	box3d::b3Fixture cube_fixture;
	box3d::b3Fixture sphere_fixture;

	cube_fixture.set_mesh(cube_mesh_a);
	sphere_fixture.set_mesh(sphere_mesh_a);

	// set two object position
	box3d::b3Pose<double> cube_rel_pose;
	box3d::b3Pose<double> sphere_rel_pose;

	cube_rel_pose.set_linear({ 2.0, 0, 0 });

	cube_mesh_a->set_relative_pose(&cube_rel_pose);
	sphere_mesh_a->set_relative_pose(&sphere_rel_pose);

	double distance = box3d::get_gjk_distance(&cube_fixture, &sphere_fixture);

	spdlog::info("the distance between cube and sphere is {}", distance);
  }

  {
	// sphere and sphere
	spdlog::info("test gjk between sphere and sphere");

	box3d::b3Fixture sphere_fixture_a;
	box3d::b3Fixture sphere_fixture_b;

	sphere_fixture_a.set_mesh(sphere_mesh_a);
	sphere_fixture_b.set_mesh(sphere_mesh_b);

	// set two object position
	box3d::b3Pose<double> sphere_rel_pose_a;
	box3d::b3Pose<double> sphere_rel_pose_b;

	sphere_rel_pose_a.set_linear({ 5.0, 0, 0 });

	sphere_mesh_a->set_relative_pose(&sphere_rel_pose_a);
	sphere_mesh_b->set_relative_pose(&sphere_rel_pose_b);

	double distance = box3d::get_gjk_distance(&sphere_fixture_a, &sphere_fixture_b);

	spdlog::info("the distance between sphere and sphere is {}", distance);
  }


  return 0;
}