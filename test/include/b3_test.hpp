
#ifndef BOX3D_B3_TEST_HPP
#define BOX3D_B3_TEST_HPP

#include "box3d.hpp"

class TestBase;


using TestCreateFcn = TestBase*();

struct TestEntry
{
    const char* name;
    TestCreateFcn* creatFcn;
};


int register_test(const char* category, const char* name, TestCreateFcn* fcn);

class TestBase {

    static std::filesystem::path s_body_def_dir;

    static std::filesystem::path s_mesh_dir;

    static std::filesystem::path s_scene_dir;

    box3d::b3World* m_world;

public:

    TestBase();

    // TODO: check the destructor.
    ~TestBase() = default;

    virtual void simulation_step();

    box3d::b3World* get_world() {
        return m_world;
    }

    static void parse_scene(TestBase* test, const std::string &scene_str);

    void create_object(const nlohmann::json &object);

};

#endif //BOX3D_B3_TEST_HPP
