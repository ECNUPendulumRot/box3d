
#include "test.hpp"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>

class TestConeData : public Test {

    std::filesystem::path fileName = "./coneData.csv";
    std::ifstream file;

    b3ConeShape coneShape;

    b3Body* body = nullptr;

public:

    ~TestConeData() {
        file.close();
    }

    TestConeData() {
        file.open(fileName, std::ios::in);
        if (!file.is_open()) {
            std::cout << "file open failed" << std::endl;
        } else {
            std::cout << "file open success" << std::endl;
        }
        coneShape.set_as_cone(2, 5);
        b3BodyDef bodyDef;
        bodyDef.m_type = b3BodyType::b3_dynamic_body;

        body = m_world->create_body(bodyDef);
        b3FixtureDef fd;
        fd.m_shape = &coneShape;
        fd.m_density = 1.0;
        body->create_fixture(fd);
    }

    virtual void step(Settings& settings) {
        b3Vec3r p;
        b3Quaternionr q;
        std::string lineStr;
        std::getline(file, lineStr);
        std::stringstream ss(lineStr);
        std::string item;
        std::vector<real> values;
        while (std::getline(ss, item, ',')) {
            values.push_back(std::stof(item));
        }
        p.set(values[0], values[1], values[2]);
        q.set(values[6], values[3], values[4], values[5]);
        body->set_position(p);
        body->set_quaternion(q);
        Test::step(settings);
    }

    static Test* create() {
        return new TestConeData;
    }
};


static int test_index = register_test("Cone", "cone data", TestConeData::create);