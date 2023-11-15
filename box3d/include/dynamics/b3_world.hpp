
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include  <filesystem>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"


namespace box3d {

    class b3World;

}


class box3d::b3World {

    b3Body* m_body_list;

    int32 m_body_count;

    b3Vector3d m_gravity = b3Vector3d(0, 0, 9.8);

    double m_hz = 60;

public:

    b3World();

    b3Body* create_body(const b3BodyDef& def);

    b3Mesh* create_mesh(const std::filesystem::path& file_path);

    void test_step();


};


#endif //BOX3D_B3_WORLD_HPP
