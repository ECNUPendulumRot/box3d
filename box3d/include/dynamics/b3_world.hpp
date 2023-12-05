
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include  <filesystem>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"
#include "dynamics/b3_rigid_body.hpp"

#include "collision/b3_broad_phase.hpp"

namespace box3d {

    class b3World;

    class b3Body;
}


class box3d::b3World {

    //b3Body* m_body_list;

    b3Body* m_rigid_body_list;

    int32 m_body_count;

    b3Vector3d m_gravity = b3Vector3d(0, 0, 9.8);

    double m_hz = 60;

    b3BroadPhase m_broad_phase;

public:

    b3World();

    ~b3World();

    //b3Body* create_body(const b3BodyDef& def);

    void set_gravity(const b3Vector3d& gravity) {
        m_gravity = gravity;
    }

    bool empty() const {
        return m_body_count == 0;
    }

    void test_step();

    b3Body* create_rigid_body(const b3BodyDef& def);

    b3Body* create_body(const b3BodyDef& def);

    // void destroy_body(b3Body* body);


    b3BroadPhase* get_broad_phase() {
        return &m_broad_phase;
    }

protected:

    void solve_rigid(double delta_t);



};


#endif //BOX3D_B3_WORLD_HPP
