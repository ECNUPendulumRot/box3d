
#ifndef BOX3D_B3_FIXTURE_HPP
#define BOX3D_B3_FIXTURE_HPP

#include "geometry/b3_mesh.hpp"
#include "collision/b3_aabb.hpp"
#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"
#include "geometry/b3_shape.hpp"

namespace box3d {

    class b3Fixture;

    struct b3FixtureProxy;

    class b3FixtureDef;

    /////////////////////

    class b3Body;

    class b3BroadPhase;
}


class box3d::b3FixtureDef {

    double m_restitution = 0.0;

    double m_friction = 0.0;

    /**
     * @brief The shape of the fixture.
     * This shape must be allocated by the user.
     * It can be allocated on the stack because the shape will be cloned
     */
    b3Shape* m_shape = nullptr;

public:

    double get_restitution() const {
        return m_restitution;
    }

    double get_friction() const {
        return m_friction;
    }

    b3Shape* get_shape() const {
        return m_shape;
    }

};


struct box3d::b3FixtureProxy {

    enum {

        b3NullProxy = -1

    };

    b3AABB m_aabb;

    b3Fixture* m_fixture = nullptr;

    int32 m_proxy_id = -1;

    int32 m_child_id = -1;

    b3Fixture* get_fixture() const {
        return m_fixture;
    }

};


class box3d::b3Fixture {

    double m_restitution = 0.0;

    double m_friction = 0.0;

    b3Shape* m_shape = nullptr;

    b3Body* m_body = nullptr;

    b3BodyType m_type = b3BodyType::b3_RIGID;

    b3FixtureProxy* m_proxies = nullptr;

    int32 m_proxy_count = 0;

    b3Fixture* m_next = nullptr;

public:

    void create_fixture(const b3FixtureDef& f_def, b3Body* body);

    void create_proxy(b3BroadPhase* broad_phase, b3PoseD& pose);

    b3Body* get_body() const {
        return m_body;
    }

    b3Shape* get_shape() const {
        return m_shape;
    }

    void set_shape(b3Shape* shape) {
        m_shape = shape;
    } 
};

#endif //BOX3D_B3_FIXTURE_HPP
