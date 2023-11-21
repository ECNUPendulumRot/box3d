
#ifndef BOX3D_B3_FIXTURE_HPP
#define BOX3D_B3_FIXTURE_HPP

#include "geometry/b3_mesh.hpp"
#include "collision/b3_aabb.hpp"
#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"

namespace box3d {

    class b3Fixture;

    class b3FixtureProxy;

    class b3FixtureDef;

    class b3Body;
}


class box3d::b3FixtureDef {

    double m_restitution = 0.0;

    double m_friction = 0.0;

public:

    double get_restitution() const {
        return m_restitution;
    }

    double get_friction() const {
        return m_friction;
    }

};


class box3d::b3FixtureProxy {

    friend class b3Fixture;

    enum {

        b3NullProxy = -1

    };

    b3AABB m_aabb;
    b3Fixture* m_fixture;
    int32 m_proxy_id;

};


class box3d::b3Fixture {

    double m_restitution = 0.0;

    double m_friction = 0.0;

    b3Mesh* m_body_mesh = nullptr;

    b3Body* m_body = nullptr;

    b3BodyType m_type = b3BodyType::b3_RIGID;

    b3FixtureProxy* m_proxy = nullptr;

public:

    void create_fixture(const b3FixtureDef& f_def, b3Body* body);

    void create_rigid_proxy();

};

#endif //BOX3D_B3_FIXTURE_HPP
