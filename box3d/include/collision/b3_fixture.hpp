
#ifndef BOX3D_B3_FIXTURE_HPP
#define BOX3D_B3_FIXTURE_HPP


#include "geometry/b3_shape.hpp"

#include "collision/b3_aabb.hpp"

/////////// Forward Delaration ///////////

class b3Fixture;

class b3BroadPhase;

//////////////////////////////////////////


struct b3FixtureDef {

    double m_restitution = 0.0;

    double m_friction = 0.0;

    double m_density = 1.0;

    /**
     * @brief The shape of the fixture.
     * This shape must be allocated by the user.
     * It can be allocated on the stack because the shape will be cloned
     */
    b3Shape* m_shape = nullptr;

public:

    inline double get_restitution() const {
        return m_restitution;
    }

    inline double get_friction() const {
        return m_friction;
    }

    inline b3Shape* get_shape() const {
        return m_shape;
    }

    inline double get_density() const {
        return m_density;
    }

};


struct b3FixtureProxy {

    enum {

        b3NullProxy = -1

    };

    b3AABB m_aabb;

    b3Fixture* m_fixture = nullptr;

    int32 m_proxy_id = -1;

    // the id of the proxy of the child shape
    int32 m_child_id = -1;

    inline b3Fixture* get_fixture() const {
        return m_fixture;
    }

};


class b3Fixture {

    friend class b3Body;

    double m_restitution = 0.0;

    double m_friction = 0.0;

    double m_density = 0.0;

    b3Shape* m_shape = nullptr;

    b3Body* m_body = nullptr;

    b3FixtureProxy* m_proxies = nullptr;

    int32 m_proxy_count = 0;

    b3Fixture* m_next = nullptr;

public:

    void create_fixture(b3BlockAllocator* block_allocator,  const b3FixtureDef& f_def, b3Body* body);

    void create_proxy(b3BroadPhase* broad_phase, b3TransformD& m_xf);

    inline void get_mass_data(b3MassProperty& mass_data) const {
        m_shape->compute_mass_properties(mass_data, m_density);
    }

    b3Body* get_body() const {
        return m_body;
    }

    b3Shape* get_shape() const {
        return m_shape;
    }
 
    void set_shape(b3Shape* shape) {
        m_shape = shape;
    }

    inline b3ShapeType get_shape_type() const {
        return m_shape->get_type();
    }

    b3FixtureProxy* get_fixture_proxy(int index) {
        b3_assert(index >= 0 && index < m_proxy_count);
        return (m_proxies + index);
    }
};


#endif //BOX3D_B3_FIXTURE_HPP
