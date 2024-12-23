
#ifndef BOX3D_B3_FIXTURE_HPP
#define BOX3D_B3_FIXTURE_HPP


#include "geometry/b3_shape.hpp"

#include "collision/b3_aabb.hpp"

/////////// Forward Delaration ///////////

class b3Fixture;

class b3BroadPhase;

//////////////////////////////////////////


struct b3FixtureDef {

    real m_restitution = 0.0;

    real m_friction = 0.0;

    real m_rolling_friction = 0.0;

    real m_spinning_friction = 0.0;

    real m_density = 1.0;

    /**
     * @brief The shape of the fixture.
     * This shape must be allocated by the user.
     * It can be allocated on the stack because the shape will be cloned
     */
    b3Shape* m_shape = nullptr;

    b3Transformr m_local_transform;
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

    real m_restitution = 0.0;

    real m_friction = 0.0;

    real m_rolling_friction = 0.0;

    real m_spinning_friction = 0.0;

    // TODO: replace this with m_mass
    real m_density = 0.0;

    b3Shape* m_shape = nullptr;

    b3Body* m_body = nullptr;

    b3FixtureProxy* m_proxies = nullptr;

    int32 m_proxy_count = 0;

    b3Fixture* m_next = nullptr;

    b3Transformr m_local_transform;

public:

    b3Transformr get_world_transform(const b3Transformr& xf);

    b3Transformr get_local_transform() const {
        return m_local_transform;
    }

    void create_fixture(b3BlockAllocator* block_allocator,  const b3FixtureDef& f_def, b3Body* body);

    void create_proxy(b3BroadPhase* broad_phase, b3Transformr& m_xf);

    inline void get_mass_data(b3MassProperty& mass_data) const {
        m_shape->compute_mass_properties(mass_data, m_density);
    }

    b3Vec3r get_center_offset() const {
        return m_local_transform.position();
    }

    b3Body* get_body() const {
        return m_body;
    }

    b3Fixture* get_next() const {
        return m_next;
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

    real get_friction() const {
        return m_friction;
    }

    real get_restitution() const {
        return m_restitution;
    }

    real get_rolling_friction() const {
        return m_rolling_friction;
    }

    real get_spinning_friction() const {
        return m_spinning_friction;
    }

private:

    void synchronize(b3BroadPhase* broad_phase, const b3Transformr& transform1, const b3Transformr& transform2);

};


#endif //BOX3D_B3_FIXTURE_HPP
