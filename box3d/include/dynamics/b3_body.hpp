
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "common/b3_allocator.hpp"

#include "dynamics/b3_transform.hpp"
#include "dynamics/b3_body_def.hpp"


/////////// Forward Delaration ///////////

class b3World;

class b3FixtureDef;

class b3Fixture;

struct b3ContactEdge;

//////////////////////////////////////////


class b3Body {

    friend class b3World;

    /**
     * @brief Type of the body
     */
    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    b3World* m_world = nullptr;


    ///////////////// Kinematic Properties /////////////////

    // CoM
    b3Vector3r m_local_center = b3Vector3r::zero();
    /**
     *  position and orientation of the center of body.
     */
    b3Transformr m_xf;
    /**
     * linear velocity and angular velocity of the center of body.
     */
    b3Transformr m_velocity;

    ////////////////// Dynamic Properties //////////////////

    real m_density = 1.0;

    real m_volume = 0.0;

    real m_mass = 0.0;

    real m_inv_mass = 0.0;

    b3Matrix3r m_inertia = b3Matrix3r::zero();

    b3Matrix3r m_inv_inertia = b3Matrix3r::zero();

    b3Vector3r m_force = b3Vector3r::zero();

    b3Vector3r m_gravity = b3Vector3r::zero();

    b3Vector3r m_torque;

    ///////////////// Collision Properties /////////////////

    b3ContactEdge* m_contact_list = nullptr;

    b3Fixture* m_fixture_list = nullptr;

    int32 m_fixture_count = 0;

    ///////////////// General linked list /////////////////

    b3Body* m_prev = nullptr;

    b3Body* m_next = nullptr;


    //////////////// Island ////////////////////////////////

    int32 m_island_index;

    uint32 m_flags = 0;

public:

    enum Flag {
        // This is used to generate islands.
        e_island_flag = 1
    };


    /**
     * @brief Construct a new b3Body object
     */
    b3Body() = default;

    explicit b3Body(const b3BodyDef& body_def);

    b3Fixture* create_fixture(const b3FixtureDef& def);

    ///////////////// Getter and Setter /////////////////

    inline b3Body* next() const {
        return m_next;
    }

    inline b3BodyType get_type() const {
        return m_type;
    }

    inline b3Vector3r get_force() const {
        return m_force;
    }

    inline b3Vector3r get_gravity() const {
        return m_gravity * m_mass;
    }

    inline b3Vector3r get_torque() const {
        return m_torque;
    }

    void apply_force(b3Vector3r& force) {
        m_force = force;
    }

    void apply_gravity(b3Vector3r& gravity) {
        m_gravity = gravity;
    }

    void apply_torque(b3Vector3r& torque) {
        m_torque = torque;
    }

    b3ContactEdge* get_contact_list() const {
        return m_contact_list;
    }

    b3Fixture* get_fixture() const {
        return m_fixture_list;
    }

    b3Transformr get_pose() const {
        return m_xf;
    }

    void set_pose(b3Transformr& xf) {
        m_xf = xf;
    }

    b3Transformr get_velocity() const {
        return m_velocity;
    }

    void set_velocity(b3Transformr& velocity) {
        m_velocity = velocity;
    }

    int32 get_island_index() const {
        return m_island_index;
    }

    real get_inv_mass() const {
        return m_inv_mass;
    }

    real get_mass() const {
        return m_mass;
    }

    b3Matrix3r get_inv_inertia() const {
        return m_inv_inertia;
    }

    b3Vector3r get_local_center() const {
        return m_local_center;
    }

    inline void set_next(b3Body* next) {
        m_next = next;
    }

    inline void set_type(b3BodyType type) {
        m_type = type;
    }

    inline void set_flag(Flag flag) {
        m_flags |= flag;
    }

    inline bool test_flag(Flag flag) {
        return m_flags & flag;
    }

    inline void unset_flag(Flag flag) {
        m_flags &= ~flag;
    }

    inline void set_world(b3World* world) {
        m_world = world;
    }

    inline void set_contact_list(b3ContactEdge* contact_list) {
        m_contact_list = contact_list;
    }

    inline void set_island_index(int32 index) {
        m_island_index = index;
    }

    /**
     * @brief destroy all fixtures of this body.
     * When the body is destroyed, will call this function to free memory.
     */
    void destroy_fixtures();

private:
    /**
     * recalculate mass, inertia, and center of mass of the body.
     */
    void reset_mass_data();

    /**
     *
     * get velocity and position of bodies in the world at the time step begin.
     * solve velocity constraints(if exist), integrate positions.
     * and then, write the position and velocity back to the body
     */
    void synchronize_fixtures();

};


#endif //BOX3D_B3_BODY_HPP
