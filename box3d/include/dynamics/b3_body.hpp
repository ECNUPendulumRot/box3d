
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "common/b3_allocator.hpp"
#include "math/b3_quat.hpp"
#include "dynamics/b3_transform.hpp"
#include "dynamics/b3_body_def.hpp"


/////////// Forward Delaration ///////////

class b3World;

class b3FixtureDef;

class b3Fixture;

class b3BodySim;

class b3Body;

struct b3ContactEdge;

//////////////////////////////////////////


struct b3BodySim {

    b3Vec3r p;// the position of the body
    b3Quatr q;// the quaternion of the body
    b3Vec3r v;// the linear velocity of the body
    b3Vec3r w;// the angular velocity of the body, in form of angle axis
    b3Body* body;
    uint32 flag;
    int32 static_island_index = -1;
    int32 normal_island_index = -1;
    int32 index = -1;

    static b3BodySim from_body(b3Body* body);
};


struct b3Sweep {

    b3Sweep() = default;

    void advance(real alpha);

    void get_transform(b3Transr &xf, real beta) const;

    real alpha0;
    b3Vec3r m_local_center;
    b3Vec3r p0, p;
    b3Quatr q0, q;
};


class b3Body {

public:

    /**
     * @brief Type of the body
     */
    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    b3World* m_world = nullptr;


    ///////////////// Kinematic Properties /////////////////

    // CoM
    b3Vec3r m_local_center = b3Vec3r::zero();

    // the quaternion of the body
    b3Quatr m_q;

    // the position of the body
    b3Vec3r m_p;

    // the linear velocity of the body
    b3Vec3r m_v;

    // the angular velocity of the body
    // in form of angle axis
    b3Vec3r m_w;

    ////////////////// Dynamic Properties //////////////////

    real m_density = 1.0;

    real m_volume = 0.0;

    real m_mass = 0.0;

    real m_inv_mass = 0.0;

    b3Mat33r m_inertia = b3Mat33r::zero();

    b3Mat33r m_inv_inertia = b3Mat33r::zero();

    b3Vec3r m_force = b3Vec3r::zero();

    b3Vec3r m_gravity = b3Vec3r::zero();

    b3Vec3r m_torque;

    ///////////////// Collision Properties /////////////////

    b3ContactEdge* m_contact_list = nullptr;

    b3Fixture* m_fixture_list = nullptr;

    int32 m_fixture_count = 0;

    ///////////////// General linked list /////////////////

    b3Body* m_prev = nullptr;

    b3Body* m_next = nullptr;

    //////////////// Island ////////////////////////////////

    int32 m_normal_island_index;
    int32 m_static_island_index;
    uint32 m_flags = 0;

    real m_sleep_time = 0.0;

    b3Sweep m_sweep;

    b3BodySim m_body_sim;

    b3BodySim* generate_sim_body();

    enum Flag {
        e_island_flag = 0x0001,
        e_normal_island_flag = 0x0002,
        e_static_island_flag = 0x0004,
        e_awake_flag = 0x0008,
        e_auto_sleep_flag = 0x0010,
    };

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

    inline b3Vec3r get_force() const {
        return m_force;
    }

    inline b3Vec3r get_gravity() const {
        return m_gravity * m_mass;
    }

    inline b3Vec3r get_torque() const {
        return m_torque;
    }

    void apply_force(b3Vec3r& force) {
        m_force = force;
    }

    void apply_gravity(b3Vec3r& gravity) {
        m_gravity = gravity;
    }

    void apply_torque(b3Vec3r& torque) {
        m_torque = torque;
    }

    b3ContactEdge* get_contact_list() const {
        return m_contact_list;
    }

    b3Fixture* get_fixture_list() const {
        return m_fixture_list;
    }

    b3Vec3r get_position() const {
        return m_p;
    }

    b3Quatr get_quaternion() const {
        return m_q;
    }

    void set_quaternion(const b3Quatr& q) {
        m_q = q;
    }

    void set_position(const b3Vec3r& p) {
        m_p = p;
    }

    const b3Vec3r& get_linear_velocity() const {
        return m_v;
    }

    const b3Vec3r& get_angular_velocity() const {
        return m_w;
    }

    const b3Vec3r& get_local_center() {
        return m_local_center;
    }

    void set_linear_velocity(b3Vec3r& v) {
        m_v = v;
    }

    void set_angular_velocity(b3Vec3r& w) {
        m_w = w;
    }

    const real& get_inv_mass() const {
        return m_inv_mass;
    }

    const real& get_mass() const {
        return m_mass;
    }

    const b3Mat33r& get_inv_inertia() const {
        return m_inv_inertia;
    }

    const b3Mat33r& get_inertia() const {
        return m_inertia;
    }

    const b3Vec3r& get_local_center() const {
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

    inline void set_normal_island_index(int32 index) {
        m_normal_island_index = index;
    }

    inline void set_static_island_index(int32 index) {
        m_static_island_index = index;
    }

    inline void set_awake(bool flag) {
        if (m_type == b3BodyType::b3_static_body) {
            return;
        }
        if (flag) {
            m_flags |= e_awake_flag;
            m_sleep_time = 0.0f;
        } else {
            m_flags &= ~e_awake_flag;
            m_sleep_time = 0.0;
            m_v = b3Vec3r::zero();
            m_w = b3Vec3r::zero();
        }
    }

    inline bool is_awake() {
        return (m_flags & e_awake_flag) == e_awake_flag;
    }

    void destroy_fixtures();

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
