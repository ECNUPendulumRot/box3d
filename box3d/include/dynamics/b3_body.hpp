
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP

#include "common/b3_allocator.hpp"
#include "math/b3_quat.hpp"
#include "dynamics/b3_transform.hpp"
#include "dynamics/b3_body_def.hpp"

#include <vector>

/////////// Forward Delaration ///////////

class b3World;

class b3FixtureDef;

class b3Fixture;

struct b3ContactEdge;

class b3ConstraintBase;

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
    b3Vec3r m_local_center = b3Vec3r::zero();

    // the quaternion of the body
    b3Quaternionr m_q;

    // the position of the body
    b3Vec3r m_p;

    // the linear velocity of the body
    b3Vec3r m_v;

    // the angular velocity of the body
    // in form of angle axis
    b3Vec3r m_w;

    b3Transformr m_world_transform;

    ////////////////// Dynamic Properties //////////////////

    real m_density = 1.0;

    real m_volume = 0.0;

    real m_mass = 0.0;

    real m_inv_mass = 0.0;

    b3Mat33r m_local_inertia = b3Mat33r::zero();

    b3Mat33r m_local_inv_inertia = b3Mat33r::zero();

    b3Mat33r m_inv_inertia_tensor_world;

    b3Mat33r m_inertia = b3Mat33r::zero();

    b3Mat33r m_inv_inertia = b3Mat33r::zero();

    b3Vec3r m_force = b3Vec3r::zero();

    real m_gravity_scale = 1.0;

    b3Vec3r m_torque;

    real m_linear_damping = 0.0;

    real m_angular_damping = 0.0;

    ///////////////// Collision Properties /////////////////

    b3ContactEdge* m_contact_list = nullptr;

    b3Fixture* m_fixture_list = nullptr;

    int32 m_fixture_count = 0;

    ///////////////// General linked list /////////////////

    b3Body* m_prev = nullptr;

    b3Body* m_next = nullptr;


    //////////////// Island ////////////////////////////////

    int32 m_island_index;

    int32 m_companion_id;

    uint32 m_flags = 0;

    std::vector<b3ConstraintBase*> m_constraints;

public:

    enum Flag {
        // This is used to generate islands.
        e_island_flag = 1
    };

    enum CollisionFlags {
        B3_DYNAMIC_OBJECT = 0,
        B3_STATIC_OBJECT = 1,
        B3_KINEMATIC_OBJECT = 2,
        B3_NO_CONTACT_RESPONSE = 4
    };

    /**
     * @brief Construct a new b3Body object
     */
    b3Body() = default;

    explicit b3Body(const b3BodyDef& body_def);

    b3Fixture* create_fixture(const b3FixtureDef& def);

    b3Vec3r get_velocity_in_local_point(const b3Vec3r& rel_pos) const {
        return m_v + m_w.cross(rel_pos);
    }

    bool add_constraint(b3ConstraintBase* constraint);

    int find_constraint(b3ConstraintBase* constraint);

    bool remove_constraint(b3ConstraintBase* constraint);

    bool should_collide(const b3Body* other);

    b3Vec3r compute_gyro_scopic_implicit(real dt) const;

    ///////////////// Getter and Setter /////////////////

    inline b3Body* next() const {
        return m_next;
    }

//    const b3Vec3r& get_inv_inertia_diag_local() const {
//        return m_local_inv_inertia;
//    }

    inline b3BodyType get_type() const {
        return m_type;
    }

    int32 get_companion_id() const {
        return m_companion_id;
    }

    void set_companion_id(int32 id) {
        m_companion_id = id;
    }

    inline b3Vec3r get_force() const {
        return m_force;
    }

    void set_gravity_scale(real scale);

    b3Vec3r get_gravity() const;

    inline b3Vec3r get_torque() const {
        return m_torque;
    }

    void apply_force(b3Vec3r& force) {
        m_force = force;
    }

    b3Mat33r get_inv_inertia_local() const {
        return m_local_inv_inertia;
    }

    void apply_gravity();

    void clear_forces() {
        m_force.set(0, 0, 0);
        m_torque.set(0, 0 , 0);
    }

    b3Mat33r get_local_inertia() const;

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
        // return m_p;
        return m_world_transform.position();
    }

    b3Transformr get_world_transform() const {
        return m_world_transform;
    }

    b3Quaternionr get_quaternion() const {
        // return m_q;
        return m_world_transform.get_rotation();
    }

    void set_quaternion(const b3Quaternionr& q) {
        m_q = q;
    }

    void update_inertia_tensor();

    void set_position(const b3Vec3r& p) {
        m_p = p;
    }

    b3Vec3r get_linear_velocity() const {
        return m_v;
    }

    b3Vec3r get_angular_velocity() const {
        return m_w;
    }

    void set_linear_velocity(const b3Vec3r& v) {
        m_v = v;
    }

    void set_angular_velocity(const b3Vec3r& w) {
        m_w = w;
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

    real get_linear_damping() const {
        return m_linear_damping;
    }

    real get_angular_damping() const {
        return m_angular_damping;
    }

    b3Mat33r get_inv_inertia() const {
        return m_inv_inertia;
    }

    b3Mat33r get_inv_inertia_tensor_world() const {
        return m_inv_inertia_tensor_world;
    }

    // TODO: delete
    b3Mat33r get_inertia() const {
        return m_inertia;
    }

    b3Vec3r get_local_center() const {
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

    void proceed_to_transform(const b3Transformr& new_transform) {
        m_world_transform = new_transform;
        update_inertia_tensor();
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

    real kinetic_energy() const;

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
