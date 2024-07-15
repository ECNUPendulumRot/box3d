// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "common/b3_allocator.hpp"
#include "math/b3_quat.hpp"
#include "dynamics/b3_transform.hpp"
#include "dynamics/b3_body_def.hpp"


/////////// Forward Declaration ///////////

class b3World;

class b3FixtureDef;

class b3Fixture;

struct b3ContactEdge;

//////////////////////////////////////////


/**
 * @brief  representing the state of an object over a continuous period.
 */
struct b3Sweep {

    /**
     * @brief Default constructor.
     */
    b3Sweep() = default;

    /**
     * @brief The advance function advances the sweep to a given interpolation factor alpha.
     * @param alpha The interpolation factor to advance
     */
    void advance(real alpha);

    /**
     * @brief calculates the interpolated transformation of an object based on a
     * given interpolation factor beta.
     * @param xf This is a reference to a b3Transr object that will hold the
     * interpolated position and orientation after the function executes.
     * @param beta The interpolation factor to calculate the transformation.
     */
    void get_transform(b3Transr &xf, real beta) const;

    real alpha0; ///< Stores the initial alpha value used for interpolation.
    b3Vec3r m_local_center; ///< Stores the local center of the object.
    b3Vec3r p0, p; ///< p0: Stores the initial position of the object. p: Stores the final position of the object.
    b3Quatr q0, q; ///< q0:  Stores the initial orientation of the object. q: Stores the final orientation of the object.
};

/**
 * @brief The b3Body class represents a physical body in a physics simulation world.
 */
class b3Body {

    /**
     * @brief Allows the b3World class to access private and protected members of the b3Body class.
     */
    friend class b3World;

    /**
     * @brief Allows the b3ContactManager class to access private and protected members of the b3Body class.
     */
    friend class b3ContactManager;

    /**
     * @brief Allows the b3Solver class to access private and protected members of the b3Body class.
     */
    friend class b3Solver;

    /**
     * @brief Type of the body
     */
    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    /**
     * @brief  Pointer to the world that this body belongs to.
     */
    b3World* m_world = nullptr;


    ///////////////// Kinematic Properties /////////////////

    // CoM
    b3Vec3r m_local_center = b3Vec3r::zero(); ///< CoM

    // the quaternion of the body
    b3Quatr m_q; ///< the quaternion of the body

    // the position of the body
    b3Vec3r m_p; ///< the position of the body

    // the linear velocity of the body
    b3Vec3r m_v; ///< the linear velocity of the body

    // the angular velocity of the body
    // in form of angle axis
    b3Vec3r m_w; ///< the angular velocity of the body,in form of angle axis

    ////////////////// Dynamic Properties //////////////////

    /**
     * @brief Density of the body. Default Value: 1.0
     */
    real m_density = 1.0;

    /**
     * @brief Volume of the body. Default Value: 0.0
     */
    real m_volume = 0.0;

    /**
     * @brief Mass of the body. Default Value: 0.0
     */
    real m_mass = 0.0;

    /**
     * @brief Inverse mass of the body. Default Value: 0.0
     */
    real m_inv_mass = 0.0;

    /**
     * @brief Inertia tensor of the body. Default Value: b3Mat33r::zero()
     */
    b3Mat33r m_inertia = b3Mat33r::zero();

    /**
     * @brief Inverse inertia tensor of the body.
     */
    b3Mat33r m_inv_inertia = b3Mat33r::zero();

    /**
     * @brief Force acting on the body.
     */
    b3Vec3r m_force = b3Vec3r::zero();

    /**
     * @brief Gravity acting on the body.
     */
    b3Vec3r m_gravity = b3Vec3r::zero();

    /**
     * @brief Torque acting on the body.
     */
    b3Vec3r m_torque;

    ///////////////// Collision Properties /////////////////

    /**
     * @brief  List of contacts involving this body.
     */
    b3ContactEdge* m_contact_list = nullptr;

    /**
     * @brief List of fixtures attached to this body.
     */
    b3Fixture* m_fixture_list = nullptr;

    /**
     * @brief Number of fixtures attached to this body.
     */
    int32 m_fixture_count = 0;

    ///////////////// General linked list /////////////////

    /**
     * @brief Pointer to the previous body in the list.
     */
    b3Body* m_prev = nullptr;

    /**
     * @brief  Pointer to the next body in the list.
     */
    b3Body* m_next = nullptr;

    //////////////// Island ////////////////////////////////

    /**
     * @brief Index of the body in the island.
     */
    int32 m_island_index;

    /**
     * @brief Flags indicating various states of the body.
     */
    uint32 m_flags = 0;

    /**
     * @brief  Time the body has been in a sleeping state.
     */
    real m_sleep_time = 0.0;

    /**
     * @brief Sweep information for continuous collision detection.
     */
    b3Sweep m_sweep;

public:

    /**
     * @brief defines symbolic constants that represent specific flags or states
     * associated with a b3Body object in a physics simulation.
     */
    enum Flag {
        e_island_flag = 0x0001, ///< Indicates whether the body is part of an island in the simulation.
        e_awake_flag = 0x0002, ///< Indicates whether the body is currently awake.
        e_auto_sleep_flag = 0x0004, ///< Indicates whether the body can automatically enter sleep mode.
    };

    /**
     * @brief  Default constructor.
     */
    b3Body() = default;

    /**
     * @brief  Constructor that initializes the body with a definition.
     * @param body_def A reference to a b3BodyDef structure that contains the initial
     * configuration for the body.
     */
    explicit b3Body(const b3BodyDef& body_def);

    /**
     * @brief The function creates and attaches a fixture to the body, defining the
     * shape and physical properties of the body.
     * @param def  A reference to a b3FixtureDef structure that contains the configuration
     * for the fixture
     * @return Returns a pointer to the newly created b3Fixture object.
     */
    b3Fixture* create_fixture(const b3FixtureDef& def);

    ///////////////// Getter and Setter /////////////////

    /**
     * @brief gets the next body in the list.
     * @return Returns the next body in the list.
     */
    inline b3Body* next() const {
        return m_next;
    }

    /**
     * @brief gets the type of the body.
     * @return Returns the type of the body.
     */
    inline b3BodyType get_type() const {
        return m_type;
    }

    /**
     * @brief gets the force acting on the body.
     * @return Returns the force acting on the body.
     */
    inline b3Vec3r get_force() const {
        return m_force;
    }

    /**
     * @brief gets the gravity force on the body.
     * @return Returns the gravity force on the body.
     */
    inline b3Vec3r get_gravity() const {
        return m_gravity * m_mass;
    }

    /**
     * @brief gets the torque acting on the body.
     * @return Returns the torque acting on the body.
     */
    inline b3Vec3r get_torque() const {
        return m_torque;
    }

    /**
     * @brief Applies a force to the body.
     * @param force Force acting on the body.
     */
    void apply_force(b3Vec3r& force) {
        m_force = force;
    }

    /**
     * @brief  Applies gravity to the body.
     * @param gravity Gravity acting on the body.
     */
    void apply_gravity(b3Vec3r& gravity) {
        m_gravity = gravity;
    }

    /**
     * @brief Applies a torque to the body.
     * @param torque Torque acting on the body.
     */
    void apply_torque(b3Vec3r& torque) {
        m_torque = torque;
    }

    /**
     * @brief Returns the contact list associated with the body.
     * @return A pointer to the b3ContactEdge list.
     */
    b3ContactEdge* get_contact_list() const {
        return m_contact_list;
    }

    /**
     * @brief Returns the fixture list associated with the body.
     * @return A pointer to the b3Fixture list.
     */
    b3Fixture* get_fixture_list() const {
        return m_fixture_list;
    }

    /**
     * @brief Returns the position of the body.
     * @return A b3Vec3r representing the position
     */
    b3Vec3r get_position() const {
        return m_p;
    }

    /**
     * @brief Returns the orientation of the body as a quaternion.
     * @return A b3Quatr representing the orientation.
     */
    b3Quatr get_quaternion() const {
        return m_q;
    }

    /**
     * @brief Sets the orientation of the body.
     * @param q The quaternion representing the new orientation.
     */
    void set_quaternion(const b3Quatr& q) {
        m_q = q;
    }

    /**
     * @brief Sets the position of the body.
     * @param p The vector representing the new position.
     */
    void set_position(const b3Vec3r& p) {
        m_p = p;
    }

    /**
     * @brief Returns the linear velocity of the body.
     * @return A reference to a b3Vec3r representing the linear velocity.
     */
    const b3Vec3r& get_linear_velocity() const {
        return m_v;
    }

    /**
     * @brief Returns the angular velocity of the body.
     * @return A reference to a b3Vec3r representing the angular velocity.
     */
    const b3Vec3r& get_angular_velocity() const {
        return m_w;
    }

    /**
     * @brief Returns the local center of mass of the body.
     * @return A reference to a b3Vec3r representing the local center of mass.
     */
    const b3Vec3r& get_local_center() {
        return m_local_center;
    }

    /**
     * @brief Sets the linear velocity of the body.
     * @param v The vector representing the new linear velocity.
     */
    void set_linear_velocity(b3Vec3r& v) {
        m_v = v;
    }

    /**
     * @brief Sets the angular velocity of the body.
     * @param w The vector representing the new angular velocity.
     */
    void set_angular_velocity(b3Vec3r& w) {
        m_w = w;
    }

    /**
     * @brief Returns the island index of the body.
     * @return A reference to the int32 representing the island index.
     */
    const int32& get_island_index() const {
        return m_island_index;
    }

    /**
     * @brief Returns the inverse mass of the body.
     * @return A reference to a real representing the inverse mass.
     */
    const real& get_inv_mass() const {
        return m_inv_mass;
    }

    /**
     * @brief Returns the mass of the body.
     * @return A reference to a real representing the mass.
     */
    const real& get_mass() const {
        return m_mass;
    }

    /**
     * @brief Returns the inverse inertia matrix of the body.
     * @return A reference to a b3Mat33r representing the inverse inertia matrix.
     */
    const b3Mat33r& get_inv_inertia() const {
        return m_inv_inertia;
    }

    /**
     * @brief Returns the inertia matrix of the body.
     * @return A reference to a b3Mat33r representing the inertia matrix
     */
    const b3Mat33r& get_inertia() const {
        return m_inertia;
    }

    /**
     * @brief Returns the local center of the body.
     * @return A reference to a b3Vec3r representing the inertia matrix
     */
    const b3Vec3r& get_local_center() const {
        return m_local_center;
    }

    /**
     * @brief Sets the next body in the linked list.
     * @param next A pointer to the next body in the list.
     */
    inline void set_next(b3Body* next) {
        m_next = next;
    }

    /**
     * @brief Sets the type of the body.
     * @param type The new type of the body.
     */
    inline void set_type(b3BodyType type) {
        m_type = type;
    }

    /**
     * @brief Sets a specific flag for the body.
     * @param flag The flag to be set.
     */
    inline void set_flag(Flag flag) {
        m_flags |= flag;
    }

    /**
     * @brief Tests if a specific flag is set.
     * @param flag The flag to be tested.
     * @return Returns true if the flag is set, false otherwise.
     */
    inline bool test_flag(Flag flag) {
        return m_flags & flag;
    }

    /**
     * @brief Unsets a specific flag for the body.
     * @param flag The flag to be unset.
     */
    inline void unset_flag(Flag flag) {
        m_flags &= ~flag;
    }

    /**
     * @brief Sets the world to which the body belongs.
     * @param world A pointer to the world.
     */
    inline void set_world(b3World* world) {
        m_world = world;
    }

    /**
     * @brief Sets the contact list for the body.
     * @param contact_list A pointer to the contact list.
     */
    inline void set_contact_list(b3ContactEdge* contact_list) {
        m_contact_list = contact_list;
    }

    /**
     * @brief Sets the island index for the body.
     * @param index The new island index.
     */
    inline void set_island_index(int32 index) {
        m_island_index = index;
    }

    /**
     * @brief Sets the body's awake state.
     * @param flag If true, the body is set to awake; if false, the body is set to sleep.
     */
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

    /**
     * @brief Checks if the body is awake.
     * @return Returns true if the body is awake, false otherwise.
     */
    inline bool is_awake() {
        return (m_flags & e_awake_flag) == e_awake_flag;
    }

    /**
     * @brief responsible for deallocating all fixtures associated with the b3Body
     * object, including their shapes, from the physics world.
     */
    void destroy_fixtures();

private:
    /**
     * @brief recalculate mass, inertia, and center of mass of the body.
     */
    void reset_mass_data();

    /**
     * @brief get velocity and position of bodies in the world at the time step begin.
     * solve velocity constraints(if exist), integrate positions.
     * and then, write the position and velocity back to the body
     */
    void synchronize_fixtures();

};


#endif //BOX3D_B3_BODY_HPP
