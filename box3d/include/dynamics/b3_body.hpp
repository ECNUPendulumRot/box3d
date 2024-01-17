
#ifndef BOX3D_B3_BODY_HPP
#define BOX3D_B3_BODY_HPP


#include "common/b3_allocator.hpp"

#include "geometry/b3_mesh.hpp"

#include "dynamics/b3_pose.hpp"
#include "dynamics/b3_body_def.hpp"
#include "geometry/b3_shape.hpp"
#include "collision/b3_contact.hpp"


namespace box3d {

    class b3Body;

    class b3World;

    ////////////////////

    class b3Fixture;

    class b3FixtureDef;

}


class box3d::b3Body {

    friend class b3World;

    /**
     * @brief Type of the body
     */
    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    b3World* m_world = nullptr;


    ///////////////// Kinematic Properties /////////////////

    // CoM
    b3Vector3d m_local_center = b3Vector3d::zero();

    b3TransformD m_xf;

    b3TransformD m_velocity;

    ////////////////// Dynamic Properties //////////////////

    double m_density = 1.0;

    double m_volume = 0.0;

    double m_mass = 0.0;

    double m_inv_mass = 0.0;

    E3Matrix3d m_inertia = E3Matrix3d::Zero();

    E3Matrix3d m_inv_inertia = E3Matrix3d::Zero();

    b3Vector3d m_force;

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

    //////
    enum {
        e_island_flag = 1
    };

public:

    /**
     * @brief Construct a new b3Body object
     */
    b3Body() = default;

    explicit b3Body(const box3d::b3BodyDef &body_def);

    b3Fixture* create_fixture(const b3FixtureDef& def);

    ///////////////// Getter and Setter /////////////////

    inline b3Body* next() const {
        return m_next;
    }

    inline b3BodyType get_type() const {
        return m_type;
    }

    b3ContactEdge* get_contact_list() const {
        return m_contact_list;
    }

    b3Fixture* get_fixture() const {
        return m_fixture_list;
    }

    b3TransformD get_pose() const {
        return m_xf;
    }

    b3TransformD get_velocity() const {
        return m_velocity;
    }

    int32 get_island_index() const {
        return m_island_index;
    }

    double get_inv_mass() const {
        return m_inv_mass;
    }

    b3Matrix3d get_inv_inertia() const {
        return m_inv_inertia;
    }

    b3Vector3d get_local_center() const {
        return m_local_center;
    }

    inline void set_next(b3Body* next) {
        m_next = next;
    }

    inline void set_type(b3BodyType type) {
        m_type = type;
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

private:

    void reset_mass_data();

};


#endif //BOX3D_B3_BODY_HPP
