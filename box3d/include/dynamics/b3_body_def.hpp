
#ifndef BOX3D_B3_BODY_DEF_HPP
#define BOX3D_B3_BODY_DEF_HPP


#include "dynamics/b3_transform.hpp"


enum class b3BodyType {

    b3_type_not_defined = -1,

    b3_static_body = 0,

    b3_kinematic_body = 1,

    b3_dynamic_body = 2,

    b3_body_type_count = 3

};


struct b3BodyDef {

    friend class b3Body;

    friend class b3World;

    b3BodyType m_type = b3BodyType::b3_type_not_defined;

    b3Transformr m_init_pose = b3Transformr::zero();

    b3Transformr m_init_velocity = b3Transformr::zero();

    real m_density = 1.0;

public:

    explicit b3BodyDef(b3BodyType type = b3BodyType::b3_dynamic_body) {
        m_type = type;
    }

    ~b3BodyDef() = default;

    b3BodyType get_type() const {
        return m_type;
    }

    inline void set_initial_status(const b3Transformr& pose, const b3Transformr& velocity) {
        m_init_pose = pose;
        m_init_velocity = velocity;
    }

    inline b3Transformr get_initial_pose() const {
        return m_init_pose;
    }

    inline b3Transformr get_initial_velocity() const {
        return m_init_velocity;
    }

    inline real get_density() const {
        return m_density;
    }
};


#endif //BOX3D_B3_BODY_DEF_HPP
