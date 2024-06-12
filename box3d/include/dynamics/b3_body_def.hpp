
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

    b3Vec3r m_init_p = b3Vec3r::zero();

    b3Quatr m_init_q;

    b3Vec3r m_init_v = b3Vec3r::zero();

    b3Vec3r m_init_w = b3Vec3r::zero();

    real m_density = 1.0;

public:

    explicit b3BodyDef(b3BodyType type = b3BodyType::b3_dynamic_body) {
        m_type = type;
    }

    ~b3BodyDef() = default;

    b3BodyType get_type() const {
        return m_type;
    }

    inline void set_init_pose(const b3Vec3r& position, const b3Vec3r& aa) {
        m_init_p = position;
        m_init_q = b3_aa_to_quaternion(aa);
    }

    inline void set_init_velocity(const b3Vec3r& v, const b3Vec3r& w) {
        m_init_v = v;
        m_init_w = w;
    }

    inline real get_density() const {
        return m_density;
    }
};


#endif //BOX3D_B3_BODY_DEF_HPP
