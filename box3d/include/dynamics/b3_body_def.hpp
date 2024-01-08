
#ifndef BOX3D_B3_BODY_DEF_HPP
#define BOX3D_B3_BODY_DEF_HPP


#include "dynamics/b3_pose.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>

namespace box3d {

    class b3BodyDef;

    class b3BodyDefInner;

    class b3BodyDefRigid;

    enum class b3BodyType;
}


enum class box3d::b3BodyType {

    b3_static_body,

    b3_kinematic_body,

    b3_dynamic_body

};


class box3d::b3BodyDef {

    b3BodyType m_type = b3BodyType::b3_dynamic_body;

    b3PoseD m_init_pose = b3PoseD::zero();

    b3PoseD m_init_velocity = b3PoseD::zero();

    double m_density = 1.0;

public:

    explicit b3BodyDef(b3BodyType type = b3BodyType::b3_dynamic_body) {
        m_type = type;
    }

    ~b3BodyDef() = default;

    b3BodyType get_type() const {
        return m_type;
    }

    void set_initial_status(const b3PoseD& pose, const b3PoseD& velocity);

};


#endif //BOX3D_B3_BODY_DEF_HPP
