
#ifndef BOX3D_B3_BODY_DEF_HPP
#define BOX3D_B3_BODY_DEF_HPP


#include "dynamics/b3_pose.hpp"

#include <nlohmann/json.hpp>

namespace box3d {

    class b3BodyDef;

    class b3BodyDefRigid;

    enum class b3BodyType;
}

enum class box3d::b3BodyType {

    b3_RIGID

};

class box3d::b3BodyDef {

public:

    static b3BodyDef* create_body_definition(const nlohmann::json& json);

};

class box3d::b3BodyDefRigid: public b3BodyDef {

    /**
     * @brief The density of the rigid body.
     * Unit: kg/m^3
     */
    double m_density = 1.0;

    b3PoseD m_init_pose = b3PoseD::zero();

    b3PoseD m_init_velocity = b3PoseD::zero();

    b3BodyType m_type = b3BodyType::b3_RIGID;

public:

    b3BodyDefRigid() = default;

    explicit b3BodyDefRigid(const nlohmann::json& json);

};

#endif //BOX3D_B3_BODY_DEF_HPP
