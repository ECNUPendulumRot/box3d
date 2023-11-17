
#include "dynamics/b3_body_def.hpp"

#include <utils/b3_json.hpp>

#include "common/b3_allocator.hpp"


box3d::b3BodyDef box3d::b3BodyDef::create_body_definition(const nlohmann::json &json)
{

    std::string type = json["body_type"];

    if (type == "rigid") {

        b3BodyDefInner* def =  new ((b3BodyDefRigid*)b3_alloc(sizeof(b3BodyDefRigid))) b3BodyDefRigid(json);

        return b3BodyDef(def, b3BodyType::b3_RIGID);
    }

}

box3d::b3BodyDef::~b3BodyDef()
{
    b3_free(m_def);
}


box3d::b3BodyDefRigid::b3BodyDefRigid(const nlohmann::json &json)
{
    nlohmann::json full_rigid = R"({
        "body_type": "rigid",

        "density": 1,
        "initial position":    [0, 0, 0],
        "initial orientation": [0, 0, 0],

        "initial linear velocity":  [0, 0, 0],
        "initial angular velocity": [0, 0, 0]
    })"_json;

    full_rigid.merge_patch(json);

    Eigen::Vector3d init_position;
    Eigen::Vector3d init_orientation;

    Eigen::Vector3d init_linear_velocity;
    Eigen::Vector3d init_angular_velocity;

    from_json(full_rigid["initial position"],         init_position);
    from_json(full_rigid["initial orientation"],      init_orientation);
    from_json(full_rigid["initial linear velocity"],  init_linear_velocity);
    from_json(full_rigid["initial angular velocity"], init_angular_velocity);

    m_density = full_rigid["density"];
    m_init_pose = b3PoseD(init_position, init_orientation);
    m_init_velocity = b3PoseD(init_linear_velocity, init_angular_velocity);

    m_type = b3BodyType::b3_RIGID;
}
