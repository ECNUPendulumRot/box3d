
#include "dynamics/b3_body_def.hpp"

#include <fstream>

#include "utils/b3_json.hpp"
#include "common/b3_allocator.hpp"
#include "dynamics/b3_body_rigid.hpp"
#include "dynamics/b3_body_affine.hpp"

box3d::b3BodyDef::~b3BodyDef()
{
    b3_free(m_def);
}


box3d::b3BodyDef box3d::b3BodyDef::create_body_definition(const std::filesystem::path &file_path)
{
    std::ifstream body_def_path(file_path.string());
    nlohmann::json body_def = nlohmann::json::parse(body_def_path);

    std::string type = body_def["body_type"];

    if (type == "rigid") {
        return create_rigid_definition(body_def);
    } else if (type == "affine") {
        return create_affine_definition(body_def);
    }
}


box3d::b3BodyDef box3d::b3BodyDef::create_rigid_definition(const nlohmann::json &body_def)
{
    // The merge operation is used for the default values
    // Users may not specify some of the properties.
    nlohmann::json full_rigid = R"({
        "body_type": "rigid",
        "density": 1
    })"_json;
    full_rigid.merge_patch(body_def);

    double density = full_rigid["density"];

    return b3BodyDefRigid::create_definition(density);
}


box3d::b3BodyDef box3d::b3BodyDef::create_affine_definition(const nlohmann::json &body_def)
{
    nlohmann::json full_affine = R"({
        "body_type": "affine",
        "density": 1,
        "stiffness": 100
    })"_json;
    full_affine.merge_patch(body_def);

    double density = full_affine["density"];
    double stiffness = full_affine["stiffness"];
    return b3BodyDefAffine::create_definition(stiffness, density);
}


void box3d::b3BodyDef::set_initial_status(const b3PoseD &pose, const b3PoseD &velocity)
{
    m_init_pose = pose;
    m_init_velocity = velocity;
}







