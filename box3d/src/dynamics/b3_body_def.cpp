
#include "dynamics/b3_body_def.hpp"

#include <fstream>

#include "utils/b3_json.hpp"
#include "common/b3_allocator.hpp"


box3d::b3BodyDef box3d::b3BodyDef::create_body_definition(const std::filesystem::path &file_path)
{
    std::ifstream body_def_path(file_path.string());
    nlohmann::json body_def = nlohmann::json::parse(body_def_path);

    std::string type = body_def["body_type"];

    if (type == "rigid") {
        return create_rigid_definition(body_def);
    }
}


box3d::b3BodyDef box3d::b3BodyDef::create_rigid_definition(const nlohmann::json &body_def)
{
    nlohmann::json full_rigid = R"({
        "body_type": "rigid",
        "density": 1
    })"_json;
    full_rigid.merge_patch(body_def);

    double density = full_rigid["density"];

    b3BodyDefInner* def = b3BodyDefRigid::create_definition(density);

    return b3BodyDef(def, b3BodyType::b3_RIGID);
}


box3d::b3BodyDef::~b3BodyDef()
{
    b3_free(m_def);
}


void box3d::b3BodyDef::set_initial_status(const b3PoseD &pose, const b3PoseD &velocity)
{
    m_init_pose = pose;
    m_init_velocity = velocity;
}


//////////////////////////////////////////////////////////////////////////////////////////


box3d::b3BodyDefRigid::b3BodyDefRigid(double density)
{
    m_density = density;
}


box3d::b3BodyDefRigid* box3d::b3BodyDefRigid::create_definition(double density)
{
    void* memory = b3_alloc(sizeof(b3BodyDefRigid));

    return new(memory) b3BodyDefRigid(density);
}

