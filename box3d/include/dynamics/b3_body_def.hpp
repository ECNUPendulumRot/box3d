
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
    b3_RIGID,
    b3_AFFINE,
};

class box3d::b3BodyDef {

    friend class b3BodyRigid;
    friend class b3BodyAffine;

    b3BodyDefInner* m_def;

    b3BodyType m_type = b3BodyType::b3_RIGID;

    b3PoseD m_init_pose = b3PoseD::zero();

    b3PoseD m_init_velocity = b3PoseD::zero();

public:

    explicit b3BodyDef(b3BodyDefInner* def, b3BodyType type = b3BodyType::b3_RIGID) {
        m_def = def;
        m_type = type;
    }

    ~b3BodyDef();

    b3BodyDefInner* get_inner_def() const {
        return m_def;
    }

    b3BodyType get_type() const {
        return m_type;
    }

    void set_initial_status(const b3PoseD& pose, const b3PoseD& velocity);

    static b3BodyDef create_body_definition(const std::filesystem::path &file_path);

    static b3BodyDef create_rigid_definition(const nlohmann::json& body_def);

    static b3BodyDef create_affine_definition(const nlohmann::json& body_def);

};


class box3d::b3BodyDefInner {

public:

    virtual ~b3BodyDefInner() = default;

};


#endif //BOX3D_B3_BODY_DEF_HPP
