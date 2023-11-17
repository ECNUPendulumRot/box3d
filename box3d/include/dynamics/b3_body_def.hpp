
#ifndef BOX3D_B3_BODY_DEF_HPP
#define BOX3D_B3_BODY_DEF_HPP


#include "dynamics/b3_pose.hpp"

#include <nlohmann/json.hpp>

namespace box3d {

    class b3BodyDef;

    class b3BodyDefInner;

    class b3BodyDefRigid;

    enum class b3BodyType;
}


enum class box3d::b3BodyType {

    b3_RIGID

};


class box3d::b3BodyDef {

    b3BodyDefInner* m_def;

    b3BodyType m_type = b3BodyType::b3_RIGID;

public:

    explicit b3BodyDef(b3BodyDefInner* def, b3BodyType type = b3BodyType::b3_RIGID) {
        m_def = def;
        m_type = type;
    }

    b3BodyDefInner* get_inner_def() const {
        return m_def;
    }

    b3BodyType get_type() const {
        return m_type;
    }

    ~b3BodyDef();

    static b3BodyDef create_body_definition(const nlohmann::json& json);

};


class box3d::b3BodyDefInner {

public:

    virtual ~b3BodyDefInner() = default;

};


class box3d::b3BodyDefRigid: public b3BodyDefInner {

    friend class b3BodyRigid;

    /**
     * @brief The density of the rigid body.
     * Unit: kg/m^3
     */
    double m_density = 1.0;

    b3PoseD m_init_pose = b3PoseD::zero();

    b3PoseD m_init_velocity = b3PoseD::zero();

public:

    b3BodyDefRigid() = default;

    explicit b3BodyDefRigid(const nlohmann::json& json);

    ~b3BodyDefRigid() override = default;

    static b3BodyDefRigid create_definition(const nlohmann::json& json);


};

#endif //BOX3D_B3_BODY_DEF_HPP
