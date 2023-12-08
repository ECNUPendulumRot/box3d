
#ifndef BOX3D_B3_BODY_RIGID_HPP
#define BOX3D_B3_BODY_RIGID_HPP

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_pose.hpp"
#include "dynamics/b3_inertia.hpp"
#include "dynamics/b3_body_def.hpp"
#include "utils/b3_log.hpp"

namespace box3d {

    class b3BodyRigid;
}


class box3d::b3BodyRigid: public b3Body {

    friend class b3World;

    /**
     * The density of the rigid body.
     */
    double m_density;

    /**
     * The volume of the rigid body.
     */
    double m_volume = 0.0;

    /**
     * The mass of the rigid body.
     */
    double m_inv_mass = 0.0;

    /**
     * The center of mass of the rigid body.
     * Note that this position is wrt the origin in the OBJ.
     * We assume the orientation of origin in OBJ is same to the world frame
     */
    b3PoseD m_CoM;

    /**
     * The inertia matrix of the rigid body.
     */
    b3Inertia m_Inertia;

    /**
     * @brief Pose of the body of CoM wrt the world frame
     */
    b3PoseD m_pose;

    /**
     * @brief Velocity of CoM the body
     */
    b3PoseD m_velocity;

    b3Vector3d m_force;

public:

    /**
     * @brief Construct a new b3BodyRigid object
     */
    b3BodyRigid();

    virtual ~b3BodyRigid() = default;

    explicit b3BodyRigid(const b3BodyDef& body_def);

    /**
     * @brief Get volume, center of geometry and inertia from the mesh
     * @param density
     */
    inline void set_density(const double& density){
        m_density = density;
    };

    /**
     * @brief Get volume, center of geometry and inertia from the mesh
     * @param position
     */
    void set_position(const b3Vector3d& position){
        m_pose.set_linear(position);
    }

    void set_pose(const b3PoseD& pose){
        m_pose = pose;
    }

    void set_velocity(const b3PoseD& velocity){
        m_velocity = velocity;
    }

    b3PoseD get_pose() const {
        return m_pose;
    }

    b3PoseD get_velocity() const {
        return m_velocity;
    }

    void set_mesh(b3Mesh* mesh) override {

        b3Body::set_mesh(mesh);

        compute_mass_properties();

        mesh->set_relative_pose(&m_pose);
    }

    void apply_central_force(const b3Vector3d& force) {
        m_force += force;
    }

private:

    /**
     * @brief Compute the mass properties of the rigid body.
     * @return true on success, false on errors
     */
    bool compute_mass_properties();

};


#endif //BOX3D_B3_BODY_RIGID_HPP
