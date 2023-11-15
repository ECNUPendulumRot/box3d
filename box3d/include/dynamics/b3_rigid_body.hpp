
#ifndef BOX3D_B3_RIGID_BODY_HPP
#define BOX3D_B3_RIGID_BODY_HPP

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_pose.hpp"
#include "dynamics/b3_inertia.hpp"
#include "dynamics/b3_body_def.hpp"

namespace box3d {

    class b3RigidBody;
}


class box3d::b3RigidBody: public b3Body {

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

public:

    /**
     * @brief Construct a new b3RigidBody object
     */
    b3RigidBody();

    /**
     * @brief Construct a new b3RigidBody object
     * @param obj_file_name: path to .obj file
     */
    explicit b3RigidBody(const std::string& obj_file_name);

    explicit b3RigidBody(const b3BodyDef& body_def);

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

    // TODO: delete test function
    void test_step_by_force(double time_step) override {
        m_velocity.set_linear(m_velocity.linear() + m_force * time_step);
        m_pose.set_linear(m_pose.linear() + m_velocity.linear() * time_step);

        mesh()->transform();
    }

    void set_mesh(b3Mesh* mesh) override {

        b3Body::set_mesh(mesh);

        compute_mass_properties();

        mesh->set_relative_pose(&m_pose);

        mesh->transform();
    }

private:

    /**
     * @brief Compute the mass properties of the rigid body.
     * @return true on success, false on errors
     */
    bool compute_mass_properties();

};


#endif //BOX3D_B3_RIGID_BODY_HPP
