
#ifndef BOX3D_B3_RIGID_BODY_HPP
#define BOX3D_B3_RIGID_BODY_HPP

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_pose.hpp"
#include "dynamics/b3_inertia.hpp"

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
    double m_volume;

    /**
     * The mass of the rigid body.
     */
    double m_mass;

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

    /**
     * @brief Get volume, center of geometry and inertia from the mesh
     * @param density
     */
    inline void set_density(const double& density){
        m_density = density;
    };

    void set_position(const b3Vector3d& position){
        m_pose.set_position(position);
    }

private:

    /**
     * @brief Compute the mass properties of the rigid body.
     * @return true on success, false on errors
     */
    bool compute_mass_properties();

};


#endif //BOX3D_B3_RIGID_BODY_HPP
