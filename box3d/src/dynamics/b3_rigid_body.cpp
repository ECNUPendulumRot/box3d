
#include "dynamics/b3_rigid_body.hpp"


box3d::b3RigidBody::b3RigidBody():
    b3Body(),
    m_pose(b3PoseD()),
    m_velocity(b3PoseD()),
    m_density(1.0),
    m_CoM(b3PoseD()),
    m_Inertia(b3Inertia())
{
    ;
}


box3d::b3RigidBody::b3RigidBody(const std::string &obj_file_name):
    b3Body(obj_file_name),
    m_density(1.0),
    m_CoM(b3PoseD()),
    m_Inertia(b3Inertia())

{
    m_pose.set_linear(b3Vector3d::zero());
    m_pose.set_angular(b3Vector3d::zero());

    m_velocity.set_linear(b3Vector3d::zero());
    m_velocity.set_angular(b3Vector3d::zero());

    compute_mass_properties();
}


box3d::b3RigidBody::b3RigidBody(const box3d::b3BodyDef &body_def):
    m_volume(0.0),
    m_CoM(b3PoseD()),
    m_Inertia(b3Inertia())
{
    auto def = (b3BodyDefRigid*) body_def.get_inner_def();

    m_pose = def->m_init_pose;
    m_velocity = def->m_init_velocity;

    m_density = def->m_density;

}


bool box3d::b3RigidBody::compute_mass_properties() {

    b3Mesh* mesh = this->mesh();

    if (mesh == nullptr)
        return false;

    // The m_CoM is relative to the origin of the obj file
    mesh->mesh_properties(m_volume, m_CoM, m_Inertia);

    mesh->recenter(m_CoM);

    m_inv_mass = m_volume * m_density;

    // TODO: check this inv mass formulation
    m_inv_mass = m_inv_mass == 0 ? 0.0 : m_inv_mass;

    m_Inertia.set_relative_pose(&m_pose);

    return true;
}



