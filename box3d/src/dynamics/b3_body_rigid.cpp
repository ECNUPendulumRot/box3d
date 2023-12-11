
#include "dynamics/b3_body_rigid.hpp"


box3d::b3BodyRigid::b3BodyRigid():
    m_pose(b3PoseD()),
    m_velocity(b3PoseD()),
    m_CoM(b3PoseD()),
    m_Inertia(b3Inertia())
{
    ;
}


box3d::b3BodyRigid::b3BodyRigid(const box3d::b3BodyDef &body_def):
    m_volume(0.0),
    m_CoM(b3PoseD()),
    m_Inertia(b3Inertia())
{
    auto def = (b3BodyDefRigid*) body_def.get_inner_def();

    m_pose = body_def.m_init_pose;
    m_velocity = body_def.m_init_velocity;

    m_density = def->m_density;

}


void box3d::b3BodyRigid::set_mesh(box3d::b3Mesh *mesh)
{

    b3Body::set_mesh(mesh);

    compute_mass_properties();

    mesh->set_relative_pose(&m_pose);
}


bool box3d::b3BodyRigid::compute_mass_properties() {

    b3Mesh* mesh = this->mesh();

    if (mesh == nullptr)
        return false;

    // The m_CoM is relative to the origin of the obj file
    mesh->mesh_properties(m_volume, m_CoM, m_Inertia);

    mesh->recenter(m_CoM);

    m_inv_mass = m_volume * m_density;

    // TODO: check this inv mass formulation
    m_inv_mass = m_inv_mass == 0 ? 0.0 : 1.0 / m_inv_mass;

    m_Inertia.set_relative_pose(&m_pose);

    return true;
}


///////////////////////////////////////////////////////////////////////////////////


box3d::b3BodyDefRigid::b3BodyDefRigid(double density)
{
    m_density = density;
}


box3d::b3BodyDef box3d::b3BodyDefRigid::create_definition(double density)
{
    void* memory = b3_alloc(sizeof(b3BodyDefRigid));

    b3BodyDefRigid* rigid_def =  new(memory) b3BodyDefRigid(density);

    return b3BodyDef(rigid_def, b3BodyType::b3_RIGID);
}





