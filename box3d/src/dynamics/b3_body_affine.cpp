
#include "dynamics/b3_body_affine.hpp"


box3d::b3BodyAffine::b3BodyAffine()
{
    m_density = 1;

    m_q.setZero();
    m_q[3]  = 1.0;
    m_q[7]  = 1.0;
    m_q[11] = 1.0;

    m_M.setZero();
}


void box3d::b3BodyAffine::set_mesh(box3d::b3Mesh *mesh)
{
    b3Body::set_mesh(mesh);

    compute_mass_properties();

    // TODO: check whether the following line is necessary
    // mesh->set_relative_pose(&m_pose);
}


bool box3d::b3BodyAffine::compute_mass_properties()
{
    b3Mesh* mesh = this->mesh();

    if (mesh == nullptr)
        return false;

    // TODO: check this assumption.
    // In Affine body dynamics, the volume and Inertia does not maintain.
    double volume; b3Inertia Inertia; b3PoseD CoM;
    mesh->mesh_properties(volume, CoM, Inertia);

    mesh->recenter(CoM);

    compute_jacobian_integral(volume, Inertia, CoM);

    return true;
}


void box3d::b3BodyAffine::compute_jacobian_integral(double volume, const b3Inertia& Inertia, const b3PoseD& CoM)
{
    b3Matrix3d inertia = Inertia.get_inertia_matrix();

    double y2z2 = inertia(0, 0);
    double x2z2 = inertia(1, 1);
    double x2y2 = inertia(2, 2);

    // Integral of 1, x, y, z.
    b3Vector3d xyz = CoM.linear();
    xyz *= volume;
    double c1 = volume;
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    // What we need is x2, y2, z2, xy, xz, yz
    double y2 = (x2y2 + y2z2 -x2z2) / 2.0;
    double z2 = y2z2 - y2;
    double x2 = x2z2 - z2;

    // xy, xz, yz
    double xy = inertia(0, 1);
    double xz = inertia(0, 2);
    double yz = inertia(1, 2);

    Eigen::Matrix3d xyzm;
    xyzm.setZero();
    xyzm.col(0) = Eigen::Vector3d(x, y, z);

    Eigen::Matrix3d pseudo_inertia;
    pseudo_inertia << x2, xy, xz,
                      xy, y2, yz,
                      xz, yz, z2;

    m_M.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    for (int i = 3; i < 9; i += 3) {
        m_M.block(0, i, 3, 3) = xyzm.transpose();
        m_M.block(3, 0, 3, 3) = xyzm;
        m_M.block(i, i, 3, 3) = pseudo_inertia;
    }

    m_M *= m_density;
}

box3d::b3BodyAffine::b3BodyAffine(const box3d::b3BodyDef &body_def)
{
    auto def = (b3BodyDefAffine*) body_def.get_inner_def();

    m_stiffness = def->m_stiffness;
    m_density = def->m_density;

    m_pose = body_def.m_init_pose;
    m_velocity = body_def.m_init_velocity;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////


box3d::b3BodyDefAffine::b3BodyDefAffine(double stiffness, double density)
{
    m_stiffness = stiffness;
    m_density = density;
}


box3d::b3BodyDef box3d::b3BodyDefAffine::create_definition(double stiffness, double density)
{
    void* memory = b3_alloc(sizeof(b3BodyDefAffine));

    b3BodyDefAffine* rigid_def =  new(memory) b3BodyDefAffine(stiffness, density);

    return b3BodyDef(rigid_def, b3BodyType::b3_RIGID);
}
