
#include "dynamics/b3_world.hpp"
#include "dynamics/b3_rigid_body.hpp"

#include "common/b3_allocator.hpp"

box3d::b3World::b3World():
    m_rigid_body_list(nullptr), m_body_count(0),
    m_mesh_list(nullptr), m_mesh_count(0)
{
    ;
}


box3d::b3World::~b3World()
{
    // TODO: think about how to destruct
}


void box3d::b3World::test_step()
{
    b3Body* body = m_rigid_body_list;

    double delta_t  = 1.0 / m_hz;

    solve_rigid(delta_t);

}


void box3d::b3World::solve_rigid(double delta_t)
{
    b3Body* body = m_rigid_body_list;

    while (body != nullptr) {

        auto* rigid_body = (b3BodyRigid*) body;

        auto rigid_pose = rigid_body->get_pose();
        auto rigid_velocity = rigid_body->get_velocity();

        rigid_velocity.set_linear(rigid_velocity.linear() + m_gravity * rigid_body->m_inv_mass * delta_t);

        rigid_pose.set_linear(rigid_pose.linear() + rigid_velocity.linear() * delta_t);

        rigid_body->set_pose(rigid_pose);
        rigid_body->set_velocity(rigid_velocity);

        body = body->next();
    }
}


box3d::b3Body* box3d::b3World::create_rigid_body(const box3d::b3BodyDef& def)
{
    void* memory = b3_alloc(sizeof (b3BodyRigid));
    auto* body = new(memory) b3BodyRigid(def);

    body->set_world(this);
    body->set_next(m_rigid_body_list);
    m_rigid_body_list = body;
    ++m_body_count;

    return body;
}


box3d::b3Body *box3d::b3World::create_body(const box3d::b3BodyDef &def)
{
    switch (def.get_type()) {
        case b3BodyType::b3_RIGID:
            return create_rigid_body(def);
    }
}


box3d::b3Mesh *box3d::b3World::create_mesh(const std::filesystem::path &file_path)
{
    std::string fs_string = file_path.string();

    void* memory = b3_alloc(sizeof(b3Mesh));

    auto* mesh = new(memory) b3Mesh(fs_string);

    mesh->set_next(mesh);
    m_mesh_list = mesh;
    m_mesh_count++;

    return mesh;
}


void box3d::b3World::clear()
{
    b3Mesh* mesh = m_mesh_list;

    // Free all meshes
    while (mesh != nullptr) {
        auto* next = mesh->next();
        b3_free(mesh);
        mesh = next;
    }

    // Free all bodies
    b3Body* body = m_rigid_body_list;
    while (body != nullptr) {
        auto* next = body->next();
        b3_free(body);
        body = next;
    }
}






