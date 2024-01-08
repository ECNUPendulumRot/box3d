
#include "dynamics/b3_world.hpp"
#include "dynamics/b3_body_rigid.hpp"
#include "dynamics/b3_body_affine.hpp"

#include "common/b3_allocator.hpp"

box3d::b3World::b3World():
    m_body_list(nullptr), m_body_count(0),
    m_shape_list(nullptr), m_shape_count(0)
{
}


box3d::b3World::~b3World()
{
    // TODO: think about how to destruct
    b3_free(m_shape_list);
    b3_free(m_body_list);
}


void box3d::b3World::test_step()
{
    b3Body* body = m_body_list;

    double delta_t  = 1.0 / m_hz;

    solve(delta_t);
}


void box3d::b3World::solve(double delta_t)
{
    b3Body* body = m_body_list;

    // TODO: remove rigid body for simplicity
    while (body != nullptr) {

        auto rigid_pose = body->get_pose();
        auto rigid_velocity = body->get_velocity();

        rigid_velocity.set_linear(rigid_velocity.linear() + m_gravity * body->m_inv_mass * delta_t);

        rigid_pose.set_linear(rigid_pose.linear() + rigid_velocity.linear() * delta_t);

        body->set_pose(rigid_pose);
        body->set_velocity(rigid_velocity);

        body = body->next();
    }
}


box3d::b3Body* box3d::b3World::create_rigid_body(const box3d::b3BodyDef& def)
{
    void* memory = b3_alloc(sizeof (b3BodyRigid));
    auto* body = new(memory) b3BodyRigid(def);

    body->set_world(this);
    body->set_next(m_body_list);
    m_body_list = body;
    ++m_body_count;

    return body;
}


box3d::b3Body *box3d::b3World::create_body(const box3d::b3BodyDef &def)
{
    b3Body* body;

    body = create_rigid_body(def);


    body->set_type(def.get_type());

    return body;
}


box3d::b3Shape *box3d::b3World::create_shape(const std::filesystem::path &file_path)
{
    std::string fs_string = file_path.string();

    void* memory = b3_alloc(sizeof(b3Shape));

    // TODO: implement creation of shape from world
    auto* shape = new(memory) b3Shape;

    shape->set_next(m_shape_list);
    m_shape_list = shape;
    m_shape_count++;

    return shape;
}


void box3d::b3World::clear()
{
    b3Shape* shape = m_shape_list;

    // Free all meshes
    while (shape != nullptr) {
        auto* next = shape->next();
        b3_free(shape);
        shape = next;
    }

    // Free all bodies
    b3Body* body = m_body_list;
    while (body != nullptr) {
        auto* next = body->next();
        b3_free(body);
        body = next;
    }
}








