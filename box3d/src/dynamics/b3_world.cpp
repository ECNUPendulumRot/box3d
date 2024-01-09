
#include "dynamics/b3_world.hpp"

#include "common/b3_allocator.hpp"
#include "common/b3_time_step.hpp"

box3d::b3World::b3World():
    m_body_list(nullptr), m_body_count(0),
    m_shape_list(nullptr), m_shape_count(0)
{
    ;
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

    while (body != nullptr) {

        auto rigid_pose = body->m_xf;
        auto rigid_velocity = body->m_velocity;

        rigid_velocity.set_linear(rigid_velocity.linear() + m_gravity * body->m_inv_mass * delta_t);

        rigid_pose.set_linear(rigid_pose.linear() + rigid_velocity.linear() * delta_t);

        body->m_xf = rigid_pose;
        body->m_velocity = rigid_velocity;

        body = body->next();
    }
}


box3d::b3Body *box3d::b3World::create_body(const box3d::b3BodyDef &def)
{
    b3_assert(def.m_type != b3BodyType::b3_type_not_defined);

    // allocate memory for the body
    void* memory = b3_alloc(sizeof (b3Body));
    auto* body = new(memory) b3Body(def);

    body->set_world(this);

    // add to the double linked list
    body->m_prev = nullptr;
    body->m_next = this->m_body_list;

    if (m_body_list) {
        m_body_list->m_prev = body;
    }
    m_body_list = body;
    ++m_body_count;

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

void box3d::b3World::step(double dt, int32 velocity_iterations, int32 position_iterations)
{
    // ff new fixtures were added, we need to find the new contacts.
    if (m_new_contacts) {
        m_contact_manager.find_new_contact();
        m_new_contacts = false;
    }

    b3TimeStep step;
    step.m_dt = dt;
    step.m_velocity_iterations = velocity_iterations;
    step.m_position_iterations = position_iterations;

    step.m_inv_dt = dt > 0.0 ? 1.0 / dt: 0.0;

}








