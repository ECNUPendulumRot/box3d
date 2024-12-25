
#include "dynamics/b3_world.hpp"

#include "common/b3_time_step.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "solver/b3_solver.hpp"
#include "solver/b3_solver_lxj.hpp"
#include "common/b3_draw.hpp"

#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cone_shape.hpp"

#include "dynamics/b3_transform_util.hpp"
#include "dynamics/constraint/b3_constraint_base.hpp"

#include "spdlog/spdlog.h"
#include "geometry/b3_cylinder_shape.hpp"
#include <iostream>

b3World::b3World():
    m_body_list(nullptr), m_body_count(0),
    m_shape_list(nullptr), m_shape_count(0)
{
    m_contact_manager.set_block_allocator(&m_block_allocator);

    void* mem = m_block_allocator.allocate(sizeof(b3Dispatcher));
    m_dispatcher = new (mem) b3Dispatcher(&m_block_allocator);
    // TODO: maybe not need this
    m_dispatcher_info.m_time_step = 1.0 / 60;
    m_dispatcher_info.m_step_count = 0;
}


b3World::b3World(const b3Vec3r &gravity):b3World()
{
    m_gravity = gravity;
}


b3World::~b3World()
{
    // TODO: think about how to destruct
}


b3Body *b3World::create_body(const b3BodyDef &def)
{
    b3_assert(def.m_type != b3BodyType::b3_type_not_defined);

    // allocate memory for the body
    void *mem = m_block_allocator.allocate(sizeof(b3Body));
    auto *body = new (mem) b3Body(def);

    body->set_world(this);
    body->set_gravity_scale(1.0);

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


void b3World::add_shape(b3Shape* shape)
{
    shape->set_next(m_shape_list);
    m_shape_list = shape;
    m_shape_count++;
}

void b3World::add_constraint(b3ConstraintBase* constraint) {

    b3Body* bodyA = constraint->get_bodyA();
    b3Body* bodyB = constraint->get_bodyB();

    b3_assert(bodyA != bodyB);

    if (bodyA->add_constraint(constraint) && bodyB->add_constraint(constraint)) {
        m_constraints.push_back(constraint);
    }
}

void b3World::remove_constraint(b3ConstraintBase* constraint) {
    b3Body* bodyA = constraint->get_bodyA();
    b3Body* bodyB = constraint->get_bodyB();

    if (bodyA->remove_constraint(constraint) && bodyB->remove_constraint(constraint)) {
        for (int i = 0; i < m_constraints.size(); i++) {
            if (m_constraints[i] == constraint) {
                m_constraints.erase(m_constraints.begin() + i);
                break;
            }
        }
    }
}

void b3World::clear()
{
    // TODO: Check this function
    // Free all bodies
    b3Body* body = m_body_list;
    while (body != nullptr) {
        auto* next = body->next();
        // b3_free(body);
        body->destroy_fixtures();
        // TODO: free contact and contact edge etc related to body (destroy a body)
        m_block_allocator.free(body, sizeof(b3Body));
        body = next;
    }
}


void b3World::step(real dt, int32 velocity_iterations, int32 position_iterations)
{

    if (dt > real(0.0)) {
        apply_gravity();
    }

    // when new fixtures were added, we need to find the new contacts.
    if (m_new_contacts) {
        m_contact_manager.find_new_contact();
        m_new_contacts = false;
    }

    b3TimeStep step;
    step.m_dt = dt;
    step.m_velocity_iterations = velocity_iterations;
    step.m_position_iterations = position_iterations;
    step.m_inv_dt = dt > 0.0 ? real(1.0) / dt : real(0.0);

    // update contacts, aabb updates, when aabb not overlapping, delete the contact,
    // otherwise, update the contact manifold.
    m_contact_manager.collide(m_dispatcher, m_dispatcher_info);

//    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
//        if (body->m_type != b3BodyType::b3_static_body) {
//            auto p = body->get_position();
//            auto q = body->get_quaternion();
//            auto v = body->get_linear_velocity();
//            auto w = body->get_angular_velocity();
//            std::cout << "body position: " << p.x << " " << p.y << " " << p.z << ", rotation: "
//                      << q.m_x << " " << q.m_y << " " << q.m_z << " " << q.m_w << std::endl;
//            std::cout << "linear velocity: " << v.x << " " << v.y << " " << v.z << ", angular velocity: "
//                      << w.x << " " << w.y << " " << w.z << std::endl;
//        }
//    }

    // generate islands, and solve them.
    solve(step);

    clear_forces();
}


void b3World::solve(b3TimeStep &step)
{
    // clear all island flag.
    for (b3Body *body = m_body_list; body; body = body->next()) {
        body->m_flags &= ~b3Body::e_island_flag;
    }
    for (b3Contact *contact = m_contact_manager.get_contact_list(); contact; contact = contact->next()) {
	    contact->unset_flag(b3Contact::e_island_flag);
    }

    void *mem = m_block_allocator.allocate(sizeof(b3Island));
    b3Island *island = new (mem) b3Island(&m_block_allocator, m_body_count,
                                        m_contact_manager.get_contact_count(), m_constraints.size());

    // build all islands
    mem = m_block_allocator.allocate(m_body_count * sizeof(b3Body *));
    b3Body **stack = new (mem) b3Body *;
    for (b3Body *body = m_body_list; body; body = body->next()) {
        if (body->m_flags & b3Body::e_island_flag) {
            continue;
        }

        int32 stack_count = 0;
        stack[stack_count++] = body;
        body->m_flags |= b3Body::e_island_flag;

	    // Perform a depth first search (DFS) on the constraint graph.
        while (stack_count > 0) {
            b3Body *b = stack[--stack_count];

            island->add_body(b);

            // search all contact connected to this body
            for (b3ContactEdge *ce = b->get_contact_list(); ce; ce = ce->m_next) {
                b3Contact *contact = ce->m_contact;

                // Has this contact already has been added to this island ?
                if (contact->test_flag(b3Contact::e_island_flag)) {
                    continue;
                }
                if (!contact->test_flag(b3Contact::e_touching_flag)) {
                    continue;
                }

                island->add_contact(contact);
                contact->set_flag(b3Contact::e_island_flag);

                b3Body *other = ce->m_other;

                // Was the other body already has been added to this island ?
                if (other->m_flags & b3Body::e_island_flag) {
                    continue;
                }

                stack[stack_count++] = other;
                other->m_flags |= b3Body::e_island_flag;
	        }

            // search all constraints connected to this body
            for (int i = 0; i < m_constraints.size(); i++) {
                b3ConstraintBase* constraint = m_constraints[i];

                if (constraint->test_flag(b3ConstraintBase::e_island)) {
                    continue;
                }

                b3Body* bodyA = constraint->get_bodyA();
                b3Body* bodyB = constraint->get_bodyB();

                if (bodyA != b && bodyB != b) {
                    continue;
                }

                island->add_constraint(constraint);
                constraint->add_flag(b3ConstraintBase::e_island);

                if (!(bodyA->m_flags & b3Body::e_island_flag)) {
                    stack[stack_count++] = bodyA;
                    bodyA->m_flags |= b3Body::e_island_flag;
                }
                if (!(bodyB->m_flags & b3Body::e_island_flag)) {
                    stack[stack_count++] = bodyB;
                    bodyB->m_flags |= b3Body::e_island_flag;
                }
            }
        }

        // TODO：solve the constraints
//        b3Solver solver;
//        solver.init(&m_block_allocator, island, &step);
//        solver.solve();
        b3SolverLxj solver;
        solver.solve_group(island, &step);
        // Post solve cleanup.
        for(int32 i = 0; i < island->get_body_count(); ++i) {
            // Allow static bodies to participate in other islands
            b3Body* body = island->get_body(i);
            if(body->m_type == b3BodyType::b3_static_body) {
                body->m_flags &= ~b3Body::e_island_flag;
            }
        }
        for (int32 i = 0; i < m_constraints.size(); i++) {
            m_constraints[i]->remove_flag(b3ConstraintBase::e_island);
        }
        // clear all bodies and contacts count, so we can reuse the island for the next island.
        island->clear();
    }

    // Free the stack and island memory.
    m_block_allocator.free(stack, m_body_count * sizeof(b3Body*));
    m_block_allocator.free(island, sizeof(b3Island));

    // TODO: do all contacts solver, so integrate all body transform.
    integrate_transforms_internal(step);

    // TODO: synchronize ?
    for (b3Body *b = m_body_list; b; b = b->m_next) {
        // If a body was not in an island then it did not move.
        if ((b->m_flags & b3Body::e_island_flag) == 0) {
            continue;
        }
        if (b->m_type == b3BodyType::b3_static_body) {
            continue;
        }

        // Update fixtures(for broad-phase).
        b->synchronize_fixtures();
    }
    // Look for new contacts
    m_contact_manager.find_new_contact();
}


void b3World::integrate_transforms_internal(b3TimeStep &step) {

//    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
//        if (body->m_type != b3BodyType::b3_static_body) {
//            auto p = body->get_position();
//            auto q = body->get_quaternion();
//            auto v = body->get_linear_velocity();
//            auto w = body->get_angular_velocity();
//            std::cout << "积分前 body position: " << p.x << " " << p.y << " " << p.z << ", rotation: "
//                      << q.m_x << " " << q.m_y << " " << q.m_z << " " << q.m_w << std::endl;
//            std::cout << "linear velocity: " << v.x << " " << v.y << " " << v.z << ", angular velocity: "
//                      << w.x << " " << w.y << " " << w.z << std::endl;
//        }
//    }

    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
        if (body->m_type != b3BodyType::b3_static_body) {
            b3Transformr world_transform = body->get_world_transform();
            b3Transformr predicted_transform;
            b3TransformUtils::integrate_transform(world_transform, body->get_linear_velocity(), body->get_angular_velocity(), step.m_dt, predicted_transform);
            body->proceed_to_transform(predicted_transform);
            body->set_position(predicted_transform.position());
            b3Quaternionr q;
            predicted_transform.rotation_matrix().get_rotation(q);
            body->set_quaternion(q);
        }
    }
    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
        if (body->m_type != b3BodyType::b3_static_body) {
            auto p = body->get_position();
            auto q = body->get_quaternion();
            auto v = body->get_linear_velocity();
            auto w = body->get_angular_velocity();
//            std::cout << "body position: " << p.x << " " << p.y << " " << p.z << std::endl;
//            std::cout << "body position: " << p.x << " " << p.y << " " << p.z << ", rotation: "
//                      << q.m_x << " " << q.m_y << " " << q.m_z << " " << q.m_w << std::endl;
            std::cout << "linear velocity: " << v.x << " " << v.y << " " << v.z << ", angular velocity: "
                      << w.x << " " << w.y << " " << w.z << std::endl;
        }
    }
}

void b3World::apply_gravity() {
    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
        body->apply_gravity();
    }
}

void b3World::clear_forces() {
    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
        body->clear_forces();
    }
}

void b3World::debug_draw() {

    if (m_debug_draw == nullptr) {
        return;
    }

    uint32 flags = m_debug_draw->get_flags();

    if (flags & b3Draw::e_shape_bit) {
        for (b3Body *body = m_body_list; body; body = body->next()) {

            b3Transform xf = body->get_world_transform();

            for (b3Fixture *f = body->get_fixture_list(); f; f = f->get_next()) {
                b3Transformr world_transform = f->get_world_transform(xf);
                draw_shape(f, world_transform, b3Color(1.0f, 1.0f, 0.0f));
            }
        }
    }

//    for (b3Body* body = m_body_list; body != nullptr; body = body->next()) {
//        b3Transformr xf = body->get_world_transform();
//        b3AABB aabb;
//        body->get_fixture_list()->get_shape()->get_bound_aabb(&aabb, xf, 0);
//
//        b3Vec3r center = aabb.center();
//        b3Vec3r half_extent = (aabb.max() - aabb.min()) * 0.5;
//        b3CubeShape aabbShape;
//        aabbShape.set_as_box(half_extent.x, half_extent.y, half_extent.z);
//        b3Transformr aabb_xf;
//        aabb_xf.set_position(center);
//        m_debug_draw->draw_box(&aabbShape, aabb_xf, b3Color(1.0f, 0.0f, 0.0f));
//    }
}


void b3World::draw_shape(b3Fixture *fixture, const b3Transformr&xf, const b3Color &color) {

    switch (fixture->get_shape_type()) {
        case b3ShapeType::e_cube: {
            b3CubeShape* cube = (b3CubeShape*)fixture->get_shape();
            m_debug_draw->draw_box(cube, xf, color);
            break;
        }
        case b3ShapeType::e_plane: {
            b3PlaneShape* plane = (b3PlaneShape*)fixture->get_shape();
            m_debug_draw->draw_plane(plane, xf, color);
            break;
        }

        case b3ShapeType::e_sphere: {
            b3SphereShape* sphere = (b3SphereShape*)fixture->get_shape();
            m_debug_draw->draw_sphere(sphere, xf, color);
            break;
        }

        case b3ShapeType::e_cone: {
            b3ConeShape* shape = (b3ConeShape*)fixture->get_shape();
            m_debug_draw->draw_cone(shape, xf, b3Color(0.f, 1.f, 0.f));
            break;
        }

        case b3ShapeType::e_cylinder: {
            b3CylinderShape* shape = (b3CylinderShape*)fixture->get_shape();
            m_debug_draw->draw_cylinder(shape, xf, b3Color(0.f, 1.f, 0.f));
            break;
        }

        default:
            break;
    }
}




