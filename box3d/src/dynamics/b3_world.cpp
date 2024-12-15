
#include <iostream>
#include <iomanip>
#include "dynamics/b3_world.hpp"

#include "common/b3_time_step.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"


#include "solver/b3_contact_solver_substep.hpp"
#include "common/b3_draw.hpp"

#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "solver/b3_solver.hpp"

// #include "collision/b3_time_of_impact.hpp"


b3World::b3World():
    m_body_list(nullptr), m_body_count(0),
    m_shape_list(nullptr), m_shape_count(0)
{
    m_contact_manager.set_block_allocator(&m_block_allocator);
}


b3World::b3World(const b3Vec3r &gravity):b3World()
{
    m_gravity = gravity;
    m_contact_manager.set_block_allocator(&m_block_allocator);
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
    body->apply_gravity(m_gravity);

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


void b3World::step(int32 hw, int32 velocity_iterations, int32 position_iterations, b3Solver* solver)
{

    // when new fixtures were added, we need to find the new contacts.
    if (m_new_contacts) {
        m_contact_manager.find_new_contact();
        m_new_contacts = false;
    }

    b3TimeStep step;
    step.m_hw = real(hw);
    step.m_dt = hw == 0 ? 0.0 : real(1.0)/real(hw);
    step.m_velocity_iterations = velocity_iterations;
    step.m_position_iterations = position_iterations;
    step.m_integral_method = e_implicit;

    // update contacts, aabb updates, when aabb not overlapping, delete the contact,
    // otherwise, update the contact manifold.
    m_contact_manager.collide();

    // generate islands, and solve them.
    if (m_step_complete && step.m_dt > 0.0) {
        solve(step, solver);
    }

    if (m_continuous_physics && step.m_dt > 0.0) {
        //solve_toi(step);
    }
}


void b3World::debug_draw() {

    if (m_debug_draw == nullptr) {
        return;
    }

    uint32 flags = m_debug_draw->get_flags();

    if (flags & b3Draw::e_shape_bit) {
        for (b3Body *body = m_body_list; body; body = body->next()) {

            b3Transr xf(body->get_position(), body->get_quaternion());

            for (b3Fixture *f = body->get_fixture_list(); f; f = f->get_next()) {
                b3Color c;
                if (f->get_body()->is_awake()) {
                    c = b3Color(1.0f, 1.0f, 0.0f);
                } else {
                    c = b3Color(0.5f, 0.5f, 0.5f);
                }
                draw_shape(f, xf, c);
            }
        }
    }
}


void b3World::draw_shape(b3Fixture *fixture, const b3Transr&xf, const b3Color &color) {

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
        default:
            break;
    }
}


void b3World::set_allow_sleeping(bool flag) {

    if (flag == m_allow_sleep) {
        return;
    }

    m_allow_sleep = flag;
    if (m_allow_sleep == false) {
        for (b3Body* b = m_body_list; b; b = b->m_next) {
            b->set_awake(true);
        }
    }
}


void b3World::integrate_velocity(b3BodySim* body_sims, real dt) {

    for(int32 i = 0; i < m_body_count; ++i) {

        b3BodySim* b = body_sims + i;

        b3Vec3r v = b->v;
        b3Vec3r w = b->w;

        if (b->inv_mass == 0) {
            continue;
        }

        v += dt * (m_gravity + b->inv_mass * (b->ext_force ));
        w += dt * b->inv_I * b->ext_torque;

        b->v = v;
        b->w = w;
    }
}


void b3World::integrate_position(b3BodySim* body_sims, real dt) {

    for (int32 i = 0; i < m_body_count; ++i) {
        b3BodySim* b = body_sims + i;

        b3Vec3r p = b->p;
        b3Quatr q = b->q;

        const b3Vec3r& v = b->v;
        const b3Vec3r& w = b->w;

        p = p + v * dt;
        q = q + real(0.5) * dt * b3Quatr(0, w) * q;
        q.normalize();

        b->p = p;
        b->q = q;
    }
}


void b3World::write_back_status(b3BodySim *body_sims) {
    for (int32 i = 0; i < m_body_count; ++i) {
        b3BodySim* b = body_sims + i;

        b3Body* body = b->body;

        body->m_p = b->p;
        body->m_q = b->q;
        body->m_v = b->v;
        body->m_w = b->w;

        std::cout << std::fixed << std::setprecision(7);
        std::cout << "velocity: " << b->v[0] <<  " " << b->v[1] << " " << b->v[2] << std::endl;
    }
}


void b3World::solve(b3TimeStep& step, b3Solver* solver)
{
    // clear all island flag.
    for (b3Body *body = m_body_list; body; body = body->next()) {
        body->m_flags &= ~b3Body::e_island_flag;
        body->m_flags &= ~b3Body::e_static_island_flag;
        body->m_flags &= ~b3Body::e_normal_island_flag;
    }

    for (b3Contact *contact = m_contact_manager.m_contact_list; contact; contact = contact->next()) {
        contact->unset_flag(b3Contact::e_island_flag);
    }

    for (b3Contact *contact = m_contact_manager.m_static_contact_list; contact; contact = contact->next()) {
        contact->unset_flag(b3Contact::e_island_flag);
    }

    // create body structure for simulation
    void* mem = m_block_allocator.allocate(m_body_count * sizeof(b3BodySim));
    b3BodySim* body_sims = new (mem) b3BodySim[m_body_count];;

    int32 index = 0;
    for (b3Body *body = m_body_list; body; body = body->next()) {
        body->generate_sim_body(body_sims[index]);
        index++;
    }

    solver->solve(step, body_sims);

    for (b3Body *b = m_body_list; b; b = b->m_next) {

        if (b->m_type == b3BodyType::b3_static_body) {
            continue;
        }

        // If a body was not in an island then it did not move.
        if ((b->m_flags & b3Body::e_normal_island_flag) & (b->m_flags & b3Body::e_static_island_flag) == 0) {
            continue;
        }
        // Update fixtures(for broad-phase).
        b->synchronize_fixtures();
    }

    m_block_allocator.free(body_sims, m_body_count * sizeof(b3BodySim));

    // Look for new contacts
    m_contact_manager.find_new_contact();
}


void b3World::solve_toi(const b3TimeStep &step)
{
    b3Island island(&m_block_allocator, m_body_count,
                    m_contact_manager.get_contact_count());

    if (m_step_complete) {

        for (b3Body *body = m_body_list; body; body = body->next()) {
            body->m_flags &= ~b3Body::e_island_flag;
            body->m_sweep.alpha0 = 0.0f;
        }

        for (b3Contact* c = m_contact_manager.m_contact_list; c; c = c->next()) {
            c->m_flags &= ~(b3Contact::e_toi_flag | b3Contact::e_island_flag);
            c->m_toi_count = 0;
            c->m_toi = 1.0f;
        }
    }

    for (;;) {

        b3Contact* min_contact = nullptr;
        real min_alpha = 1.0;

        for (b3Contact* c = m_contact_manager.m_contact_list; c; c = c->next()) {
            if (c->m_toi_count > b3_max_sub_steps) {
                continue;
            }

            float alpha = 1.0;
            if (c->m_flags & b3Contact::e_toi_flag) {
                alpha = c->m_toi;
            } else {
                b3Fixture* f_a = c->get_fixture_a();
                b3Fixture* f_b = c->get_fixture_b();

                b3Body* b_a = f_a->get_body();
                b3Body* b_b = f_b->get_body();

                b3BodyType type_a = b_a->get_type();
                b3BodyType type_b = b_b->get_type();

                bool active_a = b_a->is_awake() && type_a != b3BodyType::b3_static_body;
                bool active_b = b_b->is_awake() && type_b != b3BodyType::b3_static_body;

                if (active_a == false && active_b == false) {
                    continue;
                }

                bool collide_a = type_a != b3BodyType::b3_dynamic_body;
                bool collide_b = type_b != b3BodyType::b3_dynamic_body;

                if (collide_a == false && collide_b == false) {
                    continue;
                }

                real alpha0 = b_a->m_sweep.alpha0;

                if (b_a->m_sweep.alpha0 < b_b->m_sweep.alpha0) {
                    alpha0 = b_b->m_sweep.alpha0;
                    b_a->m_sweep.advance(alpha0);
                } else if (b_b->m_sweep.alpha0 < b_a->m_sweep.alpha0) {
                    alpha0 = b_a->m_sweep.alpha0;
                    b_b->m_sweep.advance(alpha0);
                }

                b3_assert(alpha0 < 1.0f);

                int32 index_a = c->get_child_index_a();
                int32 index_b = c->get_child_index_b();

                // b3TOIInput input;
                // input.proxy_a.set(f_a->get_shape(), index_a);
                // input.proxy_b.set(f_b->get_shape(), index_b);
                // input.sweep_a = b_a->m_sweep;
                // input.sweep_b = b_b->m_sweep;
                // input.t_max = 1.0;

                //b3TOIOutput output;

            }
        }
    }
}




