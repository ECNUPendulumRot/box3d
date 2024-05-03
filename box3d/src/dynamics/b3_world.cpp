
#include "dynamics/b3_world.hpp"

#include "common/b3_time_step.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "solver/b3_solver.hpp"
#include "common/b3_draw.hpp"

#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"

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


void b3World::step(real dt, int32 velocity_iterations, int32 position_iterations)
{

    // when new fixtures were added, we need to find the new contacts.
    if (m_new_contacts) {
        m_contact_manager.find_new_contact();
        m_new_contacts = false;
    }

    b3TimeStep step;
    step.m_dt = dt;
    step.m_velocity_iterations = velocity_iterations;
    step.m_position_iterations = position_iterations;
    step.m_integral_method = e_implicit;
    step.m_inv_dt = dt > 0.0 ? real(1.0) / dt : real(0.0);

    // update contacts, aabb updates, when aabb not overlapping, delete the contact,
    // otherwise, update the contact manifold.
    m_contact_manager.collide();

    // generate islands, and solve them.
    solve(step);
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
                                        m_contact_manager.get_contact_count());

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
        }

        // solve the constraints
        b3Solver solver(&m_block_allocator, island, &step);
        solver.solve();

        // Post solve cleanup.
        for(int32 i = 0; i < island->get_body_count(); ++i) {
            // Allow static bodies to participate in other islands
            b3Body* body = island->get_body(i);
            if(body->m_type == b3BodyType::b3_static_body) {
                body->m_flags &= ~b3Body::e_island_flag;
            }
        }
        // clear all bodies and contacts count, so we can reuse the island for the next island.
        island->clear();
    }

    // Free the stack and island memory.
    m_block_allocator.free(stack, m_body_count * sizeof(b3Body*));
    m_block_allocator.free(island, sizeof(b3Island));

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


void b3World::debug_draw() {

    if (m_debug_draw == nullptr) {
        return;
    }

    uint32 flags = m_debug_draw->get_flags();

    if (flags & b3Draw::e_shape_bit) {
        for (b3Body *body = m_body_list; body; body = body->next()) {

            b3Transform xf(body->get_position(), body->get_quaternion());

            for (b3Fixture *f = body->get_fixture_list(); f; f = f->get_next()) {
                draw_shape(f, xf, b3Color(1.0f, 1.0f, 0.0f));
            }
        }
    }
}

void b3World::draw_shape(b3Fixture *fixture, const b3Transformr&xf, const b3Color &color) {

    switch (fixture->get_shape_type()) {
        case b3ShapeType::e_cube: {
            b3CubeShape* cube = (b3CubeShape*)fixture->get_shape();
            b3Vec3r vertices[8];
            b3Vec3r normals[6];
            for (int32 i = 0; i < 8; ++i) {
                vertices[i] = xf.transform(cube->m_vertices[i]);
            }

            for (int32 i = 0; i < 6; ++i) {
                normals[i] = xf.rotate(cube->m_normals[i]);
            }

            m_debug_draw->draw_box(cube->m_edges, cube->m_faces, normals, vertices, color);
            break;
        }
        case b3ShapeType::e_plane: {
            b3PlaneShape* plane = (b3PlaneShape*)fixture->get_shape();
            m_debug_draw->draw_plane(xf, plane->m_half_width, plane->m_half_length, color);
        }

        case b3ShapeType::e_sphere: {
            b3SphereShape* sphere = (b3SphereShape*)fixture->get_shape();
            m_debug_draw->draw_sphere(xf, sphere->get_radius(), color);
        }
        default:
            break;
    }
}



