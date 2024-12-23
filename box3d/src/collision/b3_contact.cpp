
#include "collision/b3_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_sphere_contact.hpp"
#include "collision/b3_cube_contact.hpp"
#include "collision/b3_cube_sphere_contact.hpp"
#include "collision/b3_plane_sphere_contact.hpp"
#include "collision/b3_plane_cube_contact.hpp"
#include "collision/b3_plane_cone_contact.hpp"
#include "collision/b3_cube_cone_contact.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_dispatcher.hpp"
#include "collision/algorithm/b3_collision_algorithm.hpp"
#include "common/b3_world_callback.hpp"

b3ContactRegister b3Contact::s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];
bool b3Contact::s_initialized = false;


// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
static real b3_mix_friction(real friction1, real friction2) {
    return b3_sqrt(friction1 * friction2);
}


// Restitution mixing law. The idea is to allow for anything to bounce off an inelastic surface.
static real b3_mix_restitution(real restitution1, real restitution2) {
    return b3_max(restitution1, restitution2);
}


static real b3_mix_rolling_friction(real rolling_friction1, real rolling_friction2) {
    return b3_max(rolling_friction1, rolling_friction2);
}


static real b3_mix_spinning_friction(real spinning_friction1, real spinning_friction2) {
    return b3_max(spinning_friction1, spinning_friction2);
}


void b3Contact::initialize_registers()
{
    add_type(b3SphereContact::create, b3SphereContact::destroy, b3ShapeType::e_sphere, b3ShapeType::e_sphere);
    add_type(b3CubeContact::create, b3CubeContact::destroy, b3ShapeType::e_cube, b3ShapeType::e_cube);
    add_type(b3CubeAndSphereContact::create, b3CubeAndSphereContact::destroy, b3ShapeType::e_cube, b3ShapeType::e_sphere);
    add_type(b3PlaneSphereContact::create, b3PlaneSphereContact::destroy, b3ShapeType::e_plane, b3ShapeType::e_sphere);
    add_type(b3PlaneCubeContact::create, b3PlaneCubeContact::destroy, b3ShapeType::e_plane, b3ShapeType::e_cube);
    add_type(b3PlaneConeContact::create, b3PlaneConeContact::destroy, b3ShapeType::e_plane, b3ShapeType::e_cone);
    add_type(b3CubeConeContact::create, b3CubeConeContact::destroy, b3ShapeType::e_cube, b3ShapeType::e_cone);
}


b3Contact::b3Contact(b3Fixture *f_A, int32 index_A, b3Fixture *f_B, int32 index_B)
{
    // TODO: check enable_flags

    m_fixture_a = f_A;
    m_fixture_b = f_B;

    m_index_a = index_A;
    m_index_b = index_B;

    m_prev = nullptr;
    m_next = nullptr;

    m_restitution = 0;
    m_friction = 0;

    m_manifold.m_point_count = 0;

    m_persistent_manifold = nullptr;
    m_algorithm_size = 0;
    m_algorithm = nullptr;

    m_restitution = b3_mix_restitution(m_fixture_a->get_restitution(), m_fixture_b->get_restitution());
    m_friction = b3_mix_friction(m_fixture_a->get_friction(), m_fixture_b->get_friction());
    m_rolling_friction = b3_mix_rolling_friction(m_fixture_a->get_rolling_friction(), m_fixture_b->get_rolling_friction());
    m_spinning_friction = b3_mix_spinning_friction(m_fixture_a->get_spinning_friction(), m_fixture_b->get_spinning_friction());
}


void b3Contact::add_type(
    b3ContactCreateFcn* create_fcn,
    b3ContactDestroyFcn* destroy_fcn,
    b3ShapeType type_A,
    b3ShapeType type_B)
{
    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    s_registers[type_A][type_B].create_fcn = create_fcn;
    s_registers[type_A][type_B].destroy_fcn = destroy_fcn;
    s_registers[type_A][type_B].primary = true;

    if (type_A != type_B) {
        s_registers[type_B][type_A].create_fcn = create_fcn;
        s_registers[type_B][type_A].destroy_fcn = destroy_fcn;
        s_registers[type_B][type_A].primary = false;
    }
}

b3Contact *b3Contact::create(
    b3Fixture *fixture_A, int32 index_A,
    b3Fixture *fixture_B, int32 index_B,
    b3BlockAllocator *block_allocator)
{
    if (!s_initialized) {
	    initialize_registers();
	    s_initialized = true;
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactCreateFcn *create_fcn = s_registers[type_A][type_B].create_fcn;
    if (create_fcn) {
        if (s_registers[type_A][type_B].primary) {
            return create_fcn(fixture_A, index_A, fixture_B, index_B, block_allocator);
        } else {
            return create_fcn(fixture_B, index_B, fixture_A, index_A, block_allocator);
        }
    } else {
  	    return nullptr;
    }
}


void b3Contact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator)
{
    b3_assert(s_initialized == true);

    b3Fixture *fixture_A = contact->get_fixture_a();
    b3Fixture *fixture_B = contact->get_fixture_b();

    if (contact->m_manifold.m_point_count > 0) {
        // TODO: check this set_awake
        // fixture_A->get_body()->set_awake(true);
        // fixture_B->get_body()->set_awake(true);
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactDestroyFcn *destroy_fcn = s_registers[type_A][type_B].destroy_fcn;
    destroy_fcn(contact, block_allocator);
}


// Update the contact manifold and touching status.
void b3Contact::update(b3Dispatcher* dispatcher, const b3DispatcherInfo& info, b3ContactListener* listener)
{
    // TODO: add warm start, reuse normal and tangent impulse
    // b3Manifold old_manifold = m_manifold

    b3Body *body_a = m_fixture_a->get_body();
    b3Body *body_b = m_fixture_b->get_body();

    // TODO: use dispatcher to get the persistent manifold
    if (m_persistent_manifold == nullptr) {
        m_persistent_manifold = dispatcher->get_new_manifold(body_a, body_b);
        m_persistent_manifold->set_bodies(body_a, body_b);
    }
    dispatcher->dispatch_collision_pair(this, info, m_persistent_manifold);

    b3Transformr xf_a = body_a->get_world_transform();
    b3Transformr xf_b = body_b->get_world_transform();

    // TODO: add sensor ?
    // generate the manifold between the two shapes
    evaluate(&m_manifold, xf_a, xf_b);

    bool touching = m_manifold.m_point_count > 0 || m_persistent_manifold->get_contact_point_count() > 0;

    if (touching) {
  	    m_flags |= e_touching_flag;
    } else {
  	    m_flags &= ~e_touching_flag;
    }

    if (touching && listener) {
        listener->pre_solve(this, &m_manifold);
    }
}


void b3Contact::destroy_collision_algorithm(b3BlockAllocator* block_allocator)
{
    if (m_algorithm != nullptr) {
        // in this function, will release the memory of m_persistent_manifold
        m_algorithm->~b3CollisionAlgorithm();
        block_allocator->free(m_algorithm, m_algorithm_size);
    }
}


