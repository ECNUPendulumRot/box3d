
#include "collision/b3_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_sphere_contact.hpp"

b3ContactRegister b3Contact::s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];
bool b3Contact::s_initialized = false;

void b3Contact::initialize_registers()
{
    add_type(b3SphereContact::create, b3SphereContact::destroy, b3ShapeType::e_sphere, b3ShapeType::e_sphere);
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

    m_manifold.m_point_count = 0;
}


void b3Contact::add_type(b3ContactCreateFcn* create_fcn, b3ContactDestroyFcn* destroy_fcn,
                                b3ShapeType type_A, b3ShapeType type_B)
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

b3Contact* b3Contact::create(
    b3Fixture *fixture_A, int32 index_A,
    b3Fixture *fixture_B, int32 index_B)
{
    if (!s_initialized) {
        initialize_registers();
        s_initialized = true;
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactCreateFcn* create_fcn = s_registers[type_A][type_B].create_fcn;
    if (create_fcn) {
        if (s_registers[type_A][type_B].primary) {
            return create_fcn(fixture_A, index_A, fixture_B, index_B);
        } else {
            return create_fcn(fixture_B, index_B, fixture_A, index_A);
        }
    } else {
        return nullptr;
    }
}


void b3Contact::destroy(b3Contact *contact)
{
    b3_assert(s_initialized == true);

    b3Fixture* fixture_A = contact->get_fixture_a();
    b3Fixture* fixture_B = contact->get_fixture_b();

    if (contact->m_manifold.m_point_count > 0) {
        // TODO: check this set_awake
        // fixture_A->get_body()->set_awake(true);
        // fixture_B->get_body()->set_awake(true);
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactDestroyFcn* destroy_fcn = s_registers[type_A][type_B].destroy_fcn;
    destroy_fcn(contact);
}


// Update the contact manifold and touching status.
void b3Contact::update() {
    
    // TODO: add warm start, reuse normal and tangent impluse
    // b3Manifold old_manifold = m_manifold

    b3Body* body_a = m_fixture_a->get_body();
    b3Body* body_b = m_fixture_b->get_body();

    const b3TransformD xf_a = body_a->get_pose();
    const b3TransformD xf_b = body_b->get_pose();

    // TODO: add sensor ?

    evaluate(&m_manifold, xf_a, xf_b);

    if(m_manifold.m_point_count > 0) {
        m_flags |= e_touching_flag;
    } else {
        m_flags &= ~e_touching_flag;
    }
}


