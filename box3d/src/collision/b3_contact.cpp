
#include "collision/b3_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"


box3d::b3ContactRegister box3d::b3Contact::s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];
bool box3d::b3Contact::s_initialized = false;

void box3d::b3Contact::initialize_registers()
{

}


box3d::b3Contact::b3Contact(box3d::b3Fixture *f_A, int32 index_A, box3d::b3Fixture *f_B, int32 index_B)
{
    // TODO: check enable_flags

    m_fixture_a = f_A;
    m_fixture_b = f_B;

    m_index_a = index_A;
    m_index_b = index_B;

    m_prev = nullptr;
    m_next = nullptr;

    m_node_a.m_contact = nullptr;
    m_node_a.m_prev = nullptr;
    m_node_a.m_next = nullptr;
    m_node_a.m_other = nullptr;

    m_node_b.m_contact = nullptr;
    m_node_b.m_prev = nullptr;
    m_node_b.m_next = nullptr;
    m_node_b.m_other = nullptr;

    m_manifold.point_count = 0;
}


void box3d::b3Contact::add_type(b3ContactCreateFcn* create_fcn, b3ContactDestroyFcn* destroy_fcn,
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

box3d::b3Contact* box3d::b3Contact::create(
    box3d::b3Fixture *fixture_A, int32 index_A,
    box3d::b3Fixture *fixture_B, int32 index_B)
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


void box3d::b3Contact::destroy(box3d::b3Contact *contact)
{
    b3_assert(s_initialized == true);

    b3Fixture* fixture_A = contact->get_fixture_a();
    b3Fixture* fixture_B = contact->get_fixture_b();

    if (contact->m_manifold.point_count > 0) {
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





