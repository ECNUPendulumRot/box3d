
#include "collision/b3_cube_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"


b3Contact* b3CubeContact::create(
    b3Fixture *fixture_a, int32 index_a,
    b3Fixture *fixture_b, int32 index_b)
{
    void* mem = b3_alloc(sizeof(b3CubeContact));
    return new (mem) b3CubeContact(fixture_a, fixture_b);
}


void b3CubeContact::destroy(b3Contact *contact)
{
    ((b3CubeContact*)contact)->~b3CubeContact();
    b3_free(contact);
}


b3CubeContact::b3CubeContact(b3Fixture *fixture_a, b3Fixture *fixture_b):
    b3Contact(fixture_a, 0, fixture_b, 0)
{
    b3_assert(m_fixture_a->get_shape_type() == b3ShapeType::e_cube);
    b3_assert(m_fixture_b->get_shape_type() == b3ShapeType::e_cube);
}


void b3CubeContact::evaluate(b3Manifold *manifold, const b3TransformD &xf_A, const b3TransformD &xf_B)
{
    b3_collide_cube(manifold, (b3CubeShape*)m_fixture_a->get_shape(), xf_A, (b3CubeShape*)m_fixture_b->get_shape(), xf_B);
}
