
#include "collision/b3_cube_cylinder_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"


b3Contact *b3CubeCylinderContact::create(
    b3Fixture *fixture_a, int32 index_a,
    b3Fixture *fixture_b, int32 index_b,
    b3BlockAllocator *block_allocator) {
    void *mem = block_allocator->allocate(sizeof(b3CubeCylinderContact));
    return new (mem) b3CubeCylinderContact(fixture_a, fixture_b);
}


void b3CubeCylinderContact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator) {
    ((b3CubeCylinderContact *)contact)->~b3CubeCylinderContact();
    block_allocator->free(contact, sizeof(b3CubeCylinderContact));
}


b3CubeCylinderContact::b3CubeCylinderContact(b3Fixture *fixture_a, b3Fixture *fixture_b) :
    b3Contact(fixture_a, 0, fixture_b, 0) {
    b3_assert(m_fixture_a->get_shape_type() == b3ShapeType::e_cube);
    b3_assert(m_fixture_b->get_shape_type() == b3ShapeType::e_cylinder);
}


void b3CubeCylinderContact::evaluate(b3Manifold *manifold,
                                 const b3Transformr &xf_A,
                                 const b3Transformr &xf_B) {
    // TODO: Implement this
}
