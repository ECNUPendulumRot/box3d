#include "collision/b3_plane_cube_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"


b3Contact* b3PlaneCubeContact::create(
    b3Fixture *fixture_a, int32 index_a,
    b3Fixture *fixture_b, int32 index_b,
    b3BlockAllocator* block_allocator)
{
    void* mem = block_allocator->allocate(sizeof(b3PlaneCubeContact));
    return new (mem) b3PlaneCubeContact(fixture_a, fixture_b);
}


void b3PlaneCubeContact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator)
{
    ((b3PlaneCubeContact*)contact)->~b3PlaneCubeContact();
    block_allocator->free(contact, sizeof(b3PlaneCubeContact));
}


b3PlaneCubeContact::b3PlaneCubeContact(b3Fixture *fixture_a, b3Fixture *fixture_b):
    b3Contact(fixture_a, 0, fixture_b, 0)
{
    b3_assert(fixture_a->get_shape_type() == e_plane);
    b3_assert(fixture_b->get_shape_type() == e_cube);
}


void b3PlaneCubeContact::evaluate(b3Manifold *manifold, const b3Transr &xf_a, const b3Transr &xf_b)
{
    b3_collide_plane_and_cube(
        manifold,
        (b3PlaneShape*)get_fixture_a()->get_shape(),
        xf_a,
        (b3CubeShape*)get_fixture_b()->get_shape(),
        xf_b
    );
}
