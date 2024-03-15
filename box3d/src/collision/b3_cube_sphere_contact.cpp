
#include "collision/b3_cube_sphere_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"


b3Contact* b3CubeAndSphereContact::create(
  b3Fixture* fixture_a, int32 index_a,
  b3Fixture* fixture_b, int32 index_b,
  b3BlockAllocator* block_allocator)
{
  void* mem = block_allocator->allocate(sizeof(b3CubeAndSphereContact));
  return new(mem) b3CubeAndSphereContact(fixture_a, fixture_b);
}


void b3CubeAndSphereContact::destroy(b3Contact* contact, b3BlockAllocator* block_allocator)
{
  ((b3CubeAndSphereContact*)contact)->~b3CubeAndSphereContact();
  block_allocator->free(contact, sizeof(b3CubeAndSphereContact));
}


b3CubeAndSphereContact::b3CubeAndSphereContact(b3Fixture *fixture_a, b3Fixture *fixture_b):
  b3Contact(fixture_a, 0, fixture_b, 0)
{
  b3_assert(fixture_a->get_shape_type() == e_cube);
  b3_assert(fixture_b->get_shape_type() == e_sphere);
}


void b3CubeAndSphereContact::evaluate(b3Manifold* manifold, const b3TransformD& xf_a, const b3TransformD& xf_b)
{
  b3_collide_cube_and_sphere(
    manifold,
    (b3CubeShape*)(get_fixture_a()->get_shape()), xf_a,
    (b3SphereShape*)(get_fixture_b()->get_shape()), xf_b
  );
}