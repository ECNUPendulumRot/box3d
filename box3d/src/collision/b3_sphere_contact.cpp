
#include "collision/b3_sphere_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"


b3Contact *b3SphereContact::create(b3Fixture *fixture_a,
								   int32 index_a,
								   b3Fixture *fixture_b,
								   int32 index_b,
								   b3BlockAllocator *block_allocator) {
  void *mem = block_allocator->allocate(sizeof(b3SphereContact));
  return new(mem) b3SphereContact(fixture_a, fixture_b);
}


void b3SphereContact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator) {
  ((b3SphereContact *)contact)->~b3SphereContact();
  block_allocator->free(contact, sizeof(b3SphereContact));
}


b3SphereContact::b3SphereContact(b3Fixture *fixture_a,
								 b3Fixture *fixture_b) :
  b3Contact(fixture_a, 0, fixture_b, 0) {
  b3_assert(fixture_a->get_shape_type() == e_sphere);
  b3_assert(fixture_b->get_shape_type() == e_sphere);
}


void b3SphereContact::evaluate(b3Manifold *manifold, 
							   const b3TransformD &xf_a, const b3TransformD &xf_b) {
  b3_collide_spheres(manifold,
					 (b3SphereShape *)(get_fixture_a()->get_shape()), xf_a,
					 (b3SphereShape *)(get_fixture_b()->get_shape()), xf_b);
}