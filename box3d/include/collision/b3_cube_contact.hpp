
#ifndef BOX3D_B3_CUBE_CONTACT_HPP
#define BOX3D_B3_CUBE_CONTACT_HPP


#include "collision/b3_contact.hpp"


class b3CubeContact: public b3Contact {

public:

    static b3Contact* create(b3Fixture* fixture_a, int32 index_a, b3Fixture* fixture_b, int32 index_b, b3BlockAllocator* block_allocator);

    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    b3CubeContact(b3Fixture* fixture_a, b3Fixture* fixture_b);

    virtual ~b3CubeContact() = default;

    void evaluate(b3Manifold* manifold, const b3Transr& xf_a, const b3Transr& xf_b) override;

};


#endif //BOX3D_B3_CUBE_CONTACT_HPP
