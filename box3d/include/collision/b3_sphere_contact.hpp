
#ifndef BOX3D_B3SPHERECONTACT_HPP
#define BOX3D_B3SPHERECONTACT_HPP

#include "collision/b3_contact.hpp"

namespace box3d {

    class b3SphereContact;

}


class box3d::b3SphereContact : public b3Contact {

public:

    static b3Contact* create(b3Fixture* fixture_a, int32 index_a, b3Fixture* fixture_b, int32 index_b);

    static void destroy(b3Contact* contact);

    b3SphereContact(b3Fixture* fixture_a, b3Fixture* fixture_b);

    virtual ~b3SphereContact() = default;

    void evaluate(b3Manifold* manifold, const b3TransformD& xf_a, const b3TransformD& xf_b) override;
};



#endif // BOX3D_B3SPHERECONTACT_HPP