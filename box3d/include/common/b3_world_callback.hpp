
#ifndef BOX3D_B3_WORLD_CALLBACK_HPP
#define BOX3D_B3_WORLD_CALLBACK_HPP


#include "common/b3_common.hpp"


class b3Contact;
struct b3Manifold;


class b3ContactListener {

public:

    virtual ~b3ContactListener() = default;

    virtual void begin_contact(b3Contact* contact) {b3_NOT_USED(contact);}

    virtual void end_contact(b3Contact* contact) {b3_NOT_USED(contact);}

    virtual void pre_solve(b3Contact* contact, const b3Manifold* old_manifold) {
        b3_NOT_USED(contact);
        b3_NOT_USED(old_manifold);
    }

    virtual void post_solve(b3Contact* contact, const b3Manifold* manifold) {
        b3_NOT_USED(contact);
        b3_NOT_USED(manifold);
    }
};


#endif //BOX3D_B3_WORLD_CALLBACK_HPP
