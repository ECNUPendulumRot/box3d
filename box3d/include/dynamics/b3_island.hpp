
#ifndef BOX3D_B3_ISLAND_HPP
#define BOX3D_B3_ISLAND_HPP


#include "common/b3_types.hpp"


/////////// Forward Delaration ///////////

class b3Contact;

class b3Body;

class b3BlockAllocator;

//////////////////////////////////////////


class b3Island {

public:

    int32 m_body_count;
	int32 m_contact_count;

	int32 m_body_capacity;
	int32 m_contact_capacity;

    b3Body** m_bodies;
    b3Contact** m_contacts;

    b3BlockAllocator* m_block_allocator;

    b3Island(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity);

    ~b3Island();

    void add_body(b3Body* body);

    void add_contact(b3Contact* contact);

    void clear() {
        m_body_count = 0;
        m_contact_count = 0;
    }

    b3Contact** get_contacts() const {
        return m_contacts;
    }

    int get_contacts_count() const {
        return m_contact_count;
    }

    int get_body_count() const {
        return m_body_count;
    }

    b3Body** get_bodies() const {
        return m_bodies;
    }

    b3Body* get_body(int32 index) {
        return m_bodies[index];
    }
};


#endif // BOX3D_B3_ISLAND_HPP