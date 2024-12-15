
#ifndef BOX3D_B3_ISLAND_HPP
#define BOX3D_B3_ISLAND_HPP


#include "common/b3_common.hpp"
#include "common/b3_types.hpp"

/////////// Forward Delaration ///////////

class b3Contact;

class b3Body;

class b3BlockAllocator;

class b3BodySim;

//////////////////////////////////////////


class b3Island {

public:

    int32 m_body_count;
	int32 m_contact_count;

	int32 m_body_capacity;
	int32 m_contact_capacity;

    b3BodySim** m_bodies;
    b3Contact** m_contacts;

    b3BlockAllocator* m_block_allocator;

    b3Island(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity);

    ~b3Island();

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

    void add_body(b3BodySim* body) {
        b3_assert(m_body_count < m_body_capacity);
        m_bodies[m_body_count] = body;
        m_body_count++;
    }
};


class b3StaticIsland: public b3Island {

public:

    b3StaticIsland(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity): b3Island(block_allocator, body_capacity, contact_capacity) {}

    void add_body(b3BodySim* body);
};

class b3NormalIsland: public b3Island {
public:
    b3NormalIsland(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity): b3Island(block_allocator, body_capacity, contact_capacity) {}
    void add_body(b3BodySim* body);
};

#endif // BOX3D_B3_ISLAND_HPP