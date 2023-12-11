#ifndef B3_CONTACT_HPP
#define B3_CONTACT_HPP

#include "common/b3_allocator.hpp"

namespace box3d {
    class b3Fixture;
    class b3Body;


    class b3Contact;

    class b3ContactEdge;
}

class box3d::b3ContactEdge {
    b3Body* other;
    b3Contact* contact;
    b3ContactEdge* prev;
    b3ContactEdge* next;

public:

    b3Body* get_other() const;

    b3Contact* get_contact() const;

    b3ContactEdge* get_next() const;

    b3ContactEdge* get_prev() const;

    void set_other(b3Body* other);
    
    void set_contact(b3Contact* contact);

    void set_prev(b3ContactEdge* prev);

    void set_next(b3ContactEdge* next);
};

class box3d::b3Contact {
    b3Contact* m_prev;
    b3Contact* m_next;

    b3Fixture* m_fixture_a;
    b3Fixture* m_fixture_b;

    b3ContactEdge* m_node_a;
    b3ContactEdge* m_node_b;
public:

    b3Contact() : m_prev(nullptr), m_next(nullptr),
        m_fixture_a(nullptr), m_fixture_b(nullptr) {}

    b3Fixture* get_fixture_a() const;

    b3Fixture* get_fixture_b() const;

    static b3Contact* create(const box3d::b3Fixture* fixture_a, const box3d::b3Fixture* fixture_b);

    b3Contact* get_prev() const;

    b3Contact* get_next() const;

    b3ContactEdge* get_node_a() const;

    b3ContactEdge* get_node_b() const;

    void set_next(b3Contact* next);
    
    void set_prev(b3Contact* prev);

    void set_fixture_a(b3Fixture* fixture_a);

    void set_fixture_b(b3Fixture* fixture_b);

    /**
     * @brief narrow phase
    */
    void update();

};

#endif