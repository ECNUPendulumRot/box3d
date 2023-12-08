
#include "dynamics/b3_contact.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"

box3d::b3Body* box3d::b3ContactEdge::get_other() const {
    return other;
}

box3d::b3Contact* box3d::b3ContactEdge::get_contact() const {
    return contact;
}

box3d::b3ContactEdge* box3d::b3ContactEdge::get_next() const {
    return next;
} 

box3d::b3ContactEdge* box3d::b3ContactEdge::get_prev() const {
    return prev;
}

void box3d::b3ContactEdge::set_other(b3Body* other) {
    this->other = other;
}

void box3d::b3ContactEdge::set_contact(b3Contact* contact) {
    this->contact = contact;
}

void box3d::b3ContactEdge::set_next(b3ContactEdge* next) {
    this->next = next;
}

void box3d::b3ContactEdge::set_prev(b3ContactEdge* prev) {
    this->prev = prev;
}

box3d::b3Fixture* box3d::b3Contact::get_fixture_a() const {
    return m_fixture_a;
}

box3d::b3Fixture* box3d::b3Contact::get_fixture_b() const {
    return m_fixture_b;
}

static box3d::b3Contact* create(const box3d::b3Fixture* fixture_a, const box3d::b3Fixture* fixture_b);

box3d::b3Contact* box3d::b3Contact::get_prev() const {
    return m_prev;
}

box3d::b3Contact* box3d::b3Contact::get_next() const {
    return m_next;
}

box3d::b3Contact* box3d::b3Contact::create(const b3Fixture* fixture_a, const b3Fixture* fixture_b) {
    b3Contact* c = new b3Contact;
    return c;
}

void box3d::b3Contact::set_next(b3Contact* next) {
    m_next = next;
}

void box3d::b3Contact::set_prev(b3Contact* prev) {
    m_prev = prev;
}

void box3d::b3Contact::set_fixture_a(b3Fixture* fixture_a) {
    m_fixture_a = fixture_a;
}

void box3d::b3Contact::set_fixture_b(b3Fixture* fixture_b) {
    m_fixture_b = fixture_b;
}

void box3d::b3Contact::update() {

    bool touching = test_gjk_overlap(m_fixture_a, m_fixture_b);

    // TODO: 
    if(touching) {

    }
}