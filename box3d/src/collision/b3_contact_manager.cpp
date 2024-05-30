
#include "collision/b3_contact_manager.hpp"

#include "collision/b3_contact.hpp"
#include "collision/b3_fixture.hpp"

#include "dynamics/b3_body.hpp"

#include "common/b3_block_allocator.hpp"


void b3ContactManager::find_new_contact()
{
    // when new fixtures are added, we need to find new contacts
    m_broad_phase.update_pairs(this);
}


void b3ContactManager::add_pair(b3FixtureProxy *fixture_proxy_a, b3FixtureProxy *fixture_proxy_b)
{

    b3Fixture *fixture_a = fixture_proxy_a->get_fixture();
    b3Fixture *fixture_b = fixture_proxy_b->get_fixture();

    b3Body *body_a = fixture_a->get_body();
    b3Body *body_b = fixture_b->get_body();

    int32 index_a = fixture_proxy_a->m_child_id;
    int32 index_b = fixture_proxy_b->m_child_id;

    // check whether the two proxies belongs to the same body
    // if they belong to the same, they do not need to check collision
    if (body_a == body_b) {
  	    return;
    }

    b3ContactEdge *edge = body_b->get_contact_list();

    while (edge) {
	    if (edge->m_other == body_a) {
	        b3Fixture *f_a = edge->m_contact->get_fixture_a();
	        b3Fixture *f_b = edge->m_contact->get_fixture_b();

	        int32 i_a = edge->m_contact->get_child_index_a();
	        int32 i_b = edge->m_contact->get_child_index_b();

	        if (f_a == fixture_a && f_b == fixture_b && i_a == index_a && i_b == index_b) {
	    	    // two fixtures now already exist a contact
	    	    return;
	        }

	        if (f_a == fixture_b && f_b == fixture_a && i_a == index_b && i_b == index_a) {
	    	    // two fixtures now already exist a contact
	    	    return;
	        }
        }
	    edge = edge->m_next;
    }

    b3Contact *contact = b3Contact::create(fixture_a, index_a,
                                           fixture_b, index_b, m_block_allocator);

    if (contact == nullptr) {
  	    return;
    }

    // Contact creation may swap fixtures
    fixture_a = contact->get_fixture_a();
    fixture_b = contact->get_fixture_b();
    body_a = fixture_a->get_body();
    body_b = fixture_b->get_body();

    contact->set_prev(nullptr);
    contact->set_next(m_contact_list);
    if (m_contact_list != nullptr) {
  	    m_contact_list->set_prev(contact);
    }
    m_contact_list = contact;

    // Connect to island graph

    // Connect to body A
    b3ContactEdge *node_a = contact->get_node_a();
    node_a->m_other = body_b;
    node_a->m_contact = contact;
    node_a->m_prev = nullptr;
    node_a->m_next = body_a->get_contact_list();
    if (body_a->get_contact_list() != nullptr) {
  	    body_a->get_contact_list()->m_prev = node_a;
    }
    body_a->set_contact_list(node_a);

    // Connect to body B
    b3ContactEdge *node_b = contact->get_node_b();
    node_b->m_other = body_a;
    node_b->m_contact = contact;
    node_b->m_prev = nullptr;
    node_b->m_next = body_b->get_contact_list();
    if (body_b->get_contact_list() != nullptr) {
  	    body_b->get_contact_list()->m_prev = node_b;
    }
    body_b->set_contact_list(node_b);

    ++m_contact_count;
}


void b3ContactManager::destroy(b3Contact* contact)
{
    b3Fixture* fixture_a = contact->get_fixture_a();
    b3Fixture* fixture_b = contact->get_fixture_b();
    b3Body* body_a = fixture_a->get_body();
    b3Body* body_b = fixture_b->get_body();

    if (contact->m_prev) {
        contact->m_prev->m_next = contact->m_next;
    }

    if (contact->m_next) {
        contact->m_next->m_prev = contact->m_prev;
    }

    if (contact == m_contact_list) {
  	    m_contact_list = contact->m_next;
    }

    // remove form body a
    // not use friend class.
    if (contact->m_node_a.m_prev) {
  	    contact->m_node_a.m_prev->m_next = contact->m_node_a.m_next;
    }

    if (contact->m_node_a.m_next) {
  	    contact->m_node_a.m_next->m_prev = contact->m_node_a.m_prev;
    }

    if (&contact->m_node_a == body_a->m_contact_list) {
  	    body_a->m_contact_list = contact->m_node_a.m_next;
    }

    // remove form body b
    if (contact->m_node_b.m_prev) {
  	    contact->m_node_b.m_prev->m_next = contact->m_node_b.m_next;
    }
    if (contact->m_node_b.m_next) {
  	    contact->m_node_b.m_next->m_prev = contact->m_node_b.m_prev;
    }

    if (&contact->m_node_b == body_b->m_contact_list) {
  	    body_b->m_contact_list = contact->m_node_b.m_next;
    }

    b3Contact::destroy(contact, m_block_allocator);
    --m_contact_count;
}


void b3ContactManager::collide()
{

    b3Contact *contact = m_contact_list;

    while (contact) {

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();
        int32 index_a = contact->get_child_index_a();
        int32 index_b = contact->get_child_index_b();

	    b3Body *body_a = fixture_a->get_body();
	    b3Body *body_b = fixture_b->get_body();
        // is this pair shouldn't collide,
        // destory this contact

        bool active_a = body_a->get_type() != b3BodyType::b3_static_body;
        bool active_b = body_b->get_type() != b3BodyType::b3_static_body;

        if (active_a == false && active_b == false) {
            contact = contact->next();
            continue;
        }

        int32 proxy_id_a = fixture_a->m_proxies[index_a].m_proxy_id;
        int32 proxy_id_b = fixture_b->m_proxies[index_b].m_proxy_id;
	    bool overlap = m_broad_phase.test_overlap(proxy_id_a, proxy_id_b);

        if (!overlap) {
            b3Contact *destory_c = contact;
            contact = contact->next();
            destroy(destory_c);
            continue;
        }

        // the contact persist
        contact->update(m_contact_listener);
        contact = contact->next();
    }
}