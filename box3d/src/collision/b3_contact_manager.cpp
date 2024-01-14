
#include "collision/b3_contact_manager.hpp"
#include "collision/b3_contact.hpp"

#include "collision/b3_fixture.hpp"


void box3d::b3ContactManager::find_new_contact()
{
    m_broad_phase.update_pairs(this);
}


void box3d::b3ContactManager::add_pair(b3FixtureProxy* fixture_proxy_a, b3FixtureProxy* fixture_proxy_b)
{

    b3Fixture* fixture_a = fixture_proxy_a->get_fixture();
    b3Fixture* fixture_b = fixture_proxy_b->get_fixture();

    b3Body* body_a = fixture_a->get_body();
    b3Body* body_b = fixture_b->get_body();

    int32 index_a = fixture_proxy_a->m_child_id;
    int32 index_b = fixture_proxy_b->m_child_id;

    // check whether the two proxies belongs to the same body
    // if they belong to the same, they do not need to check collision
    if(body_a == body_b) {
        return;
    }
 
    b3ContactEdge* edge = body_b->get_contact_list();

    while(edge) {

        if(edge->m_other == body_a) {
            b3Fixture* f_a = edge->m_contact->get_fixture_a();
            b3Fixture* f_b = edge->m_contact->get_fixture_b();

            int32 i_a = edge->m_contact->get_child_index_a();
            int32 i_b = edge->m_contact->get_child_index_b();


            if(f_a == fixture_a && f_b == fixture_b && i_a == index_a && i_b == index_b) {
                // two fixtures now already exist a contact
                return;
            }

            if(f_a == fixture_b && f_b == fixture_a && i_a == index_b && i_b == index_a) {
                // two fixtures now already exist a contact
                return;
            }
        }
        edge = edge->m_next;
    }

    // b3Contact* contact = b3Contact::create(fixture_a, fixture_b);
    b3Contact* contact = b3Contact::create(fixture_a, 0, fixture_b, 0);

    if(contact == nullptr) {
        return;
    }

    contact->set_prev(nullptr);
    contact->set_next(m_contact_list);
    if(m_contact_list != nullptr) {
        m_contact_list->set_prev(contact);
    }
    m_contact_list = contact;

    // Connect to island graph
    // delete ? 

    // Connect to body A
    // b3ContactEdge* node_a = contact->get_node_a();
    // node_a->set_other(body_b);
    // node_a->set_contact(contact);
    // node_a->set_prev(nullptr);
    // node_a->set_next(body_a->get_contact_list());
    // if(body_a->get_contact_list() != nullptr) {
    //     body_a->get_contact_list()->set_prev(node_a);
    // }
    // body_a->set_contact_list(node_a);

    // Connect to body B
    // b3ContactEdge* node_b = contact->get_node_b();
    // node_b->set_other(body_a);
    // node_b->set_contact(contact);
    // node_b->set_prev(nullptr);
    // node_b->set_next(body_b->get_contact_list());
    // if(body_b->get_contact_list() != nullptr) {
    //     body_b->get_contact_list()->set_prev(node_b);
    // }
    // body_b->set_contact_list(node_b);

    ++m_contact_count;
}


void box3d::b3ContactManager::destory(b3Contact* contact)
{
    
    if(contact->get_prev()) {
        contact->get_prev()->set_next(contact->get_next());
    }

    if(contact->get_next()) {
        contact->get_next()->set_prev(contact->get_prev());
    }

    if(contact == m_contact_list) {
        m_contact_list = contact->get_next();
    }

    // remove form body a
    
    // remove form body b

}


void box3d::b3ContactManager::collide()
{

    b3Contact* current_contact = m_contact_list;

    while(current_contact) {
        
        // is this pair shouldn't collide,
        // destory this contact

        // box2d do a overlap in the broad-phase ??

        // do a narrow phase
        // current_contact->update();
        current_contact = current_contact->get_next();
    }
}