
#include "b3_distance.hpp"

double box3d::b3DistanceOutput::get_distance() const {
    return distance;
}

void box3d::b3Distance(box3d::b3DistanceInput* input, box3d::b3DistanceOutput* output) {
    
    const b3DistanceProxy* proxy_a = &input->get_distance_proxy_a();
    const b3DistanceProxy* proxy_b = &input->get_distance_proxy_b();

    b3Simplex simplex;
    simplex.init(proxy_a, proxy_b);


}

void box3d::b3Simplex::init(const b3DistanceProxy* proxy_a, const b3DistanceProxy* proxy_b) {

    b3SimplexVertex* v = m_v;
    v->index_a = 0;
    v->index_b = 0;

    v->wa = proxy_a->get_vertex(v->index_a);
    v->wb = proxy_b->get_vertex(v->index_b);

    v->w = v->wb - v->wa;

    v->a = 1.0;

    m_count = 1;
}

b3Vector3d box3d::b3Simplex::get_search_direction() const {

    switch (m_count) {
    case 1:
        return -m_v[0].w;
    case 2:
        b3Vector3d e12 = m_v[1].w - m_v[0].w;
        /**
         *         e12
         *    A --------->B
         *      \       /
         *       \    /  
         *        \ / 
         *         O
         * 
        */


    }
}