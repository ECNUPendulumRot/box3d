
#include "collision/b3_gjk_distance.hpp"
#include "collision/b3_distance_proxy.hpp"

#include "common/b3_types.hpp"


void box3d::GJK::evaluate() 
{

    // initial search direction
    m_search_dir << 1, 0, 0;

    // initial point for simplex
    m_c = m_proxy_b->get_support(m_search_dir) - m_proxy_a->get_support(-m_search_dir);

    // record last used direction
    m_last_used_search_dir[0] = m_last_used_search_dir[1] 
                              = m_last_used_search_dir[2]
                              = m_last_used_search_dir[3]
                              = m_search_dir;
    int last_used_id = 0;
    
    m_search_dir = -m_search_dir;

    m_simplex_dim = 1;
    int iterations = 0;

    for(; iterations < GJK_MAX_ITERATION, m_status == GJKStatus::GJKVALID; ++iterations) {

        m_search_dir.normalize();

        bool found = false;
        for(int i = 0; i < 4; ++i) {
            if((m_last_used_search_dir[i] - m_search_dir).squaredNorm() < GJK_MIN_DISTANCE) {
                found = true;
            }
        }
        if(found) {
            break;
        }
        last_used_id = (last_used_id + 1) & 3;
        m_last_used_search_dir[last_used_id] = m_search_dir;

        // get next support point
        m_a = m_proxy_b->get_support(m_search_dir) - m_proxy_a->get_support(-m_search_dir);

        m_simplex_dim++;
        if(m_simplex_dim == 2) {
            update_simplex2();
        } else if(m_simplex_dim == 3) {
            update_simplex3();
        } else if(m_simplex_dim == 4) {
            update_simplex4();
        }
    }

    if(iterations == GJK_MAX_ITERATION) {
        m_status == GJKStatus::GJKFAILED;
        m_distance = -1;
        return;
    }
    if(m_status == GJKStatus::GJKINSIDE) {
        m_distance = 0;
        return;
    }
    compute_distance();
}


void box3d::GJK::compute_distance()
{
    if(m_simplex_dim == 1) {
        m_distance = m_c.norm();
        return;
    }
    if(m_simplex_dim == 2) {
        m_distance = m_c.cross(m_c - m_b).norm();
        return;
    }
    // m_simplex_dim == 3
    m_distance = -m_b.dot( (m_c - m_b).cross(m_d - m_b).normalized() );
}


void box3d::GJK::update_simplex2()
{
    /**
     *  m_a
     *  | 
     *  |
     *  |
     *  m_c
    */
    if(m_a.dot(m_search_dir) < 0) {
        // Closest to point m_a
        m_search_dir = -m_a;
        m_c = m_a;
        m_simplex_dim = 1;
        return;
    }
    m_b = m_a;
    m_search_dir = (m_c - m_b).cross(-m_b).cross(m_c - m_b);
    if(m_search_dir == Eigen::Vector3d::Zero()) {
        if(m_c.normalized() == (m_a - m_c).normalized()) {
            m_simplex_dim = 1;
            m_search_dir = -m_c;
        } else {
            // origin is on the linear segment ac
            m_status = GJKStatus::GJKINSIDE;
        }
    }
}


void box3d::GJK::update_simplex3()
{
    /**
     * m_b
     * | \
     * |  \
     * |   \
     * |    m_a
     * |   /
     * | /
     * m_c
    */
    // triangle's normal 
    Eigen::Vector3d ab = m_b - m_a;
    Eigen::Vector3d ac = m_c - m_a;
    Eigen::Vector3d n = ab.cross(ac);

    if(m_a.dot(n) == 0) {
        m_status = GJKStatus::GJKINSIDE;
        return;
    }

    // Determine which feature is closest to origin,
    // make that the new simplex
    m_simplex_dim = 2;
    if(m_a.dot(ab.cross(n)) < 0 ) {
        // Closest to edge AB
        m_c = m_a;
        m_search_dir = ab.cross(-m_a).cross(ab);
        return;
    }
    if(m_a.dot(n.cross(ac)) < 0) {
        // Cloest to edge AC
        m_b = m_a;
        m_search_dir = ac.cross(-m_a).cross(ac);
        return;
    }
    // triangle 
    m_simplex_dim = 3;

    if(m_a.dot(n) < 0) {
        // Above triangle
        m_d = m_c; 
        m_c = m_b;
        m_b = m_a;
        m_search_dir = n;
        return;
    }

    // Below triangle
    m_d = m_b;
    m_b = m_a;
    m_search_dir = -n;
    return;
}


void box3d::GJK::update_simplex4()
{
    // m_a is peak/tip of pyramid, BCD is the 
    // base(counterclockwise winding order)
    // We know m_a priori that origin is above BCD and below m_a
    
    // get normals of three new faces
    Eigen::Vector3d ABC = (m_b - m_a).cross(m_c - m_a);
    Eigen::Vector3d ACD = (m_c - m_a).cross(m_d - m_a);
    Eigen::Vector3d ADB = (m_d - m_a).cross(m_b - m_a);

    m_simplex_dim = 3; 

    if(ABC.dot(m_a) < 0) {
        // In front of ABC
        m_d = m_c;
        m_c = m_b;
        m_b = m_a;
        m_search_dir = ABC;
        return;
    }
    if(ADB.dot(m_a) < 0) {
        // In front of ADB
        m_c = m_d;
        m_d = m_b;
        m_b = m_a;
        m_search_dir = ADB;
        return;
    }
    if(ACD.dot(m_a) < 0) {
        // In front of ACD
        m_b = m_a;
        m_search_dir = ACD;
        return;
    }
    // else inside tetrahedron; enclosed;
    m_status = GJKStatus::GJKINSIDE;
}

