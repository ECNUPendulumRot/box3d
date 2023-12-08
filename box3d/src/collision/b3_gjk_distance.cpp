
#include "collision/b3_gjk_distance.hpp"
#include "collision/b3_distance_proxy.hpp"

#include "utils/b3_log.hpp"


void box3d::GJK::evaluate() {

    // initial search direction
    search_dir << 1, 0, 0;
    search_dir_negative = -search_dir;

    // initial point for simplex
    c = proxy_b->get_support(search_dir) - proxy_a->get_support(search_dir_negative);
    
    // search in direaction of origin
    search_dir = -search_dir;
    search_dir_negative = -search_dir;

    // get second point for a line segment simplex
    b = proxy_b->get_support(search_dir) - proxy_a->get_support(search_dir_negative);

    // didn't reach the origin, won't enclose it
    if(b.dot(search_dir) < 0) {
        simplex_dim = 1;
        spdlog::info("c is {}, {}, {}, norm is {}", c.x(), c.y(), c.z(), c.norm());
        m_distance = c.norm();
        return;
    }

    Eigen::Vector3d bc = c - b;
    Eigen::Vector3d BO = -b;
    search_dir = bc.cross(BO).cross(bc);
    // origin is on this line segment
    // search_dir == Eigen::Vector3d::Zero()
    if(search_dir.squaredNorm() <= GJK_MIN_DISTANCE) {
        m_status = GJKStatus::GjkInside;
        return;
    }

    simplex_dim = 2;
    int iterations = 0;

    for(; iterations < GJK_MAX_ITERATION, m_status == GJKStatus::GjkValid; ++iterations) {
        search_dir.normalize();
        search_dir_negative = -search_dir;
        a = proxy_b->get_support(search_dir) - proxy_a->get_support(search_dir_negative);
        // didn't reach the origin
        if(a.dot(search_dir) < 0) {
            if(simplex_dim == 2) {
                Eigen::Vector3d bc = c - b;
                Eigen::Vector3d CO = -c;
                m_distance = CO.cross(bc).norm();
            } else {
                Eigen::Vector3d bc = c - b;
                Eigen::Vector3d bd = d - b;
                Eigen::Vector3d n = bc.cross(bd);
                n.normalize();
                m_distance = -b.dot(n);
            }
            return;

        }

        simplex_dim++;
        if(simplex_dim == 3) {
            update_simplex3();
        } else if(simplex_dim == 4) {
            update_simplex4();
        }
    }
    if(iterations == GJK_MAX_ITERATION) {
        m_status == GJKStatus::GjkFailed;
        m_distance = -1;
        return;
    }
    if(m_status == GJKStatus::GjkInside) {
        m_distance = 0;
        return;
    }
    Eigen::Vector3d ac = c - a;
    Eigen::Vector3d ab = b - a;
    m_distance = b3_fabs(((-c).dot((ac).cross(ab))));
}

void box3d::GJK::update_simplex3() {
    /**
     * b
     * | \
     * |  \
     * |   \
     * |    a
     * |   /
     * | /
     * c
    */
    // triangle's normal 
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ac = c - a;
    Eigen::Vector3d n = ab.cross(ac);
    Eigen::Vector3d AO = -a;

    // Determine which feature is closest to origin,
    // make that the new simplex
    simplex_dim = 2;
    if(AO.dot(ab.cross(n)) > 0 ) {
        // Closest to edge AB
        c = a;
        search_dir = ab.cross(AO).cross(ab);
        return;
    }
    if(AO.dot(n.cross(ac)) > 0) {
        // Cloest to edge AC
        b = a;
        search_dir = ac.cross(AO).cross(ac);
        return;
    }
    // triangle 
    simplex_dim = 3;
    if(b3_fabs(AO.dot(n)) < GJK_MIN_DISTANCE) {
        spdlog::info("AO is {}, {}, {}", AO.x(), AO.y(), AO.z());
        spdlog::info("n is {}, {}, {}", n.x(), n.y(), n.z());
        spdlog::info("AO dot n is {}", AO.dot(n));
        m_status = GJKStatus::GjkInside;
        return;
    }

    if(AO.dot(n) > 0) {
        // Above triangle
        d = c; 
        c = b;
        b = a;
        search_dir = n;
        return;
    }

    // Below triangle
    d = b;
    b = a;
    search_dir = -n;
    return;
}

void box3d::GJK::update_simplex4() {
    // a is peak/tip of pyramid, BCD is the 
    // base(counterclockwise winding order)
    // We know a priori that origin is above BCD and below a
    
    // get normals of three new faces
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ac = c - a;
    Eigen::Vector3d ad = d - a;
    Eigen::Vector3d ABC = ab.cross(ac);
    Eigen::Vector3d ACD = ac.cross(ad);
    Eigen::Vector3d ADB = ad.cross(ab);

    Eigen::Vector3d AO = -a; // dir to origin
    simplex_dim = 3; 

    if(ABC.dot(AO) > 0) {
        // In front of ABC
        d = c;
        c = b;
        b = a;
        search_dir = ABC;
        return;
    }
    if(ADB.dot(AO) > 0) {
        // In front of ADB
        c = d;
        d = b;
        b = a;
        search_dir = ADB;
        return;
    }
    if(ACD.dot(AO) > 0) {
        // In front of ACD
        b = a;
        search_dir = ACD;
        return;
    }
    // else inside tetrahedron; enclosed;
    m_status = GJKStatus::GjkInside;
}

/*
void box3d::GJK::evaluate(const b3Vector3d search_direction) {

    // 迭代次数
    int32 iterations = 0;
    // 记录当前距离的平方
    double sqdist = 0.0;
    // 用于比较更新
    double alpha = 0.0;
    // 记录最近使用的搜索方向
    b3Vector3d lastw[4];
    // index of lastw
    int clastw = 0;

    m_free[0] = &m_store[0];
    m_free[1] = &m_store[1];
    m_free[2] = &m_store[2];
    m_free[3] = &m_store[3];
    m_nfree = 4;
    m_current = 0;
    m_status = GJKStatus::GjkValid;
    m_distance = 0;

    // default: [1, 0, 0]
    m_ray = search_direction;

    m_simplices[0].rank = 0;
    append_vertice(m_simplices[0], m_ray);
    m_simplices[0].p[0] = 1;
    m_ray = m_simplices[0].c[0]->w;
    sqdist = m_ray.length2();
    lastw[0] = lastw[1] = lastw[2] = lastw[3] = m_ray;

    while(m_status == GJKStatus::GjkValid) {
        const int next = 1 - m_current;
        Simplex& cs = m_simplices[m_current];
        Simplex& ns = m_simplices[next];

        // Check zero
        const double rl = m_ray.length();
        if(rl < GJK_MIN_DISTANCE) {
            // Touching or inside
            m_status = GJKStatus::GjkInside;
            break;
        }

        // Append new vertice in '-v' direction
        append_vertice(cs, -m_ray);
        // the new witness point
        const b3Vector3d& w = cs.c[cs.rank - 1]->w;
        bool found = false;
        for(int32 i = 0; i < 4; ++i) {
            /// the search direction was used 
            if((w - lastw[i]).length2() < GJK_DUPLICATED_EPS) {
                found = true;
                break;
            }
        }

        if(found) {
            // Return old simple
            remove_vertice(m_simplices[m_current]);
            break;
        } else {
            // update lastw
            clastw = (clastw + 1) & 3;
            lastw[clastw] = w;
        }

        // Check for temination
        // ???
        const double omega = m_ray.dot(w) / rl;
        alpha = b3_max(alpha, omega);
        if((rl - alpha) - (GJK_ACCURARY * rl) <= 0) {
            // return old simple
            remove_vertice(m_simplices[m_current]);
            break;
        }

        // reduce simplex
        double weights[4];
        int mask = 0;
        switch (cs.rank) {
        case 2:
            sqdist = project_origin(cs.c[0]->w, 
                                    cs.c[1]->w, weights, mask);
            break;
        case 3:
            sqdist = project_origin(cs.c[0]->w, 
                                    cs.c[1]->w,
                                    cs.c[2]->w,
                                    weights, mask);
            break;
        case 4:
            sqdist = project_origin(cs.c[0]->w,
                                    cs.c[1]->w,
                                    cs.c[2]->w,
                                    cs.c[3]->w, 
                                    weights, mask);
            break;
        }
        if(sqdist > 0) {
            // Valid
            // ????
            ns.rank = 0;
            m_ray = b3Vector3d(0, 0, 0);
            m_current = next;
            for(int i = 0; i < cs.rank; ++i) {
                if(mask & (1 << i)) {
                    ns.c[ns.rank] = cs.c[i];
                    ns.p[ns.rank++] = weights[i];
                    m_ray += cs.c[i]->w * weights[i];
                } else {
                    m_free[m_nfree++] = cs.c[i];
                }
            }
            if(mask == 15) {
                m_status = GJKStatus::GjkInside;
            }
        } else {
            // return old simplex
            remove_vertice(m_simplices[m_current]);
            break;
        }
        ++iterations;
        if(iterations == GJK_MAX_ITERATION) {
            m_status = GJKStatus::GjkFailed;
        }
    }
    m_simplex = &m_simplices[m_current];
    switch (m_status) {
    case GJKStatus::GjkValid:
        m_distance = m_ray.length();
        break;
    case GJKStatus::GjkInside:
        m_distance = 0;
        break;
    case GJKStatus::GjkFailed:
        m_distance = -1;
        break;
    }
}

void box3d::GJK::get_support(const b3Vector3d& d, SV& sv) const {
    sv.d = d.normalized();
    b3Vector3d a = proxy_a->get_support(sv.d);
    b3Vector3d b = proxy_b->get_support(-sv.d);

    spdlog::info("direction is {}, {}, {}", sv.d.x(), sv.d.y(), sv.d.z());
    spdlog::info("select the vector in a is {}, {}, {}", a.x(), a.y(), a.z());
    spdlog::info("select the vector in b is {}, {}, {}", b.x(), b.y(), b.z());

    sv.w = proxy_a->get_support(sv.d) - proxy_b->get_support(-sv.d);
}

void box3d::GJK::append_vertice(Simplex& simplex, const b3Vector3d& v) {
    simplex.p[simplex.rank] = 0;
    simplex.c[simplex.rank] = m_free[--m_nfree];
    get_support(v, *simplex.c[simplex.rank++]);

}

void box3d::GJK::remove_vertice(Simplex& simplex) {
    m_free[m_nfree++] = simplex.c[--simplex.rank];
}

double box3d::GJK::project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            double* w, int& mask) {
    const b3Vector3d d = b - a;
    const double l = d.length();

    if(l > GJK_SIMPLEX2_EPS) {
        double t = 0;
        if(l > 0) {
            t = -a.dot(b) / l;
        }
        if(t >= 1) {
            w[0] = 0;
            w[1] = 1;
            mask = 2;
            return b.length2();
        } else if(t <= 0) {
            w[0] = 1;
            w[1] = 0;
            mask = 1;
            return a.length2();
        } else {
            w[1] = t;
            w[0] = 1 - t;
            mask = 3;
            return (a + d * t).length2();
        }
    }
    return -1;
}

double box3d::GJK::project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            const b3Vector3d& c, double* w, int& mask) {
    static const int imd3[] = { 1, 2, 0};
    const b3Vector3d* vt[] = { &a, &b, &c };
    const b3Vector3d dl[] = { a - b, b - c, c - a};
    const b3Vector3d n = dl[0].cross(dl[1]);
    const double l = n.length2();
    if(l > GJK_SIMPLEX3_EPS) {
        double mindist = -1;
        double subw[2] = { 0, 0 };
        int subm = 0;
        for(int i = 0; i < 3; ++i) {
            if(vt[i]->dot(dl[i].cross(n)) > 0) {
                const int j = imd3[i];
                const double subd = project_origin(*vt[i], *vt[j], subw, subm);
                if(mindist < 0 || subd < mindist) {
                    mindist = subd;
                    mask = static_cast<int>((subm & 1) ? 1 << i : 0 + (subm & 2) ? 1 << j : 0);
                    w[i] = subw[0];
                    w[j] = subw[1];
                    w[imd3[j]] = 0;
                }
            }
        }
        if(mindist < 0) {
            const double d = a.dot(n);
            const double s = b3_sqrt(l);
            const b3Vector3d p = n * (d / l);
            mindist = p.length2();
            mask = 7;
            w[0] = (dl[1].cross(b - p)).length() / s;
            w[1] = (dl[2].cross(c - p)).length() / s;
            w[2] = 1 - w[0] - w[1];
        }
        return mindist;
    }
    return -1;
}

double box3d::GJK::project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            const b3Vector3d& c, const b3Vector3d& d, double* w, int& mask) {
    static const int imd3[] = { 1, 2, 0};
    const b3Vector3d* vt[] = { &a, &b, &c, &d };
    const b3Vector3d dl[] = { a - d, b - d, c - d };
    const double vl = det(dl[0], dl[1], dl[2]); 

    const bool ng = (vl * a.dot((b - c).cross(a - b))) <= 0;
    if(ng && b3_fabs(vl) > GJK_SIMPLEX4_EPS) {
        double mindist = -1;
        double subw[3] = { 0, 0, 0};
        int subm = 0;
        for(int i = 0; i < 3; ++i) {
            const int j = imd3[i];
            const double s = vl * d.dot(dl[i].cross(dl[j]));
            if(s > 0) {
                const double subd = project_origin(*vt[i], *vt[j], d, subw, subm);
                if(mindist < 0 || subd < mindist) {
                    mindist = subd;
                    mask = static_cast<int>(subm & 1 ? 1 << i : 0 + 
                                            subm & 2 ? 1 << j : 0 + 
                                            subm & 4 ? 8 : 0);
                    w[i] = subw[0];
                    w[j] = subw[1];
                    w[imd3[j]] = 0;
                    w[3] = subw[2];
                }
            }
        }
        if(mindist < 0) {
            mindist = 0;
            mask = 15;
            w[0] = det(c, b, d) / vl;
            w[1] = det(a, c, d) / vl;
            w[2] = det(b, a, d) / vl;
            w[3] = 1 - w[0] - w[1] - w[2];
        }
        return mindist;
    }
    return -1;
}

double box3d::GJK::det(const b3Vector3d& a, const b3Vector3d& b, const b3Vector3d& c) {
    return (a.y() * b.z() * c.x() + a.z() * b.x() * c.y() - 
            a.x() * b.z() * c.y() - a.y() * b.x() * c.z() + 
            a.x() * b.y() * c.z() - a.z() * b.y() * c.x());
}

*/