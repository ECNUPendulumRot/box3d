
#ifndef BOX3D_B3_DISTANCE_HPP
#define BOX3D_B3_DISTANCE_HPP

#include "common/b3_types.hpp"


namespace box3d {

    class b3Fixture;
    class b3DistanceProxy;

    enum class GJKStatus {
        GjkValid,
        GjkInside,
        GjkFailed
    };

    class MinkowskiDiff;
    class GJK;
}

#define GJK_MAX_ITERATION 128
#define GJK_MIN_DISTANCE ((double)0.0001)
#define GJK_DUPLICATED_EPS ((double)0.0001)
#define GJK_ACCURARY ((double)0.0001)
#define GJK_SIMPLEX2_EPS ((double)0.0)
#define GJK_SIMPLEX3_EPS ((double)0.0)
#define GJK_SIMPLEX4_EPS ((double)0.0)

class box3d::GJK {
    struct SV{
        // d: search direction, unit 
        // w: simplex point
        // body a seacrh direction is d
        // body b search direction is -d
        b3Vector3d d, w;
    };
    struct Simplex {

        SV* c[4];
        // 权重
        double p[4];
        // Simplex 阶数
        int rank;
    };

    const b3DistanceProxy* proxy_a;
    const b3DistanceProxy* proxy_b;
    // init search direction
    b3Vector3d m_ray = b3Vector3d(1, 0, 0);
    double m_distance = 0.0;
    Simplex m_simplices[2];
    SV m_store[4];
    SV* m_free[4];
    int m_nfree = 0;
    int m_current = 0;
    Simplex* m_simplex;
    GJKStatus m_status;

public:

    GJK(const b3DistanceProxy* a, b3DistanceProxy* b) : 
            proxy_a(a), proxy_b(b) { }

    void evaluate(const b3Vector3d search_direction = b3Vector3d(1, 0, 0));

    void get_support(const b3Vector3d& d, SV& sv) const;

    void append_vertice(Simplex& simplex, const b3Vector3d& v);

    void remove_vertice(Simplex& simplex);

    double get_distance() const {
        return m_distance;
    }

    // 2-simplex: the closet point distance
    static double project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            double* w, int& mask);
    
    static double project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            const b3Vector3d& c, double* w, int& mask);
    
    static double project_origin(const b3Vector3d& a, const b3Vector3d& b, 
            const b3Vector3d& c, const b3Vector3d& d, double* w, int& mask);

    static double det(const b3Vector3d& a, const b3Vector3d& b, const b3Vector3d& c);
};

#endif