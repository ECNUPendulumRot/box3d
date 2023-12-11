
#ifndef BOX3D_B3_GJK_DISTANCE_HPP
#define BOX3D_B3_GJK_DISTANCE_HPP

#include "common/b3_types.hpp"


namespace box3d
{

    class b3Fixture;

    class b3DistanceProxy;

    enum class GJKStatus 
    {
        GJKVALID,
        GJKINSIDE,
        GJKFAILED
    };

    class GJK;
}

#define GJK_MAX_ITERATION 128
#define GJK_MIN_DISTANCE ((double)0.0001)

/// https://vec3.ca/gjk/implementation/
/// https://github.com/kevinmoran/GJK
/// https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/NarrowPhaseCollision/btGjkEpa3.h

class box3d::GJK
{

    const b3DistanceProxy* m_proxy_a;
    const b3DistanceProxy* m_proxy_b;
    
    // Simplex: just a set of points
    Eigen::Vector3d m_a, m_b, m_c, m_d;

    Eigen::Vector3d m_search_dir;

    Eigen::Vector3d m_last_used_search_dir[4];

    double m_distance;

    GJKStatus m_status = GJKStatus::GJKVALID;

    // simplex dimension
    int32 m_simplex_dim = 0;

public:

    GJK(const b3DistanceProxy* a, const b3DistanceProxy* b): 
            m_proxy_a(a), m_proxy_b(b) {}

    void evaluate();

    double get_distance() const {
        return m_distance;
    }

private:

    void update_simplex2();

    void update_simplex3();
    
    void update_simplex4();

    void compute_distance();

};


#endif