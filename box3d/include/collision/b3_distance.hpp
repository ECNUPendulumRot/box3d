
#ifndef BOX3D_B3_DISTANCE_HPP
#define BOX3D_B3_DISTANCE_HPP

#include "common/b3_types.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_distance_proxy.hpp"

namespace box3d {

    class b3Fixture;
    class b3DistanceProxy;

    class b3DistanceInput;
    class b3DistanceOutput;
    class b3Simplex;
    class b3SimplexVertex;

    void b3Distance(b3DistanceInput* input, b3DistanceOutput* output);
}

class box3d::b3DistanceInput {
    b3DistanceProxy proxy_a;
    b3DistanceProxy proxy_b;

public:

    b3DistanceInput(b3Fixture* a, b3Fixture* b) :
        proxy_a(a), proxy_b(b) {}

    b3DistanceProxy get_distance_proxy_a() const {
        return proxy_a;
    }

    b3DistanceProxy get_distance_proxy_b() const {
        return proxy_b;
    }
};

class box3d::b3DistanceOutput {
    b3MatrixXd point_a;
    b3MatrixXd point_b;
    double distance;
    int32 iterations;

public:

    double get_distance() const;
};

class box3d::b3SimplexVertex {
public:
    /**
     * support point in proxy_a
    */
    b3Vector3d wa; 
    /**
     * support point in proxy_b
    */
    b3Vector3d wb;
    /**
     * wb - wa
    */
    b3Vector3d w;

    /**
     * barycentric coordinate for closest point
    */
    double a;
    /**
     * wa index
    */
    int32 index_a;
    /**
     * wb index
    */
    int32 index_b;
};

class box3d::b3Simplex {
    int32 m_count = 0;
    b3SimplexVertex m_v[4];
public:
    b3Vector3d get_search_direction() const;
    b3Vector3d get_closet_point() const;
    void get_witness_points(b3Vector3d& pA, b3Vector3d& pB) const;
    double get_metric() const;

    void solve2();
    void solve3();
    void solve4();

    /**
     * @brief determine the first witness point
    */
    void init(const b3DistanceProxy* proxy_a, const b3DistanceProxy* proxy_b);
};


#endif