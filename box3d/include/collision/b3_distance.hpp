
#ifndef BOX3D_B3_DISTANCE_HPP
#define BOX3D_B3_DISTANCE_HPP


#include "math/b3_vec3.hpp"
#include "geometry/b3_shape.hpp"


struct b3DistanceProxy {

    b3DistanceProxy() : m_vertices(nullptr), m_count(0), m_radius(0.0f) {
        ;
    }

    void set(const b3Shape* shape, int32 index);

    b3Vec3r m_buffer[2];
    const b3Vec3r* m_vertices;
    int32 m_count;
    real m_radius;
};


struct b3SimplexCache
{
    real metric;		///< length or area
    uint16 count;
    uint8 index_a[3];	///< vertices on shape A
    uint8 index_b[3];	///< vertices on shape B
};


struct b3DistanceInput
{
    b3DistanceProxy proxy_a;
    b3DistanceProxy proxy_b;
    b3Transr transform_a;
    b3Transr transform_b;
    bool use_radii;
};


struct b3DistanceOutput
{
    b3Vec3r point_a;		///< closest point on shapeA
    b3Vec3r point_b;		///< closest point on shapeB
    real distance;
    int32 iterations;	///< number of GJK iterations used
};


void b3_distance(b3DistanceOutput* output, b3SimplexCache* cache, const b3DistanceInput* input);


#endif //BOX3D_B3_DISTANCE_HPP
