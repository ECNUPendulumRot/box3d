
#ifndef BOX3D_B3_COLLISION_HPP
#define BOX3D_B3_COLLISION_HPP

#include "common/b3_types.hpp"
#include "dynamics/b3_pose.hpp"

namespace box3d {

    union b3ContactID;

    struct b3Manifold;

    struct b3ManifoldPoint;

    struct b3ContactFeature;

    class b3SphereShape;

    class b3CubeShape;
}


/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct box3d::b3ContactFeature
{
    enum Type
    {
        e_vertex = 0,
        e_face = 1
    };

    uint8 indexA;		///< Feature index on shapeA
    uint8 indexB;		///< Feature index on shapeB
    uint8 typeA;		///< The feature type on shapeA
    uint8 typeB;		///< The feature type on shapeB
};


union box3d::b3ContactID {
    b3ContactFeature cf;
    uint32 key;					///< Used to quickly compare contact ids.
};

struct box3d::b3ManifoldPoint {

    b3Vector3d m_local_point;		///< usage depends on manifold type

    float m_normal_impulse;	///< the non-penetration impulse

    float m_tangent_impulse;	///< the friction impulse

    b3ContactID id;			///< uniquely identifies a contact point between two shapes
};


struct box3d::b3Manifold {

    enum Type
    {
        e_circles,
        e_faceA,
        e_faceB
    };

    b3ManifoldPoint points[8];	///< the points of contact
    b3Vector3d local_normal;								///< not use for Type::e_points
    b3Vector3d local_point;								///< usage depends on manifold type
    Type type;
    int32 point_count;								///< the number of manifold points

    //////
    double m_penetration;

};


/// Compute the collision manifold between two circles.
void b3_collide_spheres(box3d::b3Manifold* manifold,
					    const box3d::b3SphereShape* sphere_a, 
                        const b3TransformD& xf_a,
					    const box3d::b3SphereShape* sphere_b, 
                        const b3TransformD& xf_b);

/// Compute the collision manifold between circle and cube
void b3_collide_cube_and_sphere(box3d::b3Manifold* manifold,
                                const box3d::b3CubeShape* cube_a, 
                                const b3TransformD& xf_a,
                                const box3d::b3SphereShape* sphere_b,
                                const b3TransformD& xf_b);


#endif //BOX3D_B3_COLLISION_HPP
