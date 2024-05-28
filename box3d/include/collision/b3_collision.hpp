
#ifndef BOX3D_B3_COLLISION_HPP
#define BOX3D_B3_COLLISION_HPP

#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Delaration ///////////

class b3CubeShape;

class b3SphereShape;

class b3PlaneShape;

//////////////////////////////////////////


/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b3ContactFeature
{
    enum Type {
        e_f_p = 0x01,
        e_e_e = 0x02,
        e_p_f = 0x04
    };

    uint8 type;

    uint8 index_1;

    uint8 index_2;

    uint8 index_ext;

};


union b3ContactID {

    b3ContactFeature cf;

    uint32 key;					///< Used to quickly compare contact ids.

};


struct b3ClipVertex {

    b3Vec3r v;

    b3ContactID id;

};


// TODO: check to delete this struct.
struct b3ManifoldPoint {

    b3Vec3r m_local_point;		///< usage depends on manifold type

    float m_normal_impulse;	///< the non-penetration impulse

    float m_tangent_impulse;	///< the friction impulse

    b3ContactID id;			///< uniquely identifies a contact point between two shapes
};


struct b3Manifold {

    enum Type {

        e_spheres,

        e_face_A,

        e_face_B,

        e_edges
    };

    b3ManifoldPoint m_points[8];	///< the points of contact

    b3Vec3r m_local_normal;								///< not use for Type::e_points

    b3Vec3r m_local_point;								///< usage depends on manifold type

    // TODO: this maybe not useful, check to delete it.
    Type m_type;

    int32 m_point_count;								///< the number of manifold points

    ////// the penetration of two bodies. divide it equally between the two shapes.
    real m_penetration;
};


struct b3WorldManifold {

    void initialize(const b3Manifold* manifold,
                    const b3Transr& xf_A, real radius_A,
                    const b3Transr& xf_B, real radius_B);

    b3Vec3r normal;
    b3Vec3r points[8];
    real separations[8];
};


/// Compute the collision manifold between two circles.
void b3_collide_spheres(b3Manifold* manifold,
					    const b3SphereShape* sphere_a,
                        const b3Transr& xf_a,
					    const b3SphereShape* sphere_b,
                        const b3Transr& xf_b);

/// Compute the collision manifold between circle and cube
void b3_collide_cube_and_sphere(b3Manifold* manifold,
                                const b3CubeShape* cube_a,
                                const b3Transr& xf_a,
                                const b3SphereShape* sphere_b,
                                const b3Transr& xf_b);

/// Compute the collision manifold between two cubes
void b3_collide_cube(b3Manifold* manifold,
                     const b3CubeShape* cube_A,
                     const b3Transr& xf_A,
                     const b3CubeShape* cube_B,
                     const b3Transr& xf_B);

/// Compute the collision manifold between a plane and a sphere.
void b3_collide_plane_and_sphere(b3Manifold* manifold,
                                 const b3PlaneShape* plane_a,
                                 const b3Transr& xf_a,
                                 const b3SphereShape* sphere_b,
                                 const b3Transr& xf_b);

/// Compute the collision manifold between a plane and a cube.
void b3_collide_plane_and_cube(b3Manifold* manifold,
                               const b3PlaneShape* plane_a,
                               const b3Transr& xf_a,
                               const b3CubeShape* cube_b,
                               const b3Transr& xf_b);

#endif //BOX3D_B3_COLLISION_HPP
