// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.



#ifndef BOX3D_B3_COLLISION_HPP
#define BOX3D_B3_COLLISION_HPP

#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Declaration ///////////

class b3CubeShape;

class b3SphereShape;

class b3PlaneShape;

//////////////////////////////////////////


/**
 * @brief The features that intersect to form the contact point
 * This must be 4 bytes or less.
 */
struct b3ContactFeature
{

    /**
     * @brief enum-type
     * e_f_p: corresponds to the hexadecimal value 0x01, indicating the face-point type
     * e_e_e: corresponds to the hexadecimal value 0x02, indicating the edge-edge type.
     * e_p_f: corresponds to the hexadecimal value 0x04, indicating the point-faces type
     */
    enum Type {
        e_f_p = 0x01,
        e_e_e = 0x02,
        e_p_f = 0x04
    };

    /**
     * @brief Represents the type of a feature
     */
    uint8 type;

    /**
     * @brief Contact feature related index
     */
    uint8 index_1;

    /**
     * @brief Contact feature related index
     */
    uint8 index_2;

    /**
     * @brief Contact feature related index, indicates extended index or additional index information
     */
    uint8 index_ext;

};

/**
 * @brief b3ContactID is a union of b3ContactFeature and uint32 key,
 * used for fast comparison of contact identifiers.
 */
union b3ContactID {

    /**
     * @brief Represents detailed information about contact features
     */
    b3ContactFeature cf;

    /**
    * @brief Used to quickly compare contact ids.
    */
    uint32 key;

};

/**
 * @brief Represents the clipping vertex in collision detection
 */
struct b3ClipVertex {

    /**
     * @brief v: a vertex
     */
    b3Vec3r v;

    /**
     * @brief The contact identifier associated with the vertex
     */
    b3ContactID id;

};


// TODO: check to delete this struct.
/**
 * @brief Describe the point of contact between two shapes
 */
struct b3ManifoldPoint {

    /**
     * @brief usage depends on manifold type
     */
    b3Vec3r m_local_point;

    /**
     * @brief the non-penetration impulse
     */
    float m_normal_impulse;

    /**
     * @brief the friction impulse
     */
    float m_tangent_impulse;

    /**
     * @brief uniquely identifies a contact point between two shapes
     */
    b3ContactID id;
};

/**
 * @brief Represents a collision manifold that stores contact points
 */
struct b3Manifold {

    /**
     * @brief enum-type
     * e_spheres: indicates collisions between two spheres
     * e_face_A: indicates A face A in the collision.
     * e_face_B: indicates the other side B in the collision.
     * e_edges: Edges in a collision.
     */
    enum Type {

        e_spheres,

        e_face_A,

        e_face_B,

        e_edges
    };

    /**
     * @brief the points of contact
     */
    b3ManifoldPoint m_points[8];

    /**
     * @brief not use for Type::e_points
     */
    b3Vec3r m_local_normal;

    /**
     * @brief usage depends on manifold type
     */
    b3Vec3r m_local_point;

    /**
     * @brief enum-type
     */
    Type m_type;

    /**
     * @brief  the number of manifold points
     */
    int32 m_point_count;
};

/**
 * @brief Calculate and store collision information in world space
 */
struct b3WorldManifold {

    /**
     * @brief Initialize the member variables of the b3WorldManifold structure based on
     * the given collision manifold and the transformation information of two shapes.
     * @param manifold Contact point set
     * @param xf_A The transformation of shape A
     * @param radius_A the radius of A
     * @param xf_B The transformation of shape B
     * @param radius_B the radius of B
     */
    void initialize(const b3Manifold* manifold,
                    const b3Transr& xf_A, real radius_A,
                    const b3Transr& xf_B, real radius_B);

    /**
     * @brief Collision surface normal
     */
    b3Vec3r normal;

    /**
     * @brief Groups of contact points in world space
     */
    b3Vec3r points[8];

    /**
     * @brief Contact point distance/separation distance array
     */
    real separations[8];
};

/**
 * @brief Compute the collision manifold between two circles.
 * @param manifold A pointer to the b3Manifold structure that will be filled with collision details if the spheres collide
 * @param sphere_a A pointer to the shape of the first sphere involved in the collision
 * @param xf_a Constant reference, transformation of the first sphere (position and direction)
 * @param sphere_b A pointer to the shape of the second sphere involved in the collision
 * @param xf_b Transformation of the second sphere (position and direction)
 */
void b3_collide_spheres(b3Manifold* manifold,
					    const b3SphereShape* sphere_a,
                        const b3Transr& xf_a,
					    const b3SphereShape* sphere_b,
                        const b3Transr& xf_b);


/**
 * @brief Compute the collision manifold between circle and cube
 * @param manifold If the cube and the sphere collide, the structure will be filled with collision details.
 * @param cube_a A pointer to the cube shape involved in the collision
 * @param xf_a Transformation of the cube (position and orientation)
 * @param sphere_b A pointer to the shape of the sphere involved in the collision
 * @param xf_b Transformation of the sphere (position and direction)
 */
void b3_collide_cube_and_sphere(b3Manifold* manifold,
                                const b3CubeShape* cube_a,
                                const b3Transr& xf_a,
                                const b3SphereShape* sphere_b,
                                const b3Transr& xf_b);


/**
 * @brief Compute the collision manifold between two cubes
 * @param manifold Pointer to the b3Manifold structure for storing collision details between cubes
 * @param cube_A Pointer to the first cube shape involved in the collision
 * @param xf_A The transformation information of the first cube, including position and orientation
 * @param cube_B Pointer to the second cube shape involved in the collision
 * @param xf_B Transformation information for the second cube, including position and orientation.
 */
void b3_collide_cube(b3Manifold* manifold,
                     const b3CubeShape* cube_A,
                     const b3Transr& xf_A,
                     const b3CubeShape* cube_B,
                     const b3Transr& xf_B);


/**
 * @brief Compute the collision manifold between a plane and a sphere.
 * @param manifold Pointer to the b3Manifold structure for storing details of collisions between the plane and the sphere
 * @param plane_a A pointer to the shape of the plane involved in the collision
 * @param xf_a Information about the transformation of the plane, including position and direction
 * @param sphere_b A pointer to the shape of the sphere involved in the collision
 * @param xf_b Information about the transformation of the sphere, including position and direction
 */
void b3_collide_plane_and_sphere(b3Manifold* manifold,
                                 const b3PlaneShape* plane_a,
                                 const b3Transr& xf_a,
                                 const b3SphereShape* sphere_b,
                                 const b3Transr& xf_b);


/**
 * @brief Compute the collision manifold between a plane and a cube.
 * @param manifold Pointer to the b3Manifold structure for storing details of collisions between planes and cubes
 * @param plane_a A pointer to the shape of the plane involved in the collision
 * @param xf_a Information about the transformation of the plane, including position and direction
 * @param cube_b A pointer to the cube shape involved in the collision
 * @param xf_b The transformation information of the cube, including position and orientation
 */
void b3_collide_plane_and_cube(b3Manifold* manifold,
                               const b3PlaneShape* plane_a,
                               const b3Transr& xf_a,
                               const b3CubeShape* cube_b,
                               const b3Transr& xf_b);

#endif //BOX3D_B3_COLLISION_HPP
