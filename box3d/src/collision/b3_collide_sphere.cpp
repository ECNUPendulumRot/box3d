
#include "collision/b3_collision.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"


void b3_collide_spheres(box3d::b3Manifold* manifold,
					    const box3d::b3SphereShape* sphere_a, 
                        const b3TransformD& xf_a,
					    const box3d::b3SphereShape* sphere_b, 
                        const b3TransformD& xf_b) {
    
    manifold->point_count = 0;

    b3Vector3d ca = xf_a.transform(sphere_a->get_centroid_of_sphere());
    b3Vector3d cb = xf_b.transform(sphere_b->get_centroid_of_sphere());

    b3Vector3d ab = cb - ca;

    double sq_distance = ab.dot(ab);
    double radius = sphere_a->get_radius() + sphere_b->get_radius();

    if(sq_distance > radius * radius) {
        // two spheres are not collide.
        return;
    }

    manifold->type = manifold->e_circles;
    manifold->point_count = 1;
    manifold->local_normal = ab.normalized();

    double penetration = b3_sqrt(sq_distance) - radius;
    
    double length = sphere_a->get_radius() + penetration / 2;

    // the collide point on the world frame
    manifold->points[0].m_local_point = ca + manifold->local_normal * length;
    manifold->points[0].id.key = 0;
    manifold->m_penetration = penetration;
}


void b3_collide_cube_and_sphere(box3d::b3Manifold* manifold,
                                const box3d::b3CubeShape* cube_a,
                                const b3TransformD& xf_a,
                                const box3d::b3SphereShape* sphere_b,
                                const b3TransformD& xf_b) {
    
    manifold->point_count = 0;

    /// shallow penetration and deep penetration
    /// the center of sphere is in the cube ?

    /// Compute the center of sphere position on cube frame
    b3Vector3d c_local = xf_b.transform(sphere_b->get_centroid_of_sphere());
    // c_local = xf_a.transform_local(c_local);

    // traverse six face normal to find 

}