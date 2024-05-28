
#ifndef B3_MANIFOLD_RESULT_HPP
#define B3_MANIFOLD_RESULT_HPP

#include "math/b3_vec3.hpp"

class b3PersistentManifold;
class b3Body;

// b3ManifoldResult is a helper class to manage contact results.
class b3ManifoldResult {

private:

    b3PersistentManifold* m_manifold;

    const b3Body* m_bodyA;
    const b3Body* m_bodyB;

    // TODO:
    int m_part_idA;
    int m_part_idB;

    int m_indexA;
    int m_indexB;

public:

    b3ManifoldResult(const b3Body* bodyA, const b3Body* bodyB);

    void set_persistent_manifold(b3PersistentManifold* manifold) {
        m_manifold = manifold;
    }

    b3PersistentManifold* get_persistent_manifold() const {
        return m_manifold;
    }

    /**
     * @param normal : in the world frame, from A to B
     * @param pointA the point of A in world frame
     * so the point of B is point_in_world + normal * depth
     * @param depth  < 0
     */
    void add_contact_point(const b3Vec3r& normal, const b3Vec3r& pointA, real depth);

    void refresh_contact_points();

};


#endif