
#ifndef B3_PERSISTENT_MANIFOLD_HPP
#define B3_PERSISTENT_MANIFOLD_HPP

///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.

#include "b3_persistent_manifold_point.hpp"
#include "dynamics/b3_transform.hpp"

class b3Body;


#define MANIFOLD_CACHE_SIZE 4


///  btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///  Those contact points are created by the collision narrow phase.
///  The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///  updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///  reduces the cache to 4 points, when more then 4 points are added, using following rules:
///  the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///  note that some pairs of objects might have more then one contact manifold.

class b3PersistentManifold {

    b3PersistentManifoldPoint m_points[MANIFOLD_CACHE_SIZE];

    int m_point_count;

    real m_contact_breaking_threshold;

    real m_contact_processing_threshold;

    // in dispatch, index
    int m_index;

    b3Body* m_bodyA;
    b3Body* m_bodyB;

    /// sort cached points so most isolated points come first
    int sort_points(const b3PersistentManifoldPoint& point);

public:

    b3PersistentManifold() : m_bodyA(nullptr), m_bodyB(nullptr),
                             m_point_count(0), m_contact_breaking_threshold(0.02) {}

    void set_index(int index) {
        m_index = index;
    }

    int get_index() const {
        return m_index;
    }

    b3PersistentManifold(b3Body* bodyA, b3Body* bodyB, real contact_breaking_threshold) :
    m_bodyA(bodyA), m_bodyB(bodyB), m_point_count(0), m_contact_breaking_threshold(contact_breaking_threshold) {}

    b3Body* get_bodyA() const {
        return m_bodyA;
    }

    b3Body* get_bodyB() const {
        return m_bodyB;
    }

    void set_bodies(b3Body* bodyA, b3Body* bodyB) {
        m_bodyA = bodyA;
        m_bodyB = bodyB;
    }

    int get_contact_point_count() const {
        return m_point_count;
    }

    const b3PersistentManifoldPoint& get_cached_point(int index) const {
        b3_assert(index >= 0 && index < m_point_count);
        return m_points[index];
    }

    b3PersistentManifoldPoint& get_cached_point(int index) {
        b3_assert(index >= 0 && index < m_point_count);
        return m_points[index];
    }

    int get_cache_entry(const b3PersistentManifoldPoint& new_point) const;

    int add_manifold_point(const b3PersistentManifoldPoint& new_point);

    void remove_contact_point(int index);

    void replace_contact_point(const b3PersistentManifoldPoint& new_point, int insert_index);

    real get_contact_breaking_threshold() const {
        return m_contact_breaking_threshold;
    }

    void set_contact_processing_threshold(real contact_processing_threshold) {
        m_contact_processing_threshold = contact_processing_threshold;
    }

    real get_contact_processing_threshold() const {
        return m_contact_processing_threshold;
    }

    bool valid_contact_distance(const b3PersistentManifoldPoint& point) const {
        return point.m_distance <= m_contact_breaking_threshold;
    }

    // calculated new world space coordinates and depth, and reject points that exceed the collision margin
    void refresh_contact_points(const b3Transformr& xfA, const b3Transformr& xfB);

    void clear_manifold() {
        m_point_count = 0;
    }
};

#endif //B3_PERSISTENT_MANIFOLD_HPP