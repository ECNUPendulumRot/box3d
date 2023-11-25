
#include "b3_collision.hpp"
#include "b3_gjk_distance.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_distance_proxy.hpp"

bool box3d::b3_gjk_test_overlap(b3Fixture* fixture_a, b3Fixture* fixture_b) {
    
    b3DistanceProxy distance_proxy_a(fixture_a);
    b3DistanceProxy distance_proxy_b(fixture_b);

    GJK gjk(&distance_proxy_a, &distance_proxy_b);

    gjk.evaluate();

    // TODO: set the threshold
    return gjk.get_distance() < 0.001;
}
