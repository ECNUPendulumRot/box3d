
#include "collision/b3_collision.hpp"
#include "collision/b3_gjk_distance.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_distance_proxy.hpp"

bool box3d::test_gjk_overlap(b3Fixture* fixture_a, b3Fixture* fixture_b) {

    // TODO: set the threshold
    return get_gjk_distance(fixture_a, fixture_b) < 0.001;
}

double box3d::get_gjk_distance(b3Fixture* fixture_a, b3Fixture* fixture_b) {
    
    b3DistanceProxy distance_proxy_a(fixture_a);
    b3DistanceProxy distance_proxy_b(fixture_b);

    GJK gjk(&distance_proxy_a, &distance_proxy_b);

    gjk.evaluate();

    return gjk.get_distance();
}