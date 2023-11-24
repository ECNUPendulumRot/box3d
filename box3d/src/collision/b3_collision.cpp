
#include "b3_collision.hpp"
#include "b3_distance.hpp"

bool box3d::b3TestOverlap(b3Fixture* fixture_a, b3Fixture* fixture_b) {
    
    b3DistanceInput input(fixture_a, fixture_b);
    b3DistanceOutput output;

    b3Distance(&input, &output);

    // set the threshold
    return output.get_distance() < 1e-06;
}
