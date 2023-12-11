#ifndef B3_COLLISION_HPP
#define B3_COLLISION_HPP

#include "collision/b3_fixture.hpp"

namespace box3d {

    bool test_gjk_overlap(b3Fixture* fixture_a, b3Fixture* fixture_b);

    double get_gjk_distance(b3Fixture* fixture_a, b3Fixture* fixture_b);
}


#endif