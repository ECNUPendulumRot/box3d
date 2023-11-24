#ifndef B3_COLLISION_HPP
#define B3_COLLISION_HPP

#include "collision/b3_fixture.hpp"

namespace box3d {

    bool b3TestOverlap(b3Fixture* fixture_a, b3Fixture* fixture_b);

}


#endif