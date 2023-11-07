
#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP


#include "dynamics/b3_body.hpp"

namespace box3d {

    class b3World;

}

class box3d::b3World {

    b3Body* m_body_list;

    int32 m_body_count;

    b3Vector3d m_gravity;

public:

    b3World();

};


#endif //BOX3D_B3_WORLD_HPP
