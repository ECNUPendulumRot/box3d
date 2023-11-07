
#ifndef BOX3D_B3_INERTIA_HPP
#define BOX3D_B3_INERTIA_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_pose.hpp"

namespace box3d {

    class b3Inertia;

}


class box3d::b3Inertia {

    b3Matrix3d m_I;

    b3PoseD* m_rel_p;

public:

    /**
     * @brief Construct a new b3Inertia object
     */
    b3Inertia();

    inline void set_relative_pose(b3PoseD* rel_pose){
        m_rel_p = rel_pose;
    };

    inline void set_inertia(double xx, double yy, double zz, double xy, double xz, double yz){
        m_I(0, 0) = xx;
        m_I(0, 1) = xy;
        m_I(0, 2) = xz;
        m_I(1, 0) = m_I(0, 1);
        m_I(1, 1) = yy;
        m_I(1, 2) = yz;
        m_I(2, 0) = m_I(0, 2);
        m_I(2, 1) = m_I(1, 2);
        m_I(2, 2) = zz;
    };

    inline void set_inertia(const b3Matrix3d& I){
        m_I = I;
    };

};



#endif //BOX3D_B3_INERTIA_HPP
