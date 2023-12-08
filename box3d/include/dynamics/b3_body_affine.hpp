
#ifndef BOX3D_B3_BODY_AFFINE_HPP
#define BOX3D_B3_BODY_AFFINE_HPP

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"
#include "common/b3_types.hpp"

namespace box3d {

    class b3BodyAffine;

}


class box3d::b3BodyAffine: public b3Body
{
    /**
     * @brief The affine transform of the body.
     * q = (p, A1, A2, A3)
     */
    b3Vector12d m_q;

    double m_stiffness;

    double m_potential_orth;

    double m_kinetic_energy;

public:

    b3BodyAffine();

    inline void set_stiffness(double stiffness) {
        m_stiffness = stiffness;
    }

};


#endif //BOX3D_B3_BODY_AFFINE_HPP
