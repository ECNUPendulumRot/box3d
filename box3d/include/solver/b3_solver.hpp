
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP


namespace box3d {

    class b3Solver;

    ///////////////////

    class b3World;

}


class box3d::b3Solver {

public:

    virtual void initialize(b3World* world) = 0;

    virtual int solve(double delta_t) = 0;
    
};

#endif //BOX3D_B3_SOLVER_HPP
