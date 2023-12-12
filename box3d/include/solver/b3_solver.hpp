
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP


namespace box3d {

    class b3Solver;

}


class box3d::b3Solver {

public:

    virtual int solve() = 0;
    
};

#endif //BOX3D_B3_SOLVER_HPP
