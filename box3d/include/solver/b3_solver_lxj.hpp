
#ifndef B3_SOLVER_LXJ_HPP
#define B3_SOLVER_LXJ_HPP

#include "b3_solver_body.hpp"
#include "b3_solver_constraint.hpp"
#include "dynamics/constraint/b3_constraint_base.hpp"

#include <vector>


class b3Island;
class b3TimeStep;
class b3PersistentManifold;
struct b3PersistentManifoldPoint;


class b3SolverLxj {

protected:

    std::vector<b3SolverBody> m_tmp_solver_bodies_pool;
    std::vector<b3SolverConstraint> m_tmp_solver_contact_constraint_pool;
    std::vector<b3SolverConstraint> m_tmp_solver_contact_friction_constraint_pool;
    std::vector<b3SolverConstraint> m_tmp_solver_rolling_friction_constraint_pool;
    std::vector<b3SolverConstraint> m_tmp_solver_non_contact_constraint_pool;

//    std::vector<int> m_order_temp_constraint_pool;
//    std::vector<int> m_order_friction_constraint_pool;

    std::vector<b3ConstraintBase::b3ConstraintInfo1> m_tmp_constraint_sizes_pool;

    int m_max_override_num_solver_iterations;
    int m_fixed_body_id;
    real m_least_squares_residual;

    static real restitution_curve(real rel_vel, real restitution, real velocity_threshold);

    void setup_friction_constraint(b3SolverConstraint& solver_constraint, const b3Vec3r& normal_axis, int solver_body_idA, int solver_body_idB,
                                   b3PersistentManifoldPoint& cp, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB, real relaxation);
    void setup_torsional_friction_constraint(b3SolverConstraint& solver_constraint, const b3Vec3r& normal_axis1, int solver_body_idA, int solver_body_idB,
                                             b3PersistentManifoldPoint& cp, real torsional_friction);

    b3SolverConstraint& add_friction_constraint(const b3Vec3r& normal_axis, int solver_body_idA, int solver_body_idB, int friction_index,
                                                b3PersistentManifoldPoint& cp, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB, real relaxation);
    b3SolverConstraint& add_torsional_friction_constraint(const b3Vec3r& normal_axis, int solver_body_idA, int solver_body_idB, int friction_index,
                                                          b3PersistentManifoldPoint& cp, real torsional_friction);

    void convert_bodies(b3Island* island, b3TimeStep* time_step);
    void convert_contacts(b3Island* island, b3TimeStep* time_step);
    void convert_contact(b3PersistentManifold* manifold, b3TimeStep* time_step);
    void convert_joints(b3Island* island, b3TimeStep* time_step);
    void convert_joint(b3SolverConstraint* current_constraint_row, b3ConstraintBase* constraint, const b3ConstraintBase::b3ConstraintInfo1& info1,
                       int solver_body_idA, int solver_body_idB, b3TimeStep* time_step);

    void setup_contact_constraint(b3SolverConstraint& solver_constraint, int solver_body_idA, int solver_body_idB, b3PersistentManifoldPoint& cp,
                                  const b3TimeStep* time_step, real& relaxation, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB);

    static void init_solver_body(b3SolverBody& solver_body, b3Body* body, real dt);

    int get_or_init_solver_body(b3Body& body, real dt);

    void solve_group_setup(b3Island* island, b3TimeStep* time_step);
    void solve_group_iterations(b3Island* island, b3TimeStep* time_step);
    void solve_group_finish(b3TimeStep* time_step);
    void solve_group_positions(b3TimeStep* time_step);
    real solver_single_iteration(b3Island* island, b3TimeStep* time_step);

    real resolve_penetration_impulse(b3SolverBody& bodyA, b3SolverBody& bodyB, b3SolverConstraint& c);

    void write_back_bodies(real dt);
    void write_back_contacts(b3TimeStep* time_step);

public:

    void solve_group(b3Island* island, b3TimeStep* time_step);

};

#endif