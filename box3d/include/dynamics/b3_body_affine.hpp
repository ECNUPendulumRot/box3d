
#ifndef BOX3D_B3_BODY_AFFINE_HPP
#define BOX3D_B3_BODY_AFFINE_HPP

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"
#include "common/b3_types.hpp"

namespace box3d {

    class b3BodyAffine;
    class b3BodyDefAffine;

}


class box3d::b3BodyDefAffine: public b3BodyDefInner {

    friend class b3BodyAffine;

    /**
     * @brief The density of the rigid body.
     * Unit: kg/m^3
     */
    double m_density = 1.0;

    double m_stiffness = 0;

public:

    b3BodyDefAffine() = default;

    explicit b3BodyDefAffine(double stiffness, double density);

    ~b3BodyDefAffine() override = default;

    static b3BodyDef create_definition(double stiffness, double density);

    b3BodyDefInner* get_def() {
        return this;
    }

};


class box3d::b3BodyAffine: public b3Body {

    friend class b3SolverAffine;

    /**
     * @brief The affine transform of the body.
     * q = (p^T, a1^T, a2^T, a3^T)
     * A = (a1, a2, a3)^T
     */
    b3Vector12d m_q;

    b3Vector12d m_q_dot;

    double m_stiffness = 0;

    double m_density = 1;

    // TODO: check whether the volume is the current one or the deformed one.
    /**
     * @brief The origin volume of the body.
     */
    double m_volume = 0;

    /**
     * @brief The potential is calculated as followed:
     * stiffness * volume * || A * A^T - I||^2_F
     */
    double m_potential_orth = 0;

    double m_kinetic_energy = 0;

    /**
     * @brief Pose of the body of CoM wrt the world frame
     */
    b3PoseD m_pose;

    /**
     * @brief Velocity of CoM the body
     */
    b3PoseD m_velocity;

    /**
     * @brief The affine mass matrix of the body.
     * This will computed after mesh is set.
     */
    b3Matrix12d m_M;

    b3Matrix12d m_inv_M;

public:

    b3BodyAffine();

    explicit b3BodyAffine(const b3BodyDef& body_def);

    inline void set_density(const double& density){
        m_density = density;
    };

    inline void set_stiffness(double stiffness) {
        m_stiffness = stiffness;
    }

    void set_mesh(b3Mesh* mesh) override;

    inline b3Matrix12d get_inv_mass_matrix() const {
        return m_inv_M;
    }

    inline void set_q(const b3Vector12d& q) {
        m_q = q;
    }

    inline void set_q_dot(const b3Vector12d& q_dot) {
        m_q_dot = q_dot;
    }

    inline b3Vector12d get_q() const {
        return m_q;
    }

    inline b3Vector12d get_q_dot() const {
        return m_q_dot;
    }

    b3Vector12d affine_gravity_acc(const b3Vector3d& gravity);

    b3Vector12d get_potential_energy_gradient();

private:

    b3Vector12d orthogonal_potential_gradient();

    bool compute_mass_properties();

    void compute_jacobian_integral(double volume, const b3Inertia& Inertia, const b3PoseD& CoM);

};


#endif //BOX3D_B3_BODY_AFFINE_HPP
