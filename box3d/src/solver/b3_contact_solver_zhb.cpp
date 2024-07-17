// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "solver/b3_contact_solver_zhb.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "dynamics/b3_body.hpp"
#include "solver/b3_lemke.hpp"
#include "common/b3_time_step.hpp"
#include <spdlog/spdlog.h>

/**
 * @brief b3ContactSolverZHB: This class is responsible for solving contact constraints between bodies during the simulation.
 */
b3ContactSolverZHB::b3ContactSolverZHB(b3ContactSolverDef *def)
{
    // Initialize member variables with the definition provided
    m_step = def->step;
    m_contacts = def->contacts;
    m_count = def->count;

    m_ps = def->ps;
    m_qs = def->qs;
    m_vs = def->vs;
    m_ws = def->ws;
    m_block_allocator = def->block_allocator;

    // Allocate memory for velocity and position constraints
    m_velocity_constraints = (b3ContactVelocityConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactVelocityConstraint));
    m_position_constraints = (b3ContactPositionConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactPositionConstraint));

    // Initialize constraints for each contact
    for(int32 i = 0; i < m_count; ++i) {

        b3Contact* contact = m_contacts[i];

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();
        b3Shape* shape_a = fixture_a->get_shape();
        b3Shape* shape_b = fixture_b->get_shape();
        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();
        real radius_a = shape_a->get_radius();
        real radius_b = shape_b->get_radius();
        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;
        b3_assert(point_count > 0);

        //////////////////////////// Velocity Constraints ////////////////////////////
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_friction = contact->get_friction();
        vc->m_restitution = contact->get_restitution();
        vc->m_restitution_threshold = contact->get_restitution_threshold();

        vc->m_index_a = body_a->get_island_index();
        vc->m_index_b = body_b->get_island_index();
        vc->m_inv_mass_a = body_a->get_inv_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;
        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia() * R_b;

        vc->m_point_count = point_count;
        vc->m_contact_index = i;

        //////////////////////////// Position Constraints ////////////////////////////
        b3ContactPositionConstraint* pc = m_position_constraints + i;
        pc->m_index_a = vc->m_index_a;
        pc->m_index_b = vc->m_index_b;
        pc->m_inv_mass_a = vc->m_inv_mass_a;
        pc->m_inv_mass_b = vc->m_inv_mass_b;
        pc->m_local_center_a = body_a->get_local_center();
        pc->m_local_center_b = body_b->get_local_center();
        pc->m_inv_I_a = vc->m_inv_I_a;
        pc->m_inv_I_b = vc->m_inv_I_b;
        pc->m_local_normal = manifold->m_local_normal;
        pc->m_local_point = manifold->m_local_point;
        pc->m_point_count = point_count;
        pc->m_radius_a = radius_a;
        pc->m_radius_b = radius_b;
        pc->m_type = manifold->m_type;

        for (int32 j = 0; j < point_count; j++) {

            b3ManifoldPoint* manifold_point = manifold->m_points + j;
            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            vcp->m_ra.set_zero();
            vcp->m_rb.set_zero();

            vcp->m_normal_mass = 0.0;
            vcp->m_bias_velocity = 0.0;
            vcp->m_normal_impulse = 0.0;

            vcp->m_wait = true;
            pc->m_local_points[j] = manifold_point->m_local_point;
        }
    }
}

/**
 * @brief Initializes velocity constraints for each contact.
 */
void b3ContactSolverZHB::init_velocity_constraints()
{
    for(int32 i = 0; i < m_count; ++i) {

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        b3ContactPositionConstraint* pc = m_position_constraints + i;

        real radius_a = pc->m_radius_a;
        real radius_b = pc->m_radius_b;
        b3Manifold* manifold = m_contacts[vc->m_contact_index]->get_manifold();

        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        const real& m_a = vc->m_inv_mass_a;
        const real& m_b = vc->m_inv_mass_b;
        const b3Mat33r& I_a = vc->m_inv_I_a;
        const b3Mat33r& I_b = vc->m_inv_I_b;
        const b3Vec3r& local_center_a = pc->m_local_center_a;
        const b3Vec3r& local_center_b = pc->m_local_center_b;

        b3Vec3r p_a = m_ps[index_a];
        b3Quatr q_a = m_qs[index_a];
        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r p_b = m_ps[index_b];
        b3Quatr q_b = m_qs[index_b];
        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        b3Transr xf_a, xf_b;
        xf_a.set_quaternion(q_a);
        xf_b.set_quaternion(q_b);
        xf_a.set_position(p_a - xf_a.rotate(local_center_a));
        xf_b.set_position(p_b - xf_b.rotate(local_center_b));

        b3WorldManifold world_manifold;
        world_manifold.initialize(manifold, xf_a, radius_a, xf_b, radius_b);

        vc->m_normal = world_manifold.normal;

        int32 point_count = vc->m_point_count;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            vcp->m_ra = world_manifold.points[j] - p_a;
            vcp->m_rb = world_manifold.points[j] - p_b;

            b3Vec3r rn_a = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rn_b = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = m_a + m_b +
                       (rn_a * I_a).dot(rn_a) +
                       (rn_b * I_b).dot(rn_b);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_bias_velocity is Jv
            vcp->m_bias_velocity = -v_rel;

        }
    }
}

/**
 * @brief Solves velocity constraints for each contact.
 * @param violate Indicates if any constraints are violated.
 * @param propagations Number of constraint propagations.
 */
void b3ContactSolverZHB::solve_velocity_constraints(bool &violate, int32 &propagations)
{
    real tolerance = 0.0;
    bool st4=false;
    for (int32 i = 0; i < m_count; ++i) {

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];

        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];

        for (int32 j = 0; j < vc->m_point_count; ++j) {

            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
            real rhs = -v_rel.dot(vc->m_normal);
            if(rhs > 0 && !violate) violate = true;
            real rhs_restitution_velocity = vcp->m_bias_velocity * vc->m_restitution;

            //there are four situations of constrains
            //1.init constrain violate,now legal
            //2.init constrain violate,now still violate
            //3.init constrain legal,now legal
            //4.init constrain legal,now violate
            if (rhs > 0) {
                if (vcp->m_bias_velocity <= 0) {
                    //4
                    if (abs(vcp->m_relative_velocity - rhs) <= tolerance) {
                        //if rhs is converged,change it to situation 2 or 1
                        //vcp->m_bias_velocity = rhs * vc->m_restitution;
                        //bool zero = false;
                        if(!vcp->m_wait){

                            --m_wait;
                            vcp->m_wait = true;
                            //zero = !(bool)m_wait;
                            spdlog::info("st:4, index A={},B={} is kicked in m_wait list,now have {} left",
                                         vc->m_index_a,vc->m_index_b,m_wait);
                            if(m_wait == 0) ++propagations;
                        } else{
                            spdlog::info("st:4, index A={},B={} is waiting to convert",vc->m_index_a,vc->m_index_b);
                        }
                        if(m_wait == 0){

                            st4 = true;
                            vcp->m_bias_velocity = rhs;
                            spdlog::info("st:4, constrain {}, {} is converted",vc->m_index_a,vc->m_index_b);
                        }
                        vcp->m_relative_velocity = rhs;
                        rhs_restitution_velocity = 0.0;
                        rhs = 0.0;
                        //问题全在这部分了，考虑少了一种情况，即多物体撞一个物体的情况
                    } else {
                        //have some problem
                        //当多个物体撞击一个物体时，
                        vcp->m_relative_velocity = rhs;
                        rhs = 0.0;
                        rhs_restitution_velocity = 0.0;
                        spdlog::info("st:4, index A={},B={} is waiting",vc->m_index_a,vc->m_index_b);
                        spdlog::info("st:4, relative v={}",vcp->m_relative_velocity);
                        if(vcp->m_wait){
                            //register a vcp of situation 4 to wait_list
                            vcp->m_wait = false;
                            ++m_wait;
                            spdlog::info("st:4, index A={},B={} is added in m_wait list,now have {} left",
                                         vc->m_index_a,vc->m_index_b,m_wait);
                        }

                    }
                } else {
                    //2
                    //怎么区分小冲突是来自数值误差还是真实计算？
                    //if(rhs< vc->m_restitution_threshold) rhs_restitution_velocity = 0.0f;
                    spdlog::info("st:2, the violating constrain is {} ,{},v_rel = {},v_bias = {}",vc->m_index_a,vc->m_index_b,rhs,rhs_restitution_velocity);
                    //spdlog::info("the lambda is {}",vcp->m_normal_mass * (rhs + rhs_restitution_velocity));
                }
            } else {
                if (vcp->m_bias_velocity > 0) {
                    //1
                    if (abs(vcp->m_relative_velocity - rhs) <= tolerance || rhs == 0.0f) {
                        vcp->m_relative_velocity = rhs;
                        //change to situation 3
                        vcp->m_bias_velocity = rhs;
                        //不加上这项结果会有问题，因此问题在于考虑转换的条件
                        rhs = 0.0;
                        rhs_restitution_velocity = 0.0;
                        spdlog::info("st:1, the violated constrain {} ,{} is real legal",vc->m_index_a,vc->m_index_b);
                    } else{
                        violate = true;
                        spdlog::info("st:1, the violated constrain {} ,{} is fake legal",vc->m_index_a,vc->m_index_b);
                        spdlog::info("st:1, the violating constrain is {} ,{},v_rel = {},v_bias = {}",vc->m_index_a,vc->m_index_b,rhs,rhs_restitution_velocity);
                    }
                } else {
                    //3
                    if(!vcp->m_wait){
                        vcp->m_wait = true;
                        m_wait--;
                        spdlog::info("st:3, index A={},B={} is kicked in m_wait list,now have {} left",
                                     vc->m_index_a,vc->m_index_b,m_wait);
                    }
                    rhs = 0.0;
                    rhs_restitution_velocity = 0.0;
                }
            }

            real lambda = 0;

            lambda = vcp->m_normal_mass * (rhs + rhs_restitution_velocity);
            real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
            lambda = new_impulse - vcp->m_normal_impulse;
            //spdlog::info("lambda = {}",lambda);
            vcp->m_normal_impulse = new_impulse;


            // apply normal Impulse
            b3Vec3r impulse = lambda * vc->m_normal;

            v_a = v_a - vc->m_inv_mass_a * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
            if (rhs != 0.0f) {
                //delete these seems have no effect to solver?why?
                v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
                vcp->m_relative_velocity = -v_rel.dot(vc->m_normal);

                if(vcp->m_relative_velocity == rhs) violate = false;
                spdlog::info("after resolve, the v_rel is{}",vcp->m_relative_velocity);
                //if(vcp->m_relative_velocity+rhs<b3_real_epsilon) spdlog::info("reverse!v={}",vcp->m_relative_velocity+rhs);
            }
        }


        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;
        /*if(violate){
            spdlog::info("v_{} = ({},{},{})",vc->m_index_a,v_a.x,v_a.y,v_a.z);
            spdlog::info("v_{} = ({},{},{})",vc->m_index_b,v_b.x,v_b.y,v_b.z);
        }*/

    }
    if(violate) spdlog::info("Iteration: {0} is over", iteration++);
    //if(st4) violate = false;//保证局部解对称性
    /*if(m_wait==0&&violate==false) violate = false;
    else violate = true;*/
}

/**
 * @brief Destroys the contact solver and releases allocated memory.
 */
b3ContactSolverZHB::~b3ContactSolverZHB()
{
    m_block_allocator->free(m_velocity_constraints, m_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_position_constraints, m_count * sizeof(b3ContactPositionConstraint));
}


