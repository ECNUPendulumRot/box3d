
#include "solver/b3_contact_solver_gGS_substep.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"

#include "solver/b3_contact_constraint.hpp"

#include <spdlog/spdlog.h>
static bool g_block_solve = false;

b3ContactSolvergGS::b3ContactSolvergGS(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step, bool is_static)
{
    init(block_allocator, island, step, is_static);
}


void b3ContactSolvergGS::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step, bool is_static)
{
    m_is_static = is_static;

    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);
}


static void prepare_contact_sims(b3Island* island, b3ContactSim* css) {
    int32 static_collide = island->m_contact_count;
    for (int32 i = 0; i < static_collide; ++i) {
        b3Contact* c = island->m_contacts[i];
        b3ContactSim* cs = css + i;

        b3Body* body_a = c->m_fixture_a->m_body;
        b3Body* body_b = c->m_fixture_b->m_body;

        cs->body_sim_a = body_a->m_body_sim;
        cs->body_sim_b = body_b->m_body_sim;

        cs->body_sim_a->index = 2*i;
        cs->body_sim_b->index = 2*i+1;

        cs->v_a = cs->body_sim_a->v;
        cs->w_a = cs->body_sim_a->w;
        cs->p_a = cs->body_sim_a->p;
        cs->q_a = cs->body_sim_a->q;

        cs->v_b = cs->body_sim_b->v;
        cs->w_b = cs->body_sim_b->w;
        cs->p_b = cs->body_sim_b->p;
        cs->q_b = cs->body_sim_b->q;

        cs->m_a = body_a->m_mass;
        cs->m_b = body_b->m_mass;

        cs->inv_m_a = body_a->m_inv_mass;
        cs->inv_m_b = body_b->m_inv_mass;

        cs->radius_a = c->m_fixture_a->m_shape->m_radius;
        cs->radius_b = c->m_fixture_b->m_shape->m_radius;

        b3Transr xf_a, xf_b;
        xf_a.set_quaternion(cs->q_a);
        xf_b.set_quaternion(cs->q_b);
        xf_a.set_position(cs->p_a);
        xf_b.set_position(cs->p_b);
        cs->world_manifold.initialize(&c->m_manifold, xf_a, cs->radius_a, xf_b, cs->radius_b);

        cs->restitution = c->m_restitution;
        cs->normal = cs->world_manifold.normal;
        cs->point_count = c->m_manifold.m_point_count;

        for (int32 j = 0; j < cs->point_count; ++j) {
            b3VelocityConstraintPoint *vcp = cs->points + j;

            vcp->m_ra = cs->world_manifold.points[j] - cs->p_a;
            vcp->m_rb = cs->world_manifold.points[j] - cs->p_b;

            b3Vec3r rn_a = vcp->m_ra.cross(cs->normal);
            b3Vec3r rn_b = vcp->m_rb.cross(cs->normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = cs->inv_m_a + cs->inv_m_b +
                       (rn_a * cs->inv_I_a).dot(rn_a) +
                       (rn_b * cs->inv_I_b).dot(rn_b);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = cs->normal.dot(cs->v_b + cs->w_b.cross(vcp->m_rb) - cs->v_a - cs->w_a.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < 0) {
                vcp->m_bias_velocity = -cs->restitution * v_rel;
            }

            vcp->m_normal_impulse = 0.0;
        }
    }
}


void b3ContactSolvergGS::prepare_contact_contraints() {

    m_contact_count = m_island->m_contact_count;
    void* mem = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactSim));
    m_contact_constraints = new (mem) b3ContactSim[m_contact_count];
    prepare_contact_sims(m_island, m_contact_constraints);

}


void b3ContactSolvergGS::solve_velocity_constraints() {
    real tolerance = 0.0;

    int32 wait = 0;

    int32 vel_iteration = m_timestep->m_velocity_iterations;



        for (int32 j = 0; j < vel_iteration; ++j) {

            for (int32 i = 0; i < m_contact_count; ++i) {

                b3ContactSim *cs = m_contact_constraints + i;

                const real& m_a = cs->inv_m_a;
                const real& m_b = cs->inv_m_b;
                const b3Mat33r& I_a = cs->inv_I_a;
                const b3Mat33r& I_b = cs->inv_I_b;

                const b3Vec3r& normal = cs->normal;

                b3Vec3r v_a = cs->body_sim_a->v;
                b3Vec3r w_a = cs->body_sim_a->w;
                b3Vec3r v_b = cs->body_sim_b->v;
                b3Vec3r w_b = cs->body_sim_b->w;

                for (int32 k = 0; k < cs->point_count; ++k) {
                    b3VelocityConstraintPoint* vcp = cs->points + k;

                    b3Vec3r v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));

                    real vn = -v_rel.dot(normal);
                    real v_bias = vcp->m_bias_velocity;
                    //there are four situations of constrains
                    //1.init constrain violate,now legal
                    //2.init constrain violate,now still violate
                    //3.init constrain legal,now legal
                    //4.init constrain legal,now violate
                    if (vn > 0) {
                        if (vcp->m_bias_velocity <= 0) {
                            //4
                            if (abs(vcp->m_relative_velocity - vn) <= tolerance) {
                                //if rhs is converged,change it to situation 2 or 1
                                //vcp->m_bias_velocity = rhs * vc->m_restitution;
                                //bool zero = false;
                                if(!vcp->m_wait){
                                    --wait;
                                    vcp->m_wait = true;
                                    //zero = !(bool)m_wait;
                                    spdlog::info("st:4, index A={},B={} is kicked in m_wait list,now have {} left",
                                                 cs->body_sim_a->index,cs->body_sim_b->index,wait);
                                    //if(m_wait == 0) ++propagations;
                                } else{
                                    spdlog::info("st:4, index A={},B={} is waiting to convert",cs->body_sim_a->index,cs->body_sim_b->index);
                                }
                                if(wait == 0){

                                    //st4 = true;
                                    vcp->m_bias_velocity = vn;
                                    spdlog::info("st:4, constrain {}, {} is converted",cs->body_sim_a->index,cs->body_sim_b->index);
                                }
                                vcp->m_relative_velocity = vn;
                                v_bias = 0.0;
                                vn = 0.0;
                                //问题全在这部分了，考虑少了一种情况，即多物体撞一个物体的情况
                            } else {
                                //have some problem
                                //当多个物体撞击一个物体时，
                                vcp->m_relative_velocity = vn;
                                vn = 0.0;
                                v_bias = 0.0;
                                spdlog::info("st:4, index A={},B={} is waiting",cs->body_sim_a->index,cs->body_sim_b->index);
                                spdlog::info("st:4, relative v={}",vcp->m_relative_velocity);
                                if(vcp->m_wait){
                                    //register a vcp of situation 4 to wait_list
                                    vcp->m_wait = false;
                                    ++wait;
                                    spdlog::info("st:4, index A={},B={} is added in m_wait list,now have {} left",
                                                 cs->body_sim_a->index,cs->body_sim_b->index,wait);
                                }

                            }
                        } else {
                            //2
                            //怎么区分小冲突是来自数值误差还是真实计算？
                            //if(rhs< vc->m_restitution_threshold) rhs_restitution_velocity = 0.0f;
                            spdlog::info("st:2, the violating constrain is {} ,{},v_rel = {},v_bias = {}",cs->body_sim_a->index,cs->body_sim_b->index,vn,v_bias);
                            //spdlog::info("the lambda is {}",vcp->m_normal_mass * (rhs + rhs_restitution_velocity));
                        }
                    } else {
                        if (vcp->m_bias_velocity > 0) {
                            //1
                            if (abs(vcp->m_relative_velocity - vn) <= tolerance || vn == 0.0f) {
                                vcp->m_relative_velocity = vn;
                                //change to situation 3
                                vcp->m_bias_velocity = vn;
                                //不加上这项结果会有问题，因此问题在于考虑转换的条件
                                vn = 0.0;
                                v_bias = 0.0;
                                spdlog::info("st:1, the violated constrain {} ,{} is real legal",cs->body_sim_a->index,cs->body_sim_b->index);
                            } else{
                                violate = true;
                                spdlog::info("st:1, the violated constrain {} ,{} is fake legal",cs->body_sim_a->index,cs->body_sim_b->index);
                                spdlog::info("st:1, the violating constrain is {} ,{},v_rel = {},v_bias = {}",cs->body_sim_a->index,cs->body_sim_b->index,vn,v_bias);
                            }
                        } else {
                            //3
                            if(!vcp->m_wait){
                                vcp->m_wait = true;
                                wait--;
                                spdlog::info("st:3, index A={},B={} is kicked in m_wait list,now have {} left",
                                             cs->body_sim_a->index,cs->body_sim_b->index,wait);
                            }
                            vn = 0.0;
                            v_bias = 0.0;
                        }
                    }

                    real lambda = vcp->m_normal_mass * (vn + v_bias);

                    real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                    lambda = new_impulse - vcp->m_normal_impulse;
                    vcp->m_normal_impulse = new_impulse;

                    b3Vec3r impulse = lambda * normal;
                    v_a = v_a - m_a * impulse;
                    w_a = w_a - I_a * vcp->m_ra.cross(impulse);
                    v_b = v_b + m_b * impulse;
                    w_b = w_b + I_b * vcp->m_rb.cross(impulse);
                    if(vn != 0.0f){
                        v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
                        vcp->m_relative_velocity = -v_rel.dot(normal);
                    }
                }
                cs->body_sim_a->v = v_a;
                cs->body_sim_a->w = w_a;
                cs->body_sim_b->v = v_b;
                cs->body_sim_b->w = w_b;
        }

    }
}