#include "solver/b3_contact_solver_zhb.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "dynamics/b3_body.hpp"
#include "solver/b3_lemke.hpp"
#include "common/b3_time_step.hpp"
#include <spdlog/spdlog.h>

static bool g_block_solve = false;


b3ContactSolverZHB::b3ContactSolverZHB(b3ContactSolverDef *def)
{
    m_step = def->step;
    m_contacts = def->contacts;
    m_count = def->count;
    m_ps = def->ps;
    m_qs = def->qs;
    m_vs = def->vs;
    m_ws = def->ws;
    m_block_allocator = def->block_allocator;


    m_velocity_constraints = (b3ContactVelocityConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactVelocityConstraint));
    m_position_constraints = (b3ContactPositionConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactPositionConstraint));

    for(int32 i = 0; i < m_count; ++i) {

        b3Contact* contact = m_contacts[i];

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();

        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();

        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count > 0);
        //////////////////////////// Velocity Constraints ////////////////////////////
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_friction = contact->get_friction();
        vc->m_contact_index = i;
        vc->m_point_count = point_count;
        vc->m_restitution_threshold = contact->get_restitution_threshold();

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;

        vc->m_index_b = body_b->get_island_index();

        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia();
        vc->m_penetration = manifold->m_penetration;
        vc->m_contact_index = i;

        // the center of body in the world frame
        b3Transr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vec3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vec3r center_b = xf_b.transform(body_b->get_local_center());

        //////////////////////////// Position Constraints ////////////////////////////

        b3ContactPositionConstraint* pc = m_position_constraints + i;
        pc->m_index_a = vc->m_index_a;
        pc->m_index_b = vc->m_index_b;

        pc->m_inv_mass_a = body_a->get_inv_mass();
        pc->m_inv_mass_b = body_b->get_inv_mass();

        pc->m_center_a = center_a;
        pc->m_center_b = center_b;

        pc->m_inv_I_a = vc->m_inv_I_a;
        pc->m_inv_I_b = vc->m_inv_I_b;
        pc->m_local_normal = manifold->m_local_normal;
        pc->m_point_count = point_count;
        pc->m_local_point = manifold->m_local_point;
        pc->m_type = manifold->m_type;
        pc->m_radius_a = fixture_a->get_shape()->get_radius();
        pc->m_radius_b = fixture_b->get_shape()->get_radius();

        for (int32 j = 0; j < point_count; j++) {

            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            b3ManifoldPoint* manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;

            pc->m_local_points[j] = manifold_point->m_local_point;
        }


    }
}


void b3ContactSolverZHB::init_velocity_constraints()
{
    for(int32 i = 0; i < m_count; ++i) {

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        b3ContactPositionConstraint* pc = m_position_constraints + i;
        b3Manifold* manifold = m_contacts[vc->m_contact_index]->get_manifold();
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vec3r p_a = m_ps[index_a];
        b3Quatr q_a = m_qs[index_a];
        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r p_b = m_ps[index_b];
        b3Quatr q_b = m_qs[index_b];
        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        b3Transr xf_a(p_a, q_a);
        b3Transr xf_b(p_b, q_b);

        real radius_a = pc->m_radius_a;
        real radius_b = pc->m_radius_b;

        b3WorldManifold world_manifold;
        world_manifold.initialize(manifold, xf_a, radius_a, xf_b, radius_b);

        vc->m_normal = world_manifold.normal;

        int32 point_count = vc->m_point_count;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            vcp->m_ra = world_manifold.points[j] - p_a;
            vcp->m_rb = world_manifold.points[j] - p_b;

            b3Vec3r ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rb_n = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = vc->m_inv_mass_a + vc->m_inv_mass_b +
                       (ra_n * vc->m_inv_I_a).dot(ra_n) +
                       (rb_n * vc->m_inv_I_b).dot(rb_n);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < -vc->m_restitution_threshold) {
                vcp->m_bias_velocity = -vc->m_restitution * v_rel;
                int32 k = 0;
            }

            vcp->m_normal_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;

        if (point_count > 1 && g_block_solve) {

            b3Vec12r* J = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            b3Vec12r* JW = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            real* mem_JWJT = (real*)m_block_allocator->allocate(point_count * point_count * sizeof(real));
            vc->m_JWJT = (real**)m_block_allocator->allocate(point_count * sizeof(real*));

            // calculate JWJT
            b3Mat1212r W;
            W.set_zero();
            W.set_block(vc->m_inv_mass_a * b3Mat33r::identity(), 0, 0);
            W.set_block(vc->m_inv_I_a, 3, 3);
            W.set_block(vc->m_inv_mass_b * b3Mat33r::identity(), 6, 6);
            W.set_block(vc->m_inv_I_b, 9, 9);

            for (int32 i = 0; i < point_count; ++i) {
                vc->m_JWJT[i] = &mem_JWJT[i * point_count];
            }

            for (int32 i = 0; i < point_count; i++) {
                b3VelocityConstraintPoint* vcp = vc->m_points;
                J[i].set_segment(-vc->m_normal, 0);
                J[i].set_segment(-vcp[i].m_ra.cross(vc->m_normal), 3);
                J[i].set_segment(vc->m_normal, 6);
                J[i].set_segment(vcp[i].m_rb.cross(vc->m_normal), 9);
            }

            for (int32 i = 0; i < point_count; i++) {
                JW[i].set_zero();
                for (int32 j = 0; j < 12; j++) {
                    JW[i] += J[i][j] * W.row(j);
                }
            }

            for (int32 i = 0; i < point_count; ++i) {
                for (int32 j = 0; j < point_count; ++j) {
                    vc->m_JWJT[i][j] = JW[i].dot(J[j]);
                }
            }

            m_block_allocator->free(J, point_count * sizeof(b3Vec12r));
            m_block_allocator->free(JW, point_count * sizeof(b3Vec12r));
        }
    }
}


void b3ContactSolverZHB::solve_velocity_constraints()
{
    real tolerance = 0.001f;
    bool violate = false;
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
            if(rhs>0 && !violate) violate = true;
            real rhs_restitution_velocity = vcp->m_bias_velocity;

            //there are four situations of constrains
            //1.init constrain violate,now legal
            //2.init constrain violate,now still violate
            //3.init constrain legal,now legal
            //4.init constrain legal,now violate
            if (rhs > 0) {
                if (rhs_restitution_velocity <= 0) {
                    //4
                    if (abs(vcp->m_relative_velocity - rhs) <= tolerance) {
                        //if rhs is converged,change it to situation 2 or 1
                        //vcp->m_bias_velocity = rhs * vc->m_restitution;
                        //bool zero = false;
                        if(!vcp->wait){

                            --m_wait;
                            vcp->wait = true;
                            //zero = !(bool)m_wait;
                            spdlog::info("st:4, index A={},B={} is added in wait list,now have {} left",
                                         vc->m_index_a,vc->m_index_b,m_wait);
                        } else{
                            spdlog::info("st:4, index A={},B={} is waiting to convert",vc->m_index_a,vc->m_index_b);
                        }
                        if(m_wait == 0){
                            vcp->m_bias_velocity = rhs * vc->m_restitution;
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
                        spdlog::info("st:4, index A={} is waiting",vc->m_index_a);
                        spdlog::info("st:4, index B={} is waiting",vc->m_index_b);
                        if(vcp->wait){
                            //register a vcp of situation 4 to wait_list
                            vcp->wait = false;
                            ++m_wait;
                            spdlog::info("st:4, wait is true now");
                        }

                    }
                } else {
                    //2
                    spdlog::info("st:2, the violating constrain is {} ,{}",vc->m_index_a,vc->m_index_b);
                }
            } else {
                if (rhs_restitution_velocity > 0) {
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
                    }
                } else {
                    //3
                    if(!vcp->wait){
                        vcp->wait = true;
                        m_wait--;
                        spdlog::info("st:3, index A={},B={} is kicked in wait list,now have {} left",
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
            }
        }


        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;
    }
    if(violate) spdlog::info("Iteration: {0} is over", iteration++);
}


struct b3PositionSolverManifold {

    void initialize(b3ContactPositionConstraint* pc, const b3Transr& xf_a, const b3Transr& xf_b, int32 index) {

        b3_assert(pc->m_point_count > 0);

        switch (pc->m_type)
        {
            case b3Manifold::e_spheres: {
                b3Vec3r point_a = xf_a.position();
                b3Vec3r point_b = xf_b.position();
                normal = pc->m_local_normal;
                point = 0.5 * (point_a + point_b);
                separation = (point_b - point_a).dot(normal) - pc->m_radius_a - pc->m_radius_b;
            }
                break;

            case b3Manifold::e_face_A:
            {
                normal = pc->m_local_normal;

                b3Vec3r plane_point = pc->m_local_point;

                b3Vec3 clipPoint = pc->m_local_points[index];
                separation = (clipPoint - plane_point).dot(normal) - pc->m_radius_a - pc->m_radius_b;
                point = clipPoint;
            }
                break;

            case b3Manifold::e_face_B:
            {
                normal = pc->m_local_normal;
                b3Vec3r plane_point = pc->m_local_point;

                b3Vec3 clipPoint = pc->m_local_points[index];
                separation = (clipPoint - plane_point).dot(normal) - pc->m_radius_a - pc->m_radius_b;
                point = clipPoint;
            }
                break;

            default:
                break;

        }
    }

    b3Vec3r normal;
    b3Vec3r point;
    float separation;
};


b3ContactSolverZHB::~b3ContactSolverZHB()
{
    for (int32 i = 0; i < m_count; ++i) {
        const int32& point_count = m_velocity_constraints[i].m_point_count;
        if (point_count > 1 && g_block_solve) {
            m_block_allocator->free(m_velocity_constraints[i].m_JWJT[0], point_count * point_count * sizeof(real));
            m_block_allocator->free(m_velocity_constraints[i].m_JWJT, point_count * sizeof(real*));
        }
    }

    m_block_allocator->free(m_velocity_constraints, m_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_position_constraints, m_count * sizeof(b3ContactPositionConstraint));
}


