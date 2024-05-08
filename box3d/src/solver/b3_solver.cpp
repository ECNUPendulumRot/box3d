
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"

#include "math/b3_mat1212.hpp"
#include <algorithm>
#include <sstream>

#include <spdlog/spdlog.h>
#include <iomanip>
#include "spdlog/sinks/stdout_color_sinks.h"


static void eliminate(real** tableau, real** I_inv, real* b, int32 i, int32 j, int32 size);


static auto s_pp_logger = spdlog::stdout_color_mt("pp");
static auto s_p_logger = spdlog::stdout_color_mt("p");

struct LexicoInv {
    real b_value = 0.0;
    real* inv_row = nullptr;
    real piv_value = 0.0;
    static int32 size;

    bool operator<(const LexicoInv& other) const {
        if (this->piv_value <= 0)
            return false;
        if (other.piv_value <= 0)
            return true;

        if (this->b_value / this->piv_value < other.b_value / other.piv_value) {
            return true;
        }

        for (int32 i = 0; i < size; ++i) {
            if (this->inv_row[i] / this->piv_value < other.inv_row[i] / other.piv_value) {
                return true;
            }
        }
        return false;
    }

};


int32 LexicoInv::size = 0;

#define DEBUG_PRINT(iteration, tableau, I_inv, b, size, pivot) \
    spdlog::info("////////////////////////////////////////////////////"); \
    spdlog::info("iteration {}\n", iteration);              \
    print_2d_array(tableau, size, 2 * size + 1, "tableau"); \
    print_array(b, size, "b vector");                       \
    print_2d_array(I_inv, size, size, "inv of identity");   \
    print_array(pivot, size, "pivot");

static void print_array(int32* array, int32 size, const char* s) {
    auto p_logger = spdlog::get("p");
    p_logger->set_pattern("%v");
    p_logger->info("current array: {}\n", s);
    std::ostringstream oss;
    for (int32 i = 0; i < size; ++i) {
        oss << array[i] << " ";
    }
    p_logger->info(oss.str());
}

static void print_array(real* array, int32 size, const char* s) {
    auto p_logger = spdlog::get("p");
    p_logger->set_pattern("%v");
    p_logger->info("current array: {}\n", s);
    std::ostringstream oss;
    for (int32 i = 0; i < size; ++i) {
        oss << std::fixed << std::setprecision(4) << array[i] << " ";
    }
    p_logger->info(oss.str());
}

static void print_2d_array(real** array, int32 row, int32 col, const char* s) {
    auto pp_loger = spdlog::get("pp");
    pp_loger->set_pattern("%v");
    spdlog::info("current mat: {}\n", s);

    for (int32 i = 0; i < row; ++i) {
        std::ostringstream oss;
        for (int32 j = 0; j < col; ++j) {
            oss <<  std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(4) << array[i][j] <<" ";
        }
        pp_loger->info(oss.str());
    }
}

static void print_12_array(const b3Mat1212r& array) {
    auto matrix_logger = spdlog::get("pp");
    matrix_logger->set_pattern("%v");
    spdlog::info("current inv mass:\n");


    for (int32 i = 0; i < 12; ++i) {
        std::ostringstream oss;
        for (int32 j = 0; j < 12; ++j) {
            oss << std::fixed << std::setprecision(4) << array.m_ts[j][i] << " ";
        }
        matrix_logger->info(oss.str());
    }
}

static void print_12_array(const b3Vec12r* array, const int32& size, const char* s) {
    auto matrix_logger = spdlog::get("pp");
    matrix_logger->set_pattern("%v");
    spdlog::info("current mat: {} \n", s);


    for (int32 i = 0; i < size; ++i) {
        std::ostringstream oss;
        for (int32 j = 0; j < 12; ++j) {
            oss << std::fixed << std::setprecision(4) << array[i][j] << " ";
        }
        matrix_logger->info(oss.str());
    }
}


static void reset_I(real** I_inv, int32 size) {
    real* start = I_inv[0];
    memset(start, 0, size * size * sizeof(real));
    for (int32 i = 0; i < size; ++i) {
        I_inv[i][i] = 1;
    }
}


real* lemke(b3ContactVelocityConstraint* vc, b3BlockAllocator *block_allocator);

b3Solver::b3Solver(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    init(block_allocator, island, step);
}


void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_method = step->m_integral_method;
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);

    // allocate memory for all bodies and contacts of every island.
    // and we need extra arrays to store the velocity and position of every body.
    // after solver all contacts and friction constraints, we copy the results back to bodies.

    // The number of velocity constraints is same to the number of contacts.
    void* memory;
    if(m_contact_count > 0) {
        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactVelocityConstraint));
        m_velocity_constraints = new (memory) b3ContactVelocityConstraint;
    } else {
        m_velocity_constraints = nullptr;
    }

    m_body_count = island->get_body_count();
    b3_assert(m_body_count > 0);
    m_bodies = island->get_bodies();

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_ps = new (memory) b3Vec3r;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Quaternionr));
    m_qs = new (memory) b3Quaternionr;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_vs = new (memory) b3Vec3r;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_ws = new (memory) b3Vec3r;

    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        m_ps[i] = b->get_position();
        m_qs[i] = b->get_quaternion();
        m_vs[i] = b->get_linear_velocity();
        m_ws[i] = b->get_angular_velocity();
    }

    ////////////////////////////////////////////////////////////////////////////////

    for(int32 i = 0; i < m_contact_count; ++i) {
        b3Contact* contact = m_contacts[i];

        b3Fixture *fixture_a = contact->get_fixture_a();
        b3Fixture *fixture_b = contact->get_fixture_b();

        b3Body *body_a = fixture_a->get_body();
        b3Body *body_b = fixture_b->get_body();

        b3Manifold *manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count > 0);

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_friction = contact->get_friction();
        vc->m_contact_index = i;
        vc->m_point_count = point_count;

        vc->m_normal = manifold->m_local_normal;

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a * body_a->get_inv_inertia() * R_a.transpose();

        vc->m_index_b = body_b->get_island_index();

        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b * body_b->get_inv_inertia() * R_b.transpose();

        vc->m_penetration = manifold->m_penetration;

        // the center of body in the world frame

        b3Transformr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transformr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vec3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vec3r center_b = xf_b.transform(body_b->get_local_center());

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;
            b3ManifoldPoint *manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;
            // vcp->m_rhs_penetration = manifold->m_penetration;
            // TODO: warm start
        }
    }

}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        m_bodies[i]->set_position(m_ps[i]);
        m_bodies[i]->set_quaternion(m_qs[i]);
        m_bodies[i]->set_linear_velocity(m_vs[i]);
        m_bodies[i]->set_angular_velocity(m_ws[i]);

        spdlog::info("i = {};v_a: {}", i, m_vs[i].z);
        // TODO: if we need SynchronizeTransform() ?
    }
}


b3Solver::~b3Solver()
{
    clear();
}


void b3Solver::clear() {

    m_timestep = nullptr;
    m_contacts = nullptr;

    for (int32 i = 0; i < m_contact_count; ++i) {
        const int32& point_count = m_velocity_constraints[i].m_point_count;
        m_block_allocator->free(m_velocity_constraints[i].m_JWJT, point_count * sizeof(real*));
        m_block_allocator->free(m_velocity_constraints[i].m_mem_JWJT, point_count * point_count * sizeof(real));
    }

    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quaternionr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));

    m_block_allocator = nullptr;
}

int b3Solver::solve() {

    // velocity update
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body *b = m_bodies[i];
        b3Vec3r v = m_vs[i];
        b3Vec3r w = m_ws[i];

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        m_vs[i] = v;
        m_ws[i] = w;
    }

    init_velocity_constraints();

    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }

    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3Solver::init_velocity_constraints()
{
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        for (int j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

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
            vcp->m_bias_velocity = -vc->m_restitution * v_rel;
            vcp->m_vn = v_rel;
            vcp->m_normal_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;


        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;

        const int32 point_count = vc->m_point_count;

        if (point_count > 1) {

            b3Vec12r* J = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            b3Vec12r* JW = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            vc->m_mem_JWJT = (real*)m_block_allocator->allocate(point_count * point_count * sizeof(real));
            vc->m_JWJT = (real**)m_block_allocator->allocate(point_count * sizeof(real**));

            // calculate JWJT
            b3Mat1212r W;
            W.set_zero();
            W.set_block(vc->m_inv_mass_a * b3Mat33r::identity(), 0, 0);
            W.set_block(vc->m_inv_I_a, 3, 3);
            W.set_block(vc->m_inv_mass_b * b3Mat33r::identity(), 6, 6);
            W.set_block(vc->m_inv_I_b, 9, 9);

            print_12_array(W);

            for (int32 i = 0; i < point_count; ++i) {
                vc->m_JWJT[i] = &vc->m_mem_JWJT[i * point_count];
            }

            for (int32 i = 0; i < point_count; i++) {
                auto* vcp = vc->m_points;
                J[i].set_segment(-vc->m_normal, 0);
                J[i].set_segment(-vcp[i].m_ra.cross(vc->m_normal), 3);
                J[i].set_segment(vc->m_normal, 6);
                J[i].set_segment(vcp[i].m_rb.cross(vc->m_normal), 9);
            }

            print_12_array(J, point_count, "J");


            for (int32 i = 0; i < point_count; i++) {
                JW[i].set_zero();
                for (int32 j = 0; j < 12; j++) {
                    JW[i] += J[i][j] * W.row(j);
                }
            }

            for (int32 i = 0; i < point_count; ++i) {
                for (int32 j = 0; j < point_count; ++j) {
                    vc->m_JWJT[i][j] = -JW[i].dot(J[j]);
                }
            }
            print_2d_array(vc->m_JWJT, point_count, point_count, "JWJT");

            m_block_allocator->free(J, point_count * sizeof(b3Vec12r));
            m_block_allocator->free(JW, point_count * sizeof(b3Vec12r));
        }
        // TODO: if we have more than one contact point, then prepare the block solver.
    }
}


void b3Solver::solve_velocity_constraints(bool is_collision)
{
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        const b3Vec3r& normal = vc->m_normal;

        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];

        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];

        //if (true) {
        if (vc->m_point_count == 1) {

            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;

                b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
                real vn = v_rel.dot(normal);

                real lambda = 0;

                lambda = -vcp->m_normal_mass * (vn - vcp->m_bias_velocity);
                real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                lambda = new_impulse - vcp->m_normal_impulse;
                vcp->m_normal_impulse = new_impulse;

                // apply normal Impulse
                b3Vec3r impulse = lambda * normal;

                v_a = v_a - vc->m_inv_mass_a * impulse;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
                v_b = v_b + vc->m_inv_mass_b * impulse;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
            }

        } else {
            // this is a Linear Complementary Problem with max size of 16
            // vn = A * x + b, vn >= 0, xn >= 0, vni * xni = 0
            // A = J * M_inv * JT
            // b = vn0 - v_bias
            // because the system is iterative, use incremental impulse instead
            // x = a + d
            // x: new total impulse
            // d: incremental impulse
            // vn = A * d + b
            //    = A * (x - a) + b
            //    = A * x + b - A * a
            // vn = A * x + b'
            // b' = b - A * a
            const int32& size = vc->m_point_count;

            real* b = (real*)m_block_allocator->allocate(size * sizeof(real));
            int32* pivot = (int32*)m_block_allocator->allocate(size * sizeof(int32));

            real* mem_tableau = (real*)m_block_allocator->allocate(size * (2 * size + 1) * sizeof(real));
            real** tableau = (real**)m_block_allocator->allocate(size * sizeof(real**));
            real* result = (real*)m_block_allocator->allocate(2 * size * sizeof(real));

            real* mem_I = (real*)m_block_allocator->allocate(size * size * sizeof(real));
            real** I_inv = (real**)m_block_allocator->allocate(size * sizeof(real**));

            real* a = (real*)m_block_allocator->allocate(size * sizeof(real));

            for (int32 i = 0; i < size; ++i) {
                I_inv[i] = &mem_I[i * size];
            }
            memset(result, 0, 2 * size * sizeof(real));
            memset(mem_I, 0, size * size * sizeof(real));
            memset(mem_tableau, 0, size * (2 * size + 1) * sizeof(real));
            for (int32 i = 0; i < size; ++i) {
                tableau[i] = &mem_tableau[i * (2 * size + 1)];
            }


            real** JWJT = vc->m_JWJT;

            bool all_positive = true;

            for (int32 i = 0; i < size; i++) {
                a[i] = vc->m_points[i].m_normal_impulse;
            }

            print_array(a, size, "a vector");

            for (int32 i = 0; i < size; i++) {
                // b[i] = vc->m_points[i].m_vn - vc->m_points[i].m_bias_velocity;
                // TODO: m_vn should be updated;
                b3VelocityConstraintPoint* vcp = vc->m_points + i;
                b3Vec3 dv = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);

                b[i] = dv.dot(normal);

                for (int32 j = 0; j < size; j++)
                    b[i] -= JWJT[i][j] * a[j];
                all_positive = all_positive && b[i] >= 0;

                tableau[i][i] = 1;
                tableau[i][2 * size] = -1;   ///< -e0 column
                for (int32 j = 0; j < size; j++) {
                    tableau[i][size + j] = JWJT[i][j];
                }
            }

            print_array(b, size, "b vecto  r");

            if (all_positive) {

                m_block_allocator->free(b, size * sizeof(real));
                m_block_allocator->free(pivot, 2 * size * sizeof(int32) + 1);
                m_block_allocator->free(mem_tableau, size * (2 * size + 1) * sizeof(real));
                m_block_allocator->free(tableau, size * sizeof(real**));
                m_block_allocator->free(mem_I, size * size * sizeof(real));
                m_block_allocator->free(I_inv, size * sizeof(real**));

                continue;
            }

            // initialize pivot vector
            // v1 ... vn x1 ... xn z0
            memset(pivot, 0, size * sizeof(int32));
            for (int32 i = 0; i < size; i++) {
                pivot[i] = i;
            }

            DEBUG_PRINT(-1, tableau, I_inv, b, size, pivot);

            ///////////////////////// Initialize /////////////////////////
            int32 iteration = 0;
            auto min_it = std::min_element(b, b + size);
            int32 min_index = std::distance(b, min_it);

            b3_assert(0 <= min_index && min_index < size);


            eliminate(tableau, I_inv, b, min_index, 2 * size, size);

            reset_I(I_inv, size); ///< set first iteration of I_inv to Identity
            int32 drop_out = min_index; ///< the first one to drop out is in v vector
            pivot[min_index] = 2 * size; ///< In this position is z0, the index of z0 is 2 * size
            print_array(pivot, size, "pivot");

            DEBUG_PRINT(iteration++, tableau, I_inv, b, size, pivot);

            for (;;) {

                // if z0 is drop out, the algorithm stopped
                if (drop_out == 2 * size)
                    break;

                int32 bring_in = drop_out % size + size * (drop_out < size);
                spdlog::info("bring-in index: {}\n", bring_in);
                spdlog::info("drop-out index: {}\n", drop_out);
                LexicoInv::size = size;

                LexicoInv lexico_min{b[0], I_inv[0], tableau[0][bring_in]};

                int32 lexico_min_index = 0;
                for (int32 i = 1; i < size; i++) {
                    LexicoInv lexico_cur{b[i], I_inv[i], tableau[i][bring_in]};
                    if (lexico_cur < lexico_min) {
                        lexico_min = lexico_cur;
                        lexico_min_index = i;
                    }
                }


                reset_I(I_inv, size); ///< set first iteration of I_inv to Identity

                // swap the basic variables
                drop_out = pivot[lexico_min_index];
                pivot[lexico_min_index] = bring_in;

                eliminate(tableau, I_inv, b, lexico_min_index,  bring_in, size);

                DEBUG_PRINT(iteration++, tableau, I_inv, b, size, pivot);

            }
            DEBUG_PRINT(iteration++, tableau, I_inv, b, size, pivot);
            int32 gogogo = 1;

            for (int i = 0; i < size; i++) {
                result[pivot[i]] = b[i];
            }

            m_block_allocator->free(b, size * sizeof(real));
            m_block_allocator->free(pivot, 2 * size * sizeof(int32) + 1);
            m_block_allocator->free(mem_tableau, size * (2 * size + 1) * sizeof(real));
            m_block_allocator->free(tableau, size * sizeof(real**));
            m_block_allocator->free(mem_I, size * size * sizeof(real));
            m_block_allocator->free(I_inv, size * sizeof(real**));

            print_array(result, 2 * vc->m_point_count, "res");
            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;
                b3Vec3r impulse = result[vc->m_point_count + j] * vc->m_normal;
                v_a = v_a - vc->m_inv_mass_a * impulse;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
                v_b = v_b + vc->m_inv_mass_b * impulse;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);

                vcp->m_normal_impulse = result[vc->m_point_count + j];
            }


            m_block_allocator->free(result, 2 * vc->m_point_count * sizeof(real));
        }

        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;

    }
}

// the tableau is just like this:
// v1 ...... vn x1 ...... xn   z0
//      I            -JWJT    -e0  b
static void eliminate(real** tableau, real** I_inv, real* b, int32 i, int32 j, int32 size) {
    b3_assert(0 <= i && i < size);
    b3_assert(0 <= j && j < 2 * size + 1);
    real pivot_value = tableau[i][j];

    real ratio = real(1.0) / pivot_value;

    // normalize the pivot value to 1
    // and normalize the pivot row
    b[i] *= ratio;
    for (int32 k = 0; k < 2 * size + 1; ++k) {
        tableau[i][k] *= ratio;
        if (k < size)
            I_inv[i][k] *= ratio;
    }
    print_2d_array(tableau, size, 2 * size + 1, "tableau with pivot normalized");

    for (int32 k = 0; k < size; k++) {
        // skip this row
        if (k == i) continue;

        real factor = tableau[k][j];
        b[k] -= factor * b[i];
        for (int32 l = 0; l < 2 * size + 1; ++l) {
            tableau[k][l] -= factor * tableau[i][l];
            if (l < size) {
                I_inv[k][l] -= factor * I_inv[i][l];
            }

        }
    }
    print_2d_array(tableau, size, 2 * size + 1, "tableau after elimination");
    return;
}

