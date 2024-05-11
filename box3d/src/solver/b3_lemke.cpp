
#include "solver/b3_lemke.hpp"

#include <algorithm>

#include "solver/b3_contact_constraint.hpp"
#include "common/b3_block_allocator.hpp"
#include "math/b3_mat1212.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iomanip>
#include <sstream>

static auto logger = spdlog::stdout_color_mt("lemke-logger");


struct b3LexicoInv {

    real b_value = 0.0;
    real* inv_row = nullptr;
    real piv_value = 0.0;
    static int32 size;

    bool operator<(const b3LexicoInv& other) const {
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


int32 b3LexicoInv::size = 0;

b3Lemke::b3Lemke(b3BlockAllocator *allocator, b3ContactVelocityConstraint *vc,
                 b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b) {
    m_block_allocator = allocator;
    m_vc = vc;
    m_size = m_vc->m_point_count;

    const b3Vec3r& normal = m_vc->m_normal;

    // set up tableau
    m_tableau = (real**)m_block_allocator->allocate(m_size * sizeof(real*));
    b3_assert(m_tableau != nullptr);
    for (int32 i = 0; i < m_size; ++i) {
        m_tableau[i] = (real*)m_block_allocator->allocate((2 * m_size + 2) * sizeof(real));
        memset(m_tableau[i], 0, (2 * m_size + 2) * sizeof(real));

        m_tableau[i][i] = 1.0; // set the identity matrix
        // set the -M matrix
        for (int32 j = 0; j < m_size; ++j) {
            m_tableau[i][m_size + j] = -m_vc->m_JWJT[i][j];
        }
        m_tableau[i][2 * m_size] = -1.0; // set the -e0 column

        // set the b column
        b3VelocityConstraintPoint* vcp = m_vc->m_points + i;
        b3Vec3 dv = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
        m_tableau[i][2 * m_size + 1] = dv.dot(normal) - vcp->m_bias_velocity;
        for (int32 j = 0; j < m_size; j++)
            m_tableau[i][2 * m_size + 1] -= m_vc->m_JWJT[i][j] * vcp->m_normal_impulse;
    }

    print_matrix((const real**)m_tableau, m_size, 2 * m_size + 2, "tableau matrix");


    m_I_inv = (real**)m_block_allocator->allocate(m_size * sizeof(real*));
    b3_assert(m_I_inv != nullptr);
    for (int32 i = 0; i < m_size; ++i) {
        m_I_inv[i] = (real*)m_block_allocator->allocate(m_size * sizeof(real));
    }

    // set up pivot vector
    m_basis = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));
    b3_assert(m_basis != nullptr);

    for (int32 i = 0; i < m_size; i++) {
        m_basis[i] = i;
    }

    // set up result vector
    m_vx = (real*)m_block_allocator->allocate(2 * m_size * sizeof(real));
    b3_assert(m_vx != nullptr);
    memset(m_vx, 0, 2 * m_size * sizeof(real));
}



bool b3Lemke::initialize_problem()
{
    // find the first row with the smallest b value
    // the z0 will be under this row
    real min_value = b3_real_max;
    bool all_positive = true;
    m_pivot_row_index = -1;
    for (int32 i = 0; i < m_size; i++) {
        real v = m_tableau[i][2 * m_size + 1];
        if (v < min_value) {
            min_value = v;
            m_pivot_row_index = i;
        }
        if (v < 0) {
            all_positive = false;
        }
    }

    // the first pivot column is the z0 column
    m_pivot_col_index = 2 * m_size;
    m_z_index = m_pivot_row_index;

    return all_positive;
}


void b3Lemke::solve()
{
    // initialize
    int32 max_iteration = 100;

    for (int32 i = 0; i < max_iteration; i++) {
        eliminate(m_pivot_row_index, m_pivot_col_index);

        print_matrix((const real**)m_tableau, m_size, 2 * m_size + 2, "tableau matrix");

        int32 pivot_col_index_old = m_pivot_col_index;

        // find the new column index
        if (m_basis[m_pivot_row_index] < m_size) {
            m_pivot_col_index = m_basis[m_pivot_row_index] + m_size;
        } else {
            m_pivot_col_index = m_basis[m_pivot_row_index] - m_size;
        }

        m_basis[m_pivot_row_index] = pivot_col_index_old;
        bool is_ray_termination = false;


//        // if z0 is drop out, the algorithm stopped
//        if (drop_out == 2 * m_size)
//            break;
//
//        int32 bring_in = drop_out % m_size + m_size * (drop_out < m_size);
//        b3LexicoInv::size = m_size;
//
//        b3LexicoInv lexico_min{m_b[0], m_I_inv[0], m_tableau[0][bring_in]};
//
//        int32 lexico_min_index = 0;
//        for (int32 i = 1; i < m_size; i++) {
//            b3LexicoInv lexico_cur{m_b[i], m_I_inv[i], m_tableau[i][bring_in]};
//            if (lexico_cur < lexico_min) {
//                lexico_min = lexico_cur;
//                lexico_min_index = i;
//            }
//        }
//
//        reset_identity();
//
//        // swap the basic variables
//        drop_out = m_basis[lexico_min_index];
//        m_basis[lexico_min_index] = bring_in;
//
//        eliminate(lexico_min_index,  bring_in);
//
//        iteration++;
    }

//    // copy the results
//    for (int i = 0; i < m_size; i++)
//        m_vx[m_basis[i]] = m_b[i];
}


// the tableau is just like this:
// v1 ...... vn x1 ...... xn   z0
//      I            -JWJT    -e0  b
void b3Lemke::eliminate(const int32 &i, const int32 &j)
{
    b3_assert(0 <= i && i < m_size);
    b3_assert(0 <= j && j < 2 * m_size + 2);

    real pivot_value = m_tableau[i][j];

    real ratio = real(1.0) / pivot_value;

    // normalize the pivot row
    for (int32 k = 0; k < 2 * m_size + 2; ++k) {
        m_tableau[i][k] *= ratio;
    }

    for (int32 k = 0; k < m_size; k++) {
        // skip this row
        if (k == i) continue;

        real factor = m_tableau[k][j];
        for (int32 l = 0; l < 2 * m_size + 2; ++l) {
            m_tableau[k][l] -= factor * m_tableau[i][l];
        }
    }
}


void b3Lemke::reset_identity() {
    memset(m_I_inv[0], 0, m_size * m_size * sizeof(real));
    for (int32 i = 0; i < m_size; ++i) {
        m_I_inv[i][i] = 1;
    }
}

void b3Lemke::print_vx() {

    auto logger = spdlog::get("lemke-logger");
    logger->set_pattern("%v");
    std::ostringstream oss;
    for (int i = 0; i < 2 * m_size; i++) {
        oss << m_vx[i] << " ";
    }
    spdlog::info("Lemke VX Vector: \n {}", oss.str());

}


b3Lemke::~b3Lemke()
{
    m_block_allocator->free(m_basis, m_size * sizeof(int32));
    m_block_allocator->free(m_vx, 2 * m_size * sizeof(real));

    m_block_allocator->free(m_tableau[0], m_size * (2 * m_size + 2) * sizeof(real));
    m_block_allocator->free(m_tableau, m_size * sizeof(real*));
    m_block_allocator->free(m_I_inv[0], m_size * m_size * sizeof(real));
    m_block_allocator->free(m_I_inv, m_size * sizeof(real*));
}


void b3Lemke::print_vec(const real *vec, const int32 &size, const char* s) {

    auto logger = spdlog::get("lemke-logger");
    logger->set_pattern("%v");
    std::ostringstream oss;
    for (int i = 0; i < size; i++) {
        oss << vec[i] << " ";
    }
    spdlog::info("vector: {} \n {}", s, oss.str());
}

void b3Lemke::print_matrix(const real **matrix, const int32 &rows, const int32 &cols, const char *s) {

        auto logger = spdlog::get("lemke-logger");
        logger->set_pattern("%v");
        std::ostringstream oss;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                oss << std::fixed << std::setfill(' ') <<  std::setw(9) << matrix[i][j] << " ";
            }
            oss << "\n";
        }
        spdlog::info("matrix: {} \n{}", s, oss.str());

}

int32 b3Lemke::find_lexicographic_minimum(const int& pivot_col_index, const int& z0Row, bool& is_ray_termination)
{
    is_ray_termination = false;
    int32* active_rows = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));

    bool first_row = true;
    real current_min = 0.0;

    for (int32 row = 0; row < m_size; row++) {
        const real denom = m_tableau[row][pivot_col_index];
    }
}


