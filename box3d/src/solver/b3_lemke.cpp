
#include "solver/b3_lemke.hpp"

#include <algorithm>

#include "solver/b3_contact_constraint.hpp"
#include "common/b3_block_allocator.hpp"
#include "math/b3_mat1212.hpp"


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


bool b3Lemke::initialize_problem(b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b)
{
    const b3Vec3r& normal = m_vc->m_normal;

    // set up tableau pointer
    real* mem_tableau = (real*)m_block_allocator->allocate(m_size * (2 * m_size + 1) * sizeof(real));
    memset(mem_tableau, 0, m_size * (2 * m_size + 1) * sizeof(real));
    m_tableau = (real**)m_block_allocator->allocate(m_size * sizeof(real**));
    for (int32 i = 0; i < m_size; ++i) {
        m_tableau[i] = &mem_tableau[i * (2 * m_size + 1)];
    }

    // set up inverse identity pointer
    real* mem_I = (real*)m_block_allocator->allocate(m_size * m_size * sizeof(real));
    memset(mem_I, 0, m_size * m_size * sizeof(real));
    m_I_inv = (real**)m_block_allocator->allocate(m_size * sizeof(real**));
    for (int32 i = 0; i < m_size; ++i) {
        m_I_inv[i] = &mem_I[i * m_size];
    }

    // set up b vector
    m_b = (real*)m_block_allocator->allocate(m_size * sizeof(real));

    // set up pivot vector
    m_pivot = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));
    for (int32 i = 0; i < m_size; i++) {
        m_pivot[i] = i;
    }

    m_vx = (real*)m_block_allocator->allocate(2 * m_size * sizeof(real));
    memset(m_vx, 0, 2 * m_size * sizeof(real));

    // initialize b vector, and early quit
    real* a = (real*)m_block_allocator->allocate(m_size * sizeof(real));
    for (int32 i = 0; i < m_size; i++) {
        a[i] = m_vc->m_points[i].m_normal_impulse;
    }

    bool all_positive = true;

    for (int32 i = 0; i < m_size; i++) {
        b3VelocityConstraintPoint* vcp = m_vc->m_points + i;
        b3Vec3 dv = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
        m_b[i] = dv.dot(normal) - vcp->m_bias_velocity * (m_flags & e_lemke_restitution);
        for (int32 j = 0; j < m_size; j++)
            m_b[i] -= m_vc->m_JWJT[i][j] * a[j];
        all_positive = all_positive && m_b[i] >= 0;
    }

    if (all_positive)
        return true;

    // Initialize the tableau
    for (int32 i = 0; i < m_size; i++) {
        m_tableau[i][i] = 1;
        m_tableau[i][2 * m_size] = -1;   ///< -e0 column
        for (int32 j = 0; j < m_size; j++) {
            m_tableau[i][m_size + j] = m_vc->m_JWJT[i][j];
        }
    }
    return false;
}


void b3Lemke::solve()
{
    // initialize
    int32 iteration = 0;
    auto min_it = std::min_element(m_b, m_b + m_size);
    int32 min_index = std::distance(m_b, min_it);

    b3_assert(0 <= min_index && min_index < m_size);

    eliminate(min_index, 2 * m_size);

    reset_identity();                ///< set first iteration of I_inv to Identity
    int32 drop_out = min_index;      ///< the first one to drop out is in v vector
    m_pivot[min_index] = 2 * m_size; ///< In this position is z0, the index of z0 is 2 * size

    for (;;) {

        // if z0 is drop out, the algorithm stopped
        if (drop_out == 2 * m_size)
            break;

        int32 bring_in = drop_out % m_size + m_size * (drop_out < m_size);
        b3LexicoInv::size = m_size;

        b3LexicoInv lexico_min{m_b[0], m_I_inv[0], m_tableau[0][bring_in]};

        int32 lexico_min_index = 0;
        for (int32 i = 1; i < m_size; i++) {
            b3LexicoInv lexico_cur{m_b[i], m_I_inv[i], m_tableau[i][bring_in]};
            if (lexico_cur < lexico_min) {
                lexico_min = lexico_cur;
                lexico_min_index = i;
            }
        }

        reset_identity();

        // swap the basic variables
        drop_out = m_pivot[lexico_min_index];
        m_pivot[lexico_min_index] = bring_in;

        eliminate(lexico_min_index,  bring_in);

        iteration++;
    }

    // copy the results
    for (int i = 0; i < m_size; i++)
        m_vx[m_pivot[i]] = m_b[i];
}


b3Lemke::~b3Lemke()
{
    m_block_allocator->free(m_b, m_size * sizeof(real));
    m_block_allocator->free(m_pivot, 2 * m_size * sizeof(int32) + 1);
    m_block_allocator->free(&m_tableau[0], m_size * (2 * m_size + 1) * sizeof(real));
    m_block_allocator->free(m_tableau, m_size * sizeof(real**));
    m_block_allocator->free(&m_I_inv[0], m_size * m_size * sizeof(real));
    m_block_allocator->free(m_I_inv, m_size * sizeof(real**));
    m_block_allocator->free(m_vx, 2 * m_size * sizeof(real));
}


// the tableau is just like this:
// v1 ...... vn x1 ...... xn   z0
//      I            -JWJT    -e0  b
void b3Lemke::eliminate(const int32 &i, const int32 &j)
{
    b3_assert(0 <= i && i < m_size);
    b3_assert(0 <= j && j < 2 * m_size + 1);
    real pivot_value = m_tableau[i][j];

    real ratio = real(1.0) / pivot_value;

    // normalize the pivot value to 1
    // and normalize the pivot row
    m_b[i] *= ratio;
    for (int32 k = 0; k < 2 * m_size + 1; ++k) {
        m_tableau[i][k] *= ratio;
        if (k < m_size)
            m_I_inv[i][k] *= ratio;
    }

    for (int32 k = 0; k < m_size; k++) {
        // skip this row
        if (k == i) continue;

        real factor = m_tableau[k][j];
        m_b[k] -= factor * m_b[i];
        for (int32 l = 0; l < 2 * m_size + 1; ++l) {
            m_tableau[k][l] -= factor * m_tableau[i][l];
            if (l < m_size) {
                m_I_inv[k][l] -= factor * m_I_inv[i][l];
            }

        }
    }
}


void b3Lemke::reset_identity() {
    real* start = m_I_inv[0];
    memset(start, 0, m_size * m_size * sizeof(real));
    for (int32 i = 0; i < m_size; ++i) {
        m_I_inv[i][i] = 1;
    }
}

