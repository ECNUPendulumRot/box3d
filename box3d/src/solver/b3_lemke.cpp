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

/**
 * @brief Constructor for b3Lemke class.
 * @param allocator Memory allocator for dynamic allocations.
 * @param vc Pointer to contact velocity constraint.
 * @param v_a Linear velocity of body A.
 * @param w_a Angular velocity of body A.
 * @param v_b Linear velocity of body B.
 * @param w_b Angular velocity of body B.
 */
b3Lemke::b3Lemke(b3BlockAllocator *allocator, b3ContactVelocityConstraint *vc,
                 b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b)
{
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
            m_tableau[i][2 * m_size + 1] -= m_vc->m_JWJT[i][j] * (m_vc->m_points + j)->m_normal_impulse;
    }

    //print_matrix((const real**)m_tableau, m_size, 2 * m_size + 2, "tableau matrix");

    // set up pivot vector
    m_basis = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));
    b3_assert(m_basis != nullptr);

    for (int32 i = 0; i < m_size; i++) {
        m_basis[i] = i;
    }

    // set up result vector
    m_x = (real*)m_block_allocator->allocate(m_size * sizeof(real));
    b3_assert(m_x != nullptr);
    memset(m_x, 0, m_size * sizeof(real));
}


/**
 * @brief Initializes the problem by finding the first row with the smallest b value.
 * @return true if all b values are positive, false otherwise.
 */
bool b3Lemke::initialize_problem()
{
    // find the first row with the smallest b value
    // the z0 will be under this row
    real min_value = b3_real_max;
    bool all_positive = true;
    m_pivot_row_index = -1;
    
    // Find the row with the smallest b value
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
    // spdlog::info("pivot row index: {}", m_pivot_row_index);
    // the first pivot column is the z0 column
    m_pivot_col_index = 2 * m_size;
    m_z_index = m_pivot_row_index;

    return all_positive;
}

/**
 * @brief Solves the LCP using the Lemke algorithm.
 */
void b3Lemke::solve()
{
    spdlog::info("---------- Lemke Solver Start! ----------");
    // initialize
    int32 max_iteration = 100;

    for (int32 i = 0; i < max_iteration; i++) {

        eliminate(m_pivot_row_index, m_pivot_col_index);

//        spdlog::info("Lemke Iteration: {}", i);
//        print_matrix((const real**)m_tableau, m_size, 2 * m_size + 2, "tableau matrix");

        int32 pivot_col_index_old = m_pivot_col_index;

        // find the new column index
        if (m_basis[m_pivot_row_index] < m_size) {
            m_pivot_col_index = m_basis[m_pivot_row_index] + m_size;
        } else {
            m_pivot_col_index = m_basis[m_pivot_row_index] - m_size;
        }

        m_basis[m_pivot_row_index] = pivot_col_index_old;
        bool is_ray_termination = false;
        m_pivot_row_index = find_lexicographic_minimum(m_pivot_col_index, m_z_index, is_ray_termination);
//        spdlog::info("pivot row index: {}", m_pivot_row_index);
//        spdlog::info("pivot col index: {}", m_pivot_col_index);
//        spdlog::info("is ray termination?: {}", is_ray_termination);
        if (is_ray_termination)
            break;

        if (m_z_index == m_pivot_row_index) {
            eliminate(m_pivot_row_index, m_pivot_col_index);
            m_basis[m_pivot_row_index] = m_pivot_col_index;
            break;
        }
    }

    if (!valid_basis()) {
        spdlog::error("Lemke solver failed to find a valid basis: Ray-Termination!");
        return;
    }

    //print_matrix((const real**)m_tableau, m_size, 2 * m_size + 2, "tableau matrix");

    for (int32 i = 0; i < m_size; i++) {
        if (m_basis[i] < m_size) continue;
        m_x[m_basis[i] - m_size] = m_tableau[i][2 * m_size + 1];
    }

    spdlog::info("---------- Lemke Solver Ended... ----------");
}


// the tableau is just like this:
// v1 ...... vn x1 ...... xn   z0
//      I            -JWJT    -e0  b
/**
 * @brief Eliminates the column j using the row i as the pivot row.
 * @param i The pivot row index.
 * @param j The pivot column index.
 */
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

/**
 * @brief Prints the solution vector m_x.
 */
void b3Lemke::print_vx() {

    auto logger = spdlog::get("lemke-logger");
    logger->set_pattern("%v");
    std::ostringstream oss;
    for (int i = 0; i < m_size; i++) {
        oss << m_x[i] << " ";
    }
    spdlog::info("Lemke VX Vector: \n {}", oss.str());

}

/**
 * @brief Prints a vector.
 * @param vec The vector to print.
 * @param size The size of the vector.
 * @param s Description of the vector.
 */
void b3Lemke::print_vec(const real *vec, const int32 &size, const char* s) {

    auto logger = spdlog::get("lemke-logger");
    logger->set_pattern("%v");
    std::ostringstream oss;
    for (int i = 0; i < size; i++) {
        oss << vec[i] << " ";
    }
    spdlog::info("vector: {} \n {}", s, oss.str());
}

/**
 * @brief Prints a matrix.
 * @param matrix The matrix to print.
 * @param rows The number of rows in the matrix.
 * @param cols The number of columns in the matrix.
 * @param s Description of the matrix.
 */
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

/**
 * @brief Finds the row with the lexicographic minimum ratio.
 * @param pivot_col_index The pivot column index.
 * @param z0Row The row index of z0.
 * @param is_ray_termination Whether ray termination occurred.
 * @return The row index with the lexicographic minimum ratio.
 */
int32 b3Lemke::find_lexicographic_minimum(const int& pivot_col_index, const int& z0Row, bool& is_ray_termination)
{
    is_ray_termination = false;
    int32* active_rows = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));

    bool first_row = true;
    real current_min = 0.0;

    int32 active_rows_count = 0;

    for (int32 row = 0; row < m_size; row++) {
        const real denom = m_tableau[row][pivot_col_index];
        if (denom > b3_real_epsilon) {
            const real q = m_tableau[row][2 * m_size + 1] / denom;
            if (first_row) {
                current_min = q;
                active_rows[0] = row;
                active_rows_count = 1;
                first_row = false;
            } else if (b3_abs(current_min - q) < b3_real_epsilon) {
                active_rows[active_rows_count++] = row;
            } else if (current_min > q) {
                current_min = q;
                active_rows_count = 1;
                active_rows[0] = row;
            }
        }

    }

    if (active_rows_count == 0) {
        is_ray_termination = true;
        return 0;
    } else if (active_rows_count == 1) {
        return active_rows[0];
    }

    for (int i = 0; i < active_rows_count; i++) {
        if (active_rows[i] == z0Row) {
            return z0Row;
        }
    }
    // Lexicographic comparison to find the minimum
    for (int32 col = 0; col < m_size; col++) {
        int32* active_rows_copy = (int32*)m_block_allocator->allocate(m_size * sizeof(int32));
        for (int i = 0; i < active_rows_count; i++) {
            active_rows_copy[i] = active_rows[i];
        }

        int32 active_rows_copy_count = active_rows_count;
        active_rows_count = 0;
        first_row = true;
        for (int32 i = 0; i < active_rows_copy_count; i++) {
            const int32 row = active_rows_copy[i]; // get the active row index
            const real denom = m_tableau[row][pivot_col_index]; // get the pivot value in this row
            const real ratio = m_tableau[row][col] / denom; // get the ratio of the pivot value and the value in the column
            if (first_row) {
                current_min = ratio;
                active_rows[0] = row;
                active_rows_count = 1;
                first_row = false;
            } else if (b3_abs(current_min - ratio) < b3_real_epsilon) {
                active_rows[active_rows_count++] = row;
            } else if (current_min > ratio) {
                current_min = ratio;
                active_rows_count = 1;
                active_rows[0] = row;
            }
        }

        if (active_rows_count == 1) {
            m_block_allocator->free(active_rows_copy, m_size * sizeof(int32));
            return active_rows[0];
        }
        m_block_allocator->free(active_rows_copy, m_size * sizeof(int32));
    }

    m_block_allocator->free(active_rows, m_size * sizeof(int32));

    is_ray_termination = true;
    return 0;
}

/**
 * @brief Checks if the basis is valid.
 * @return true if the basis is valid, false otherwise.
 */
bool b3Lemke::valid_basis()
{
    bool valid = true;
    for (int32 i = 0; i < m_size; i++) {
        if (m_basis[i] >= 2 * m_size) {
            valid = false;
            break;
        }
    }
    return valid;
}


/**
 * @brief Destructor for b3Lemke class.
 */
b3Lemke::~b3Lemke()
{
    m_block_allocator->free(m_basis, m_size * sizeof(int32));
    m_block_allocator->free(m_x, m_size * sizeof(real));

    for (int32 i = 0; i < m_size; i++) {
        m_block_allocator->free(m_tableau[i], (2 * m_size + 2) * sizeof(real));
    }
    m_block_allocator->free(m_tableau, m_size * sizeof(real*));
}


