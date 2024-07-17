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

#ifndef BOX3D_B3_LEMKE_HPP
#define BOX3D_B3_LEMKE_HPP


#include "math/b3_vec12.hpp"
#include "math/b3_vec3.hpp"
#include "solver/b3_contact_constraint.hpp"

/////////// Forward Delaration ///////////

struct b3BlockAllocator;
struct b3ContactVelocityConstraint;

//////////////////////////////////////////

/**
 * @brief Implements the Lemke's algorithm for solving linear complementarity problems (LCP).
 */
class b3Lemke {

    /**
     * @brief Pointer to the block allocator used for memory management.
     */
    b3BlockAllocator* m_block_allocator = nullptr;

    /**
     * @brief Pointer to the contact velocity constraint.
     */
    b3ContactVelocityConstraint* m_vc = nullptr;

    /**
     * @brief The size of the tableau used in the Lemke's algorithm.
     */
    int32 m_size;

    /**
     * @brief Pointer to the tableau matrix used in the Lemke's algorithm.
     */
    real** m_tableau = nullptr;

    /**
     * @brief Pointer to the array representing the basis of the tableau.
     */
    int32* m_basis = nullptr;

    /**
     * @brief Pointer to the array representing the variables of the tableau.
     */
    real* m_x = nullptr;

    /**
     * @brief Index of the current z variable.
     */
    int32 m_z_index = -1;

    /**
     * @brief Index of the row of the pivot element.
     */
    int32 m_pivot_row_index = -1;

    /**
     * @brief Index of the column of the pivot element.
     */
    int32 m_pivot_col_index = -1;

public:

    /**
     * @brief Constructs a b3Lemke object with the specified allocator, velocity constraint, and vectors.
     * @param allocator Pointer to the block allocator used for memory management.
     * @param vc Pointer to the contact velocity constraint.
     * @param v_a The linear velocity vector of body A.
     * @param w_a The angular velocity vector of body A.
     * @param v_b The linear velocity vector of body B.
     * @param w_b The angular velocity vector of body B.
     */
    b3Lemke(b3BlockAllocator* allocator, b3ContactVelocityConstraint* vc,
            b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b);

    /**
     * @brief Initializes the problem for the Lemke's algorithm.
     * @return true if initialization was successful; false otherwise.
     */
    bool initialize_problem();

    /**
     * @brief Solves the Lemke's problem.
     */
    void solve();

    /**
     * @brief Retrieves the normal impulse for a given index.
     * @param index The index of the normal impulse.
     * @return The normal impulse value at the specified index.
     */
    inline real get_normal_impulse(const int32& index) const {
        return m_x[index];
    }

    /**
     * @brief Prints the values of the velocity and impulse variables.
     */
    void print_vx();

    /**
     * @brief Destructor for the b3Lemke class.
     */
    ~b3Lemke();

private:

    /**
     * @brief Performs Gaussian elimination on the tableau matrix.
     * @param i The row index to be eliminated.
     * @param j The column index to be eliminated.
     */
    void eliminate(const int32& i, const int32& j);

    /**
     * @brief Prints the values of a vector.
     * @param vec Pointer to the vector to be printed.
     * @param size The size of the vector.
     * @param s A string describing the vector.
     */
    void print_vec(const real* vec, const int32& size, const char* s);

    /**
     * @brief Prints the values of a matrix.
     * @param matrix Pointer to the matrix to be printed.
     * @param rows The number of rows in the matrix.
     * @param cols The number of columns in the matrix.
     * @param s A string describing the matrix.
     */
    void print_matrix(const real** matrix, const int32& rows, const int32& cols, const char* s);

    /**
     * @brief Finds the lexicographic minimum in the tableau matrix.
     * @param pivot_col_index The index of the pivot column.
     * @param z0Row The index of the z0 row.
     * @param is_ray_termination Reference to a boolean indicating if ray termination is detected.
     * @return The index of the row containing the lexicographic minimum.
     */
    int32 find_lexicographic_minimum(const int& pivot_col_index, const int& z0Row, bool& is_ray_termination);

    /**
     * @brief Checks if the current basis is valid.
     * @return true if the basis is valid; false otherwise.
     */
    bool valid_basis();
};


#endif //BOX3D_B3_LEMKE_HPP
