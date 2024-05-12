
#ifndef BOX3D_B3_LEMKE_HPP
#define BOX3D_B3_LEMKE_HPP


#include "math/b3_vec12.hpp"
#include "math/b3_vec3.hpp"
#include "solver/b3_contact_constraint.hpp"

/////////// Forward Delaration ///////////

struct b3BlockAllocator;
struct b3ContactVelocityConstraint;

//////////////////////////////////////////


class b3Lemke {

    b3BlockAllocator* m_block_allocator = nullptr;

    b3ContactVelocityConstraint* m_vc = nullptr;

    int32 m_size;

    real** m_tableau = nullptr;

    real** m_I_inv = nullptr;

    int32* m_basis = nullptr;

    real* m_vx = nullptr;

    int32 m_z_index = -1;

    int32 m_pivot_row_index = -1;

    int32 m_pivot_col_index = -1;

public:

    b3Lemke(b3BlockAllocator* allocator, b3ContactVelocityConstraint* vc,
            b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b);

    bool initialize_problem();

    void solve();

    inline real get_normal_impulse(const int32& index) const {
        return m_vx[m_size + index];
    }

    void print_vx();

    ~b3Lemke();

private:

    void eliminate(const int32& i, const int32& j);

    void print_vec(const real* vec, const int32& size, const char* s);

    void print_matrix(const real** matrix, const int32& rows, const int32& cols, const char* s);

    int32 find_lexicographic_minimum(const int& pivot_col_index, const int& z0Row, bool& is_ray_termination);

    bool valid_basis();

};


#endif //BOX3D_B3_LEMKE_HPP
