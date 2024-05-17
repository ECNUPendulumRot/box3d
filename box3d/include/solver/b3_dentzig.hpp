
#ifndef B3_DENTZIG_HPP
#define B3_DENTZIG_HPP

#include "math/b3_vec3.hpp"

struct b3BlockAllocator;
struct b3ContactVelocityConstraint;

class b3Dentzig {

    b3BlockAllocator* m_block_allocator = nullptr;

    b3ContactVelocityConstraint* m_vc = nullptr;

    int32 m_size = 0;

    real** m_tableau = nullptr;

    real** m_I_inv = nullptr;

    int32* m_basis = nullptr;

    real* m_x = nullptr;

    int32 m_z_index = -1;

    int32 m_pivot_row_index = -1;

    int32 m_pivot_col_index = -1;

public:

    b3Dentzig(b3BlockAllocator* allocator, b3ContactVelocityConstraint* vc,
              b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b);

    bool initialize_problem();

    void solve();

    ~b3Dentzig();
};

#endif // B3_DENTZIG_HPP