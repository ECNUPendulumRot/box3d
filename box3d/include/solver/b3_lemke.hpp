
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

    int32* m_pivot = nullptr;

    real* m_b = nullptr;

    real* m_vx = nullptr;

    uint32 m_flags = 0;

public:

    enum {
        e_lemke_null_option = 0x0000,
        e_lemke_restitution = 0x0001
    };

    b3Lemke(b3BlockAllocator* allocator, b3ContactVelocityConstraint* vc, uint32 flags) {
        m_block_allocator = allocator;
        m_vc = vc;
        m_size = m_vc->m_point_count;
        m_flags = flags;
    }

    bool initialize_problem(b3Vec3r& v_a, b3Vec3r& w_a, b3Vec3r& v_b, b3Vec3r& w_b);

    void solve();

    inline real get_normal_impulse(const int32& index) const {
        return m_vx[m_size + index];
    }

    ~b3Lemke();

private:

    void eliminate(const int32& i, const int32& j);

    void reset_identity();
};


#endif //BOX3D_B3_LEMKE_HPP
