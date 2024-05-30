
#ifndef BOX3D_B3_DRAW_HPP
#define BOX3D_B3_DRAW_HPP

#include "math/b3_vec3.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Delaration ///////////

struct b3EdgeIndex;
struct b3FaceIndex;
struct b3CubeShape;
struct b3PlaneShape;
struct b3SphereShape;

//////////////////////////////////////////


struct b3Color {

    b3Color() = default;

    b3Color(float rIn, float gIn, float bIn, float aIn = 1.0f)
    {
        r = rIn; g = gIn; b = bIn; a = aIn;
    }

    void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
    {
        r = rIn; g = gIn; b = bIn; a = aIn;
    }

    static const b3Color black;

    float r, g, b, a;
};



class b3Draw {

public:

    b3Draw() = default;

    virtual ~b3Draw() = default;

    enum {
        e_shape_bit		     = 0x0001,	///< draw shape faces
        e_frame_only_bit     = 0x0002,	///< draw frame instead of faces
        e_aabb_bit		     = 0x0004,	///< draw axis aligned bounding boxes
        e_pair_bit		     = 0x0008,	///< draw broad-phase pairs
        e_center_of_mass_bit = 0x0010	///< draw center of mass frame
    };

    inline void set_flags(uint32 flags) {
        m_draw_flags = flags;
    }

    uint32 get_flags() const {
        return m_draw_flags;
    }

    void append_flags(uint32 flags) {
        m_draw_flags |= flags;
    }

    void clear_flags(uint32 flags) {
        m_draw_flags &= ~flags;
    }

    virtual void draw_box(const b3CubeShape* cube, const b3Transformr& xf, const b3Color& color) = 0;

    virtual void draw_plane(const b3PlaneShape* plane, const b3Transformr& xf, const b3Color& color) = 0;

    virtual void draw_sphere(const b3SphereShape* sphere, const b3Transformr& xf, const b3Color& color) = 0;

    virtual void draw_point(const b3Vec3r& p, float size, const b3Color& color) = 0;

protected:

    uint32 m_draw_flags = 0;

};

#endif //BOX3D_B3_DRAW_HPP
