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


#ifndef BOX3D_B3_DRAW_HPP
#define BOX3D_B3_DRAW_HPP

#include "math/b3_vec3.hpp"
#include "dynamics/b3_transform.hpp"


/////////// Forward Declaration ///////////

struct b3EdgeIndex;
struct b3FaceIndex;
struct b3CubeShape;
struct b3PlaneShape;
struct b3SphereShape;

//////////////////////////////////////////

/**
 * @brief The b3Color structure is designed to represent color using the RGBA
 * (Red, Green, Blue, Alpha) model. It provides functionality to initialize and
 * set color values.
 */
struct b3Color {

    /**
     * @brief This is the default constructor
     */
    b3Color() = default;

    /**
     * @brief This is a parameterized constructor that initializes a b3Color object
     * with specified values for the red, green, blue, and alpha components.
     * @param rIn The red component of the color.
     * @param gIn The green component of the color.
     * @param bIn The blue component of the color.
     * @param aIn The alpha component of the color, with a default value of 1.0f.
     */
    b3Color(float rIn, float gIn, float bIn, float aIn = 1.0f)
    {
        r = rIn; g = gIn; b = bIn; a = aIn;
    }

    /**
     * @brief This function sets the color's components to the specified values.
     * It provides a way to modify the color of an existing b3Color object.
     * @param rIn The red component of the color.
     * @param gIn The green component of the color.
     * @param bIn The blue component of the color.
     * @param aIn The alpha component of the color, with a default value of 1.0f.
     */
    void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
    {
        r = rIn; g = gIn; b = bIn; a = aIn;
    }

    /**
     * @brief This is a static constant representing the color black.
     */
    static const b3Color black;

    /**
     * @brief float r: The red component of the color.
     * float g: The green component of the color.
     * float b: The blue component of the color.
     * float a: The alpha component of the color.
     */
    float r, g, b, a;
};


/**
 * @brief The b3Draw class serves as an abstract base class for drawing various shapes
 * and visual elements in a physics simulation or rendering context. It provides
 * methods to manage drawing flags and defines pure virtual functions for drawing
 * specific shapes.
 */
class b3Draw {

public:

    /**
     * @brief Default constructor for b3Draw class.
     */
    b3Draw() = default;

    /**
     * @brief Virtual destructor for b3Draw class.
     */
    virtual ~b3Draw() = default;

    /**
     * @brief This enum defines symbolic constants representing different drawing
     * options or flags used in a graphics or physics simulation context.
     */
    enum {
        e_shape_bit		     = 0x0001,	///< draw shape faces
        e_frame_only_bit     = 0x0002,	///< draw frame instead of faces
        e_aabb_bit		     = 0x0004,	///< draw axis aligned bounding boxes
        e_pair_bit		     = 0x0008,	///< draw broad-phase pairs
        e_center_of_mass_bit = 0x0010	///< draw center of mass frame
    };

    /**
     * @brief Sets the drawing flags to the specified value.
     * @param flags The flags to set.
     */
    inline void set_flags(uint32 flags) {
        m_draw_flags = flags;
    }

    /**
     * @brief get the current drawing flags.
     * @return Returns the current drawing flags.
     */
    uint32 get_flags() const {
        return m_draw_flags;
    }

    /**
     * @brief Appends the specified flags to the current drawing flags.
     * @param flags The flags to append.
     */
    void append_flags(uint32 flags) {
        m_draw_flags |= flags;
    }

    /**
     * @brief Clears the specified flags from the current drawing flags.
     * @param flags The flags to clear.
     */
    void clear_flags(uint32 flags) {
        m_draw_flags &= ~flags;
    }

    /**
     * @brief Pure virtual function intended to be overridden by derived classes to
     * draw a cube shape.
     * @param cube Pointer to the cube shape to be drawn.
     * @param xf Transformation of the cube in the world.
     * @param color Color with which to draw the cube.
     */
    virtual void draw_box(const b3CubeShape* cube, const b3Transr& xf, const b3Color& color) = 0;

    /**
     * @brief Pure virtual function intended to be overridden by derived classes to draw a plane shape.
     * @param plane Pointer to the plane shape to be drawn.
     * @param xf Transformation of the plane in the world.
     * @param color Color with which to draw the plane.
     */
    virtual void draw_plane(const b3PlaneShape* plane, const b3Transr& xf, const b3Color& color) = 0;

    /**
     * @brief Pure virtual function intended to be overridden by derived classes to draw a sphere shape.
     * @param sphere Pointer to the sphere shape to be drawn.
     * @param xf Transformation of the sphere in the world.
     * @param color Color with which to draw the sphere.
     */
    virtual void draw_sphere(const b3SphereShape* sphere, const b3Transr& xf, const b3Color& color) = 0;

    /**
     * @brief Pure virtual function intended to be overridden by derived classes to draw a point.
     * @param p Position of the point to be drawn.
     * @param size Size or scale of the point.
     * @param color Color with which to draw the point.
     */
    virtual void draw_point(const b3Vec3r& p, float size, const b3Color& color) = 0;

    virtual void draw_line(const b3Vec3r& p1, const b3Vec3r& p2, const b3Color& color) = 0;
protected:

    /**
     * @brief This variable holds the current drawing flags that control which elements should be drawn. T
     */
    uint32 m_draw_flags = 0;

};

#endif //BOX3D_B3_DRAW_HPP
