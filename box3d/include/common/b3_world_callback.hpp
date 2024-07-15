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


#ifndef BOX3D_B3_WORLD_CALLBACK_HPP
#define BOX3D_B3_WORLD_CALLBACK_HPP


#include "common/b3_common.hpp"

/////////// Forward Declaration ///////////

class b3Contact;
struct b3Manifold;

//////////////////////////////////////////

/**
 * @brief The class serves as an interface for handling contact events between
 * physical entities in a physics simulation.
 */
class b3ContactListener {

public:

    /**
     * @brief Destructor for the b3ContactListener class.
     */
    virtual ~b3ContactListener() = default;

    /**
     * @brief Callback function invoked when two shapes start touching.
     * @param contact Pointer to the contact information.
     */
    virtual void begin_contact(b3Contact* contact) {b3_NOT_USED(contact);}

    /**
     * @brief Callback function invoked when two shapes stop touching.
     * @param contact Pointer to the contact information.
     */
    virtual void end_contact(b3Contact* contact) {b3_NOT_USED(contact);}

    /**
     * @brief Callback function invoked just before solving a contact.
     * @param contact Pointer to the contact information.
     * @param old_manifold Pointer to the old manifold of the contact.
     */
    virtual void pre_solve(b3Contact* contact, const b3Manifold* old_manifold) {
        b3_NOT_USED(contact);
        b3_NOT_USED(old_manifold);
    }

    /**
     * @brief Callback function invoked after solving a contact.
     * @param contact Pointer to the contact information.
     * @param manifold Pointer to the current manifold of the contact.
     */
    virtual void post_solve(b3Contact* contact, const b3Manifold* manifold) {
        b3_NOT_USED(contact);
        b3_NOT_USED(manifold);
    }
};


#endif //BOX3D_B3_WORLD_CALLBACK_HPP
