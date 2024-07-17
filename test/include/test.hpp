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

//
// Created by sherman on 24-1-24.
//

#ifndef BOX3D_TEST_HPP
#define BOX3D_TEST_HPP

#include <spdlog/spdlog.h>

#include "box3d.hpp"
#include "draw.hpp"

#include <fstream>
#include "utils.hpp"

struct Settings;
class Test;
class Utils;

/**
 * @brief Represents a contact point in the simulation.
 */
struct ContactPoint
{
   b3Fixture* fixtureA;    /**< The first fixture involved in the contact. */
    b3Fixture* fixtureB;    /**< The second fixture involved in the contact. */
    b3Vec3r normal;         /**< The contact normal. */
    b3Vec3r position;       /**< The contact position. */
    float normalImpulse;    /**< The normal impulse. */
    float tangentImpulse;   /**< The tangent impulse. */
    float separation;       /**< The separation distance. */
};


const int32 k_max_contact_points = 2048;/**< Maximum number of contact points. */

/**
 * @brief Base class for all tests.
 */
class Test: public b3ContactListener {

protected:


    b3World* m_world; /**< The physics world. */
    ContactPoint m_points[k_max_contact_points]; /**< Array of contact points. */
    int32 m_point_count = 0; /**< Number of contact points. */
    Utils utils; /**< Utility functions. */
    bool print_once = true; /**< Flag to print once. */
    int count = 0; /**< Counter variable. */

public:

    /**
     * @brief Constructs a Test object.
     */
    Test();

    /**
     * @brief Destroys the Test object.
     */
    virtual ~Test() {
        m_world->clear();
        utils.save_json_file();
        delete m_world;
    }

    /**
     * @brief Steps the simulation.
     * 
     * @param settings The settings for the simulation.
     */
    virtual void step(Settings& settings);

    /**
     * @brief Pre-solve event handler.
     * 
     * @param contact The contact.
     * @param old_manifold The old manifold.
     */
    virtual void pre_solve(b3Contact* contact, const b3Manifold* old_manifold);

    /**
     * @brief Prints the first non-symmetric contact.
     */
    void print_first_not_symmetry();
};


using TestCreateFcn = Test*();

/**
 * @brief Represents an entry for a test.
 */
struct TestEntry
{
    const char* category; /**< The category of the test. */

    const char* name; /**< The name of the test. */

    TestCreateFcn* create_fcn; /**< The function to create the test. */
};

/**
 * @brief Registers a test.
 * 
 * @param category The category of the test.
 * @param name The name of the test.
 * @param fcn The function to create the test.
 * @return The index of the test.
 */
int register_test(const char* category, const char* name, TestCreateFcn* fcn);

#define MAX_TEST 256

extern TestEntry g_test_entries[MAX_TEST];
extern int g_test_count;


#endif //BOX3D_TEST_HPP
