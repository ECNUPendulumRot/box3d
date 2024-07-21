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

#ifndef BOX3D_CAMERA_HPP
#define BOX3D_CAMERA_HPP

#include "box3d.hpp"

/**
 * @brief Represents a camera in the 3D space with properties and methods for view and projection.
 */
struct Camera
{
    /**
     * @brief Default constructor for Camera.
     * Initializes default values for width, height, and model matrix, and resets the view.
     */
    Camera() {
        m_width = 1280;
        m_height = 800;

        build_model_matrix(m_model_matrix);

        reset_view();
    }

    /**
     * @brief Resets the camera's view parameters to default.
     */
    void reset_view();

/**
     * @brief Builds the camera's up coordinate system.
     */
    void build_up_camera_coordinate();

    /**
     * @brief Builds the projection matrix for the camera.
     * @param m Pointer to the matrix to be filled with the projection matrix values.
     */
    void build_projection_matrix(float* m);

    /**
     * @brief Builds the view matrix for the camera.
     * @param v Pointer to the matrix to be filled with the view matrix values.
     */
    void build_view_matrix(float* v) const;

    /**
     * @brief Builds the model matrix for the camera.
     * @param m Pointer to the matrix to be filled with the model matrix values.
     */
    void build_model_matrix(float* m) const;

    // Camera width and height in pixels
    int32 m_width;
    int32 m_height;

    // Camera position and look-at point in 3D space
    b3Vec3f m_position;
    b3Vec3f m_lookat;

    // Near and far clipping planes and scale for the projection matrix
    float m_s = 0.1f;  // Scale
    float m_n = 0.1f;  // Near clipping plane
    float m_f = 100.f;  // Far clipping plane

    // Model matrix of the camera
    float m_model_matrix[16];

    // Camera's direction vectors
    b3Vec3f m_d;  // Direction vector
    b3Vec3f m_u;  // Up vector
    b3Vec3f m_r;  // Right vector
};
#endif //BOX3D_CAMERA_HPP
