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

// history
//
// author           date                description
// ---------------------------------------------------------------------------- -
// sherman          2024-4-25           created

#ifndef BOX3D_SETTINGS_HPP
#define BOX3D_SETTINGS_HPP

struct Settings {

    Settings() {
        reset();
    }

    void reset() {
        m_window_width = 1600;
        m_window_height = 900;
        m_hertz = 60.0f;
        m_test_index = 0;
        m_single_step = false;
    }

    void save() const;
    void load();

    int m_window_width;
    int m_window_height;

    int m_test_index = 0;
    int m_velocity_iteration = 8;
    int m_position_iteration = 8;
    bool m_draw_shapes = true;
    bool m_draw_frame_only = false;
    bool m_draw_contact_points = false;
    bool m_pause = false;
    bool m_single_step = false;
    bool m_generate_json = false;
    bool m_output_bodies_info = false;
    bool m_enable_sleep = true;
    bool m_enable_continuous_physics = false;
    float m_hertz;
};

#endif //BOX3D_SETTINGS_HPP
