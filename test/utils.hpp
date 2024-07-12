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
// ----------------------------------------------------------------------------
// sherman          2024-4-25           created

#ifndef BOX3D_UTILS_HPP
#define BOX3D_UTILS_HPP


#include "box3d.hpp"
#include <nlohmann/json.hpp>
#include <fstream>


class Utils {

    b3Fixture* m_fixtures[1024];

    b3Body* m_bodies[1024];
    std::string m_body_names[1024];
    int32 m_fixture_count = 0;
    int32 m_body_count = 0;

    nlohmann::json m_meta_json;
    nlohmann::json m_frame_json;

    bool file_is_open = false;
    bool json_saved = false;
    std::ofstream csv_file;

public:

    void open_file();

    void save_body_info();

    inline void close_file() {
        if (file_is_open) {
            csv_file.close();
        }
    }

    void track_fixture(b3Fixture *fixture, const char *name);

    void track_body(b3Body* body, const char* name);

    void record_frame();

    void save_json_file();

    ~Utils() {
        close_file();
        file_is_open = false;
    }

    b3Body** get_bodies() {
        return m_bodies;
    }

};


#endif //BOX3D_UTILS_HPP
