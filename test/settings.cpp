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

#include "settings.hpp"
#include "sajson/sajson.h"
#include <stdio.h>

static const char* file_name = "settings.ini";


static bool s_read_file(char*& data, int& size, const char* filename)
{
    FILE *file;
    errno_t err;
    err = fopen_s(&file, filename, "rb");

    if (err != 0) {
        return false;
    }

    fseek(file, 0, SEEK_END);
    size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (size == 0) {
        return false;
    }

    data = (char*)malloc(size + 1);
    fread(data, size, 1, file);
    fclose(file);
    data[size] = 0;

    return true;
}


void Settings::save() const
{
    FILE *file;
    errno_t err;
    err = fopen_s(&file, file_name, "w");
    if (err != 0) {
        return;
    }
    fprintf(file, "{\n");
    fprintf(file, "  \"test_index\": %d,\n", m_test_index);
    fprintf(file, "  \"window_width\": %d,\n", m_window_width);
    fprintf(file, "  \"window_height\": %d,\n", m_window_height);
    fprintf(file, "  \"hertz\": %.9g,\n", m_hertz);
    fprintf(file, "  \"velocity_iteration\": %d,\n", m_velocity_iteration);
    fprintf(file, "  \"position_iteration\": %d,\n", m_position_iteration);
    fprintf(file, "  \"enable_sleep\": %s,\n", m_enable_sleep ? "true" : "false");
    fprintf(file, "  \"enable_continuous_physics\": %s,\n", m_enable_continuous_physics ? "true" : "false");
    fprintf(file, "  \"draw_shapes\": %s,\n", m_draw_shapes ? "true" : "false");
    fprintf(file, "  \"draw_frame_only\": %s,\n", m_draw_frame_only ? "true" : "false");
    fprintf(file, "  \"draw_contact_points\": %s\n", m_draw_contact_points ? "true" : "false");
    fprintf(file, "}\n");
    fclose(file);
}


void Settings::load() {
    char* data = nullptr;
    int size = 0;
    bool found = s_read_file(data, size, file_name);

    if (!found) {
        return;
    }
    const sajson::document& document = sajson::parse(sajson::dynamic_allocation(), sajson::mutable_string_view(size, data));
    if (!document.is_valid()) {
        return;
    }

    sajson::value root = document.get_root();
    int field_count = int(root.get_length());
    for (int i = 0; i < field_count; ++i)
    {
        sajson::string field_name = root.get_object_key(i);
        sajson::value field_value = root.get_object_value(i);

        if (strncmp(field_name.data(), "test_index", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_INTEGER) {
                m_test_index = field_value.get_integer_value();
            }
            continue;
        }

        if (strncmp(field_name.data(), "window_width", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_INTEGER) {
                m_window_width = field_value.get_integer_value();
            }
            continue;
        }

        if (strncmp(field_name.data(), "window_height", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_INTEGER) {
                m_window_height = field_value.get_integer_value();
            }
            continue;
        }

        if (strncmp(field_name.data(), "hertz", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_DOUBLE || field_value.get_type() == sajson::TYPE_INTEGER) {
                m_hertz = float(field_value.get_number_value());
            }
            continue;
        }

        if (strncmp(field_name.data(), "velocity_iteration", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_INTEGER) {
                m_velocity_iteration = field_value.get_integer_value();
            }
            continue;
        }

        if (strncmp(field_name.data(), "position_iteration", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_INTEGER) {
                m_position_iteration = field_value.get_integer_value();
            }
            continue;
        }

        if (strncmp(field_name.data(), "enable_sleep", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_enable_sleep = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_enable_sleep = true;
            }
            continue;
        }

        if (strncmp(field_name.data(), "draw_shapes", field_name.length()) == 0)
        {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_draw_shapes = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_draw_shapes = true;
            }
            continue;
        }

        if (strncmp(field_name.data(), "enable_continuous_physics", field_name.length()) == 0) {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_enable_continuous_physics = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_enable_continuous_physics = true;
            }
            continue;
        }

        if (strncmp(field_name.data(), "draw_frame_only", field_name.length()) == 0) {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_draw_frame_only = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_draw_frame_only = true;
            }
            continue;
        }

        if (strncmp(field_name.data(), "draw_contact_points", field_name.length()) == 0) {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_draw_contact_points = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_draw_contact_points = true;
            }
            continue;
        }
    }

    free(data);
}
