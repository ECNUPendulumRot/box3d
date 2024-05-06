
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
    fprintf(file, "  \"draw_shapes\": %s,\n", m_draw_shapes ? "true" : "false");
    fprintf(file, "  \"draw_frame_only\": %s\n", m_draw_frame_only ? "true" : "false");
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

        if (strncmp(field_name.data(), "draw_frame_only", field_name.length()) == 0) {
            if (field_value.get_type() == sajson::TYPE_FALSE) {
                m_draw_frame_only = false;
            }
            else if (field_value.get_type() == sajson::TYPE_TRUE) {
                m_draw_frame_only = true;
            }
            continue;
        }
    }

    free(data);
}
