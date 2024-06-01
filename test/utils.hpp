
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
