
#ifndef BODY_INFO_GENERATOR_HPP
#define BODY_INFO_GENERATOR_HPP

#include <vector>
#include <fstream>
#include <iostream>
#include "box3d.hpp"
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#endif

#include <sstream>

class BodyInfoGenerator {

    bool file_is_open = false;

    std::ofstream csv_file;

public:

    void open_file(std::string* names, int count) {

        if (file_is_open) {
            return;
        }

        std::filesystem::path path = "BodyInfo";

        std::filesystem::create_directories(path);
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
            std::cout << "Directory created successfully or already exists" << std::endl;
        } else {
            std::cerr << "Error creating directory" << std::endl;
        }

        int files_count = 0;
        std::filesystem::directory_entry entry(path);
        std::filesystem::directory_iterator entries(entry);
        for(auto& it : entries) {
            files_count++;
        }
        std::cout << "total files count: " << files_count << std::endl;

        std::string file_name = "BodyInfo/body_info_" + std::to_string(files_count) + ".csv";
        csv_file.open(file_name);
        if (!csv_file.is_open()) {
            std::cerr << "create file failed" << std::endl;
            return;
        }

        file_is_open = true;

        for (int i = 0; i < count; i++) {
            csv_file << names[i] + ","
                     << "p_x,"
                     << "p_y,"
                     << "p_z,"
                     << "q_x,"
                     << "q_y,"
                     << "q_z,"
                     << "q_w,"
                     << "v_x,"
                     << "v_y,"
                     << "v_z,"
                     << "w_x,"
                     << "w_y,"
                     << "w_z, ,";
        }
        csv_file << std::endl;
    }

    void save_body_info(b3Body** bodies, std::string* names, int count) {
        open_file(names, count);

        for(int i = 0; i < count; i++) {
            b3Body* body = bodies[i];

            b3Vec3r p = body->get_position();
            b3Quaternionr q = body->get_quaternion();
            b3Vec3r v = body->get_linear_velocity();
            b3Vec3r w = body->get_angular_velocity();

            csv_file << "," << p.x << "," << p.y << "," << p.z << ",";
            csv_file << q.m_x << "," << q.m_y << "," << q.m_z << "," << q.m_w << ",";
            csv_file << v.x << "," << v.y << "," << v.z << ",";
            csv_file << w.x << "," << w.y << "," << w.z << ", ,";
        }
        csv_file << std::endl;
    }

    void close_file() {
        if (file_is_open) {
            csv_file.close();
        }
    }

    ~BodyInfoGenerator() {
        close_file();
        file_is_open = false;
    }
};

#endif // BODY_INFO_GENERATOR_HPP