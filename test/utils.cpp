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

#include "utils.hpp"
#include <filesystem>
#include <iostream>
#include <spdlog/spdlog.h>


void Utils::open_file()
{

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

    for (int i = 0; i < m_body_count; i++) {
        csv_file << m_body_names[i] + ","
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


void Utils::save_body_info()
{
    open_file();

    for(int i = 0; i < m_body_count; i++) {
        b3Body* body = m_bodies[i];

        b3Vec3r p = body->get_position();
        b3Quatr q = body->get_quaternion();
        b3Vec3r v = body->get_linear_velocity();
        b3Vec3r w = body->get_angular_velocity();

        csv_file << "," << p.x << "," << p.y << "," << p.z << ",";
        csv_file << q.m_x << "," << q.m_y << "," << q.m_z << "," << q.m_w << ",";
        csv_file << v.x << "," << v.y << "," << v.z << ",";
        csv_file << w.x << "," << w.y << "," << w.z << ", ,";
    }
    csv_file << std::endl;
}


void Utils::track_fixture(b3Fixture *fixture, const char *name)
{
    using json = nlohmann::json;

    json fixture_json;

    fixture_json["name"] = std::string(name) + "_box3d";
    b3Shape* shape = fixture->get_shape();
    switch (shape->get_type()) {
        case b3ShapeType::e_sphere: {
            fixture_json["type"] = "sphere";
            fixture_json["radius"] = fixture->get_shape()->get_radius();
            break;
        }
        case b3ShapeType::e_cube: {
            b3CubeShape* cube = (b3CubeShape*)shape;
            fixture_json["type"] = "cube";
            std::array<float, 3> hf = {cube->m_h_xyz.x, cube->m_h_xyz.y, cube->m_h_xyz.z};
            fixture_json["hf"] = hf;
            break;
        }
        case b3ShapeType::e_plane: {
            b3PlaneShape* plane = (b3PlaneShape*)shape;
            fixture_json["type"] = "plane";
            std::array<float, 2> lw = {plane->m_half_width, plane->m_half_length};
            fixture_json["hf"] = lw;
            break;
        }
        default:
            break;
    }

    m_meta_json.push_back(fixture_json);

    m_fixtures[m_fixture_count++] = fixture;
}


void Utils::track_body(b3Body *body, const char *name) {
    m_bodies[m_body_count] = body;
    m_body_names[m_body_count] = name;
    m_body_count++;
}


void Utils::record_frame()
{

    using json = nlohmann::json;

    json frame_json;

    for (int i = 0; i < m_fixture_count; ++i) {
        b3Quatr q = m_fixtures[i]->get_body()->get_quaternion();
        b3Vec3r p = m_fixtures[i]->get_body()->get_position();
        json status_json;
        status_json["position"] = {p.x, p.y, p.z};
        status_json["quaternion"] = {q.m_w, q.m_x, q.m_y, q.m_z};
        frame_json[m_meta_json[i]["name"]] = status_json;
    }

    m_frame_json.push_back(frame_json);
}


void Utils::save_json_file()
{
    nlohmann::json json;
    json["meta"] = m_meta_json;
    json["frames"] = m_frame_json;

    std::ofstream file("scene.json");
    if (file.is_open()) {
        file << json.dump(4);
        file.close();
    } else {
        spdlog::error("Failed to open file");
    }
}


