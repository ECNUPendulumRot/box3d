
#ifndef BOX3D_ANIMATION_GENERATOR_HPP
#define BOX3D_ANIMATION_GENERATOR_HPP


#include <vector>
#include <fstream>
#include <iostream>
#include "box3d.hpp"
#include "gl_render_triangles.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

#include <sstream>


class AnimationGenerator {

    int32 frame_count = 0;

public:

    void save_to_obj(gl_render_triangles* render_data, const std::string& filename) {

        if (CreateDirectory("obj_files", NULL) || ERROR_ALREADY_EXISTS == GetLastError()) {
            std::cout << "Directory created successfully" << std::endl;
        } else {
            std::cerr << "Error creating directory: " << GetLastError() << std::endl;
        }
        std::ostringstream oss;
        oss << std::setw(8) << std::setfill('0') << frame_count++;

        std::ofstream obj_file("obj_files/" + filename + oss.str() + ".obj");
        if (!obj_file.is_open()) {
            std::cerr << "Cannot open the file: " << filename << std::endl;
            return;
        }

        b3Vec3r* vertices = render_data->m_vertices;
        b3Vec3i* faces = render_data->m_faces;
        for (int i = 0; i < render_data->m_v_count; ++i) {
            obj_file << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << "\n";
        }

        for (int i = 0; i < render_data->m_f_count; ++i) {
            obj_file << "f " << faces[i].x + 1 << " " << faces[i].y + 1 << " " << faces[i].z + 1 << "\n";
        }

        // 关闭文件
        obj_file.close();
        std::cout << "OBJ file saved success: " << filename << std::endl;
    }

};


extern AnimationGenerator g_animation_generator;

#endif //BOX3D_ANIMATION_GENERATOR_HPP
