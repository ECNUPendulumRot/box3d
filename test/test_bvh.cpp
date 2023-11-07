#include "box3d.hpp"

#include <filesystem>


int main(int argc, char* argv[]) {

    set_logger_level(spdlog::level::info);

    box3d::b3Node* node;

    if(__is_trivially_copyable(box3d::b3AABB)){
        spdlog::warn("b3AABB is trivially copyable");
    }

    return 0;
}

