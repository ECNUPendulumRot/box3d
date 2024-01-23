
#ifndef BOX3D_B3_SIM_APP_HPP
#define BOX3D_B3_SIM_APP_HPP


#include <filesystem>
#include <vector>

#include <CLI/CLI.hpp>

#include "b3_scene_test.hpp"
#include "b3_gui_viewer.hpp"

#define MAX_TEST 256

namespace box3d {

    class b3World;

}


class b3SimApp {

    std::vector<std::string> m_mesh_paths;

    std::vector<std::string> m_fixture_paths;

    CLI::App m_app;

    b3GUIViewer m_viewer;

public:

    b3SimApp() = default;

    ~b3SimApp() = default;

    int launch();

    inline void set_gui_fps(double fps) {
        m_viewer.set_max_fps(fps);
    }

private:

};


#endif //BOX3D_B3_SIM_APP_HPP
