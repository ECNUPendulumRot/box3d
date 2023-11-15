//
// Created by sherman on 23-11-15.
//

#ifndef BOX3D_B3_GUI_VIEWER_HPP
#define BOX3D_B3_GUI_VIEWER_HPP


#include <memory> // shared_ptr

#include <spdlog/spdlog.h>

#include <igl/Timer.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "dynamics/b3_world.hpp"

#include "utils/b3_igl_viewer_ext.hpp"

namespace ipc::rigid {

    class UISimState : public igl::opengl::glfw::imgui::ImGuiMenu {
        typedef igl::opengl::glfw::imgui::ImGuiMenu Super;

    public:
        UISimState();
        ~UISimState() override {}

        enum PlayerState { Playing = 0, Paused, TotalPlayerStatus };

        // virtual void init(igl::opengl::glfw::Viewer* _viewer) override;

        // virtual void draw_menu() override;
        // virtual bool mouse_down(int button, int modifier) override;
        // virtual bool key_pressed(unsigned int key, int modifiers) override;

        std::shared_ptr<igl::opengl::ViewerDataExt>
        get_data(const std::string& data) const;

        void launch(const std::string& inital_scene);
        void load_scene();
        void redraw_scene();
        bool pre_draw_loop();
        bool post_draw_loop();

        bool custom_key_pressed(unsigned int unicode_key, int modifiers);

        bool load(std::string scene_filename) override
        {
            if (scene_filename != "" && m_state.load_scene(scene_filename)) {
                load_scene();
                return true;
            }
            return false;
        }
        bool save(std::string scene_filename) override
        {
            return m_state.save_simulation(scene_filename);
        }

        bool save_obj_sequence(const std::string& dir_name)
        {
            bool success = m_state.save_obj_sequence(dir_name);
            m_state.problem_ptr->state(
                    m_state.state_sequence[m_state.m_num_simulation_steps]);
            return success;
        }

        bool save_gltf(const std::string& filename)
        {
            return m_state.save_gltf(filename);
        }

        void get_window_dimensions(int& width, int& height) const
        {
            width = m_viewer.core().viewport[2] - m_viewer.core().viewport[0];
            height = m_viewer.core().viewport[3] - m_viewer.core().viewport[1];
        }

        void reload()
        {
            m_reloading_scene = true;
            load_scene();
            m_reloading_scene = false;
        }

        void simulation_step()
        {
            igl::Timer timer;
            timer.start();
            // TODO: add world simulation here
            /// m_state.simulation_step();
            timer.stop();
            m_simulation_time += timer.getElapsedTime();

            redraw_scene();
        }

        igl::opengl::glfw::Viewer m_viewer;
        PlayerState m_player_state;
        bool replaying = false;

        bool m_has_scene;
        bool m_bkp_had_collision;
        bool m_bkp_has_intersections;
        bool m_bkp_optimization_failed;
        double m_interval_time; ///< @brief time within the interval
        bool m_show_vertex_data;

    protected:

        void draw_io();
        void draw_simulation_player();
        void draw_settings();
        void draw_legends();

    private:
        std::shared_ptr<igl::opengl::MeshData> mesh_data;
        std::shared_ptr<igl::opengl::VectorFieldData> velocity_data;

        std::vector<
                std::pair<std::string, std::shared_ptr<igl::opengl::ViewerDataExt>>>
                datas_;

        bool m_reloading_scene;

        bool m_scene_changed;

        double m_simulation_time;

        std::string inital_scene;
    };

} // namespace ipc::rigid

#endif //BOX3D_B3_GUI_VIEWER_HPP
