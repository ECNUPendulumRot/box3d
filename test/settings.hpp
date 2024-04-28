//
// Created by sherman on 24-4-25.
//

#ifndef BOX3D_SETTINGS_HPP
#define BOX3D_SETTINGS_HPP

struct Settings {

    Settings() {
        reset();
    }

    void reset() {
        m_window_width = 1600;
        m_window_height = 900;

        m_test_index = 0;
    }

    int m_window_width;
    int m_window_height;

    int m_test_index = 0;

    bool m_draw_shapes = true;
};

#endif //BOX3D_SETTINGS_HPP
