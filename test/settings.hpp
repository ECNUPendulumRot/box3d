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
        m_hertz = 60.0f;
        m_test_index = 0;
        m_single_step = false;
        m_show_contact_points = false;
    }

    void save() const;
    void load();

    int m_window_width;
    int m_window_height;

    int m_test_index = 0;

    bool m_draw_shapes = true;
    bool m_draw_frame_only = false;
    bool m_pause = false;
    bool m_single_step = false;
    bool m_show_contact_points = false;
    float m_hertz;
};

#endif //BOX3D_SETTINGS_HPP
