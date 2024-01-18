
#ifndef BOX3D_B3_TIMER_HPP
#define BOX3D_B3_TIMER_HPP


#include <chrono>
#include <thread>


class b3Timer {

    std::chrono::steady_clock::time_point m_t0;

    std::chrono::steady_clock::time_point m_t1;

    std::chrono::duration<double> m_time_step;

    static std::chrono::duration<double> m_sleep_adjust;

    static double m_filter_param;

public:

    explicit b3Timer(double m_target_hz) {
        m_time_step = std::chrono::duration<double>(m_target_hz == 0 ? 0: 1.0 / m_target_hz);
        m_t0 = std::chrono::steady_clock::now();
    }

    b3Timer(): b3Timer(0) {
        m_t0 = std::chrono::steady_clock::now();
    }

    void sleep() {

        m_t1 = std::chrono::steady_clock::now();
        auto sleep_time = m_time_step - (m_t0 - m_t1) + m_sleep_adjust;
        if (sleep_time > std::chrono::duration<double>(0)) {
            std::this_thread::sleep_for(sleep_time);
        }

        auto t2 = std::chrono::steady_clock::now();
        auto frame_time = t2 - m_t0;

        m_sleep_adjust = (1.0 - m_filter_param) * m_sleep_adjust + m_filter_param * (m_time_step - frame_time);

    }

};

std::chrono::duration<double> b3Timer::m_sleep_adjust = std::chrono::duration<double>(0);
double b3Timer::m_filter_param = 0.1;

#endif //BOX3D_B3_TIMER_HPP
