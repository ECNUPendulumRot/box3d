
#ifndef BOX3D_B3_TIMER_HPP
#define BOX3D_B3_TIMER_HPP


#include <chrono>
#include <thread>


class b3Timer {

    std::chrono::steady_clock::time_point m_t0;

    std::chrono::steady_clock::time_point m_t1;

public:

    explicit b3Timer(double m_target_hz) {
        m_t0 = std::chrono::steady_clock::now();
    }

    b3Timer(): b3Timer(0) {
        m_t0 = std::chrono::steady_clock::now();
    }

    void reset() {
        m_t0 = std::chrono::steady_clock::now();
    }

    double get_time_ms() {
        std::chrono::duration<double> duration = std::chrono::steady_clock::now() - m_t0;
        return duration.count() * 1000.0;
    }

    double get_time_us() {
        std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - m_t0);
        return duration.count();
    }
};


#endif //BOX3D_B3_TIMER_HPP
