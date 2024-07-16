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


#ifndef BOX3D_B3_TIMER_HPP
#define BOX3D_B3_TIMER_HPP


#include <chrono>
#include <thread>

/**
 * @brief The class is designed to measure elapsed time.
 */
class b3Timer {

    /**
     * @brief Stores the start time or the last reset time of the timer.
     */
    std::chrono::steady_clock::time_point m_t0;

    /**
     * @brief Intended to store the end time for measuring the duration.
     */
    std::chrono::steady_clock::time_point m_t1;

public:

    /**
     * @brief the constructor of b3Timer
     * @param m_target_hz Intended to set a target frequency in Hz
     */
    explicit b3Timer(double m_target_hz) {
        m_t0 = std::chrono::steady_clock::now();
    }

    /**
     * @brief the constructor of b3Timer
     */
    b3Timer(): b3Timer(0) {
        m_t0 = std::chrono::steady_clock::now();
    }

    /**
     * @brief Resets the start time of the timer to the current time.
     */
    void reset() {
        m_t0 = std::chrono::steady_clock::now();
    }

    /**
     * @brief Calculates the elapsed time in milliseconds since the timer was last started or reset.
     * @return  double representing the elapsed time in milliseconds.
     */
    double get_time_ms() {
        std::chrono::duration<double> duration = std::chrono::steady_clock::now() - m_t0;
        return duration.count() * 1000.0;
    }
};


#endif //BOX3D_B3_TIMER_HPP
