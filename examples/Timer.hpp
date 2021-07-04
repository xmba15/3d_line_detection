/**
 * @file    Timer.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <chrono>
#include <stdexcept>

namespace util
{
class Timer
{
 public:
    Timer()
        : m_start(std::chrono::steady_clock::time_point::min())
    {
    }

    void clear()
    {
        m_start = std::chrono::steady_clock::time_point::min();
    }

    bool isStarted() const
    {
        return (m_start != std::chrono::steady_clock::time_point::min());
    }

    void start()
    {
        m_start = std::chrono::steady_clock::now();
    }

    std::int64_t getMs() const
    {
        if (!this->isStarted()) {
            throw std::runtime_error("timer has not been started");
        }

        const std::chrono::steady_clock::duration diff = std::chrono::steady_clock::now() - m_start;
        return std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
    }

 private:
    std::chrono::steady_clock::time_point m_start;
};
}  // namespace util
