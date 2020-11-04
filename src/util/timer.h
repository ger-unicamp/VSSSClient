#ifndef _TIMER_HCC
#define _TIMER_HCC

#include <chrono>

class Timer
{
    std::chrono::_V2::steady_clock::time_point begin;

public:
    Timer() : begin(std::chrono::steady_clock::now()) {}

    /**
     * @brief Returns time reference in miliseconds
     * 
     * @return int64_t 
     */
    int64_t operator()()
    {
        auto round = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(round - this->begin).count();
    }
};

#endif