#pragma once

#include <chrono>
#include <vector>
#include <string>

class PerformanceTimer {
private:
    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point endTime;
    double duration;
public:
    PerformanceTimer();
    void start();
    void stop();
    double getDuration() const;
    void reset();
    
    template<typename Func>
    double measureExecutionTime(Func func) {
        reset();
        start();
        func();
        stop();
        return getDuration();
    }
    std::string getFormattedResults() const;
}; 