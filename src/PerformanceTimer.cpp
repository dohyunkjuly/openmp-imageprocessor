#include "PerformanceTimer.h"
#include <numeric>
#include <sstream>
#include <iomanip>

PerformanceTimer::PerformanceTimer() {
    reset();
}

void PerformanceTimer::start() {
    startTime = std::chrono::high_resolution_clock::now();
}

void PerformanceTimer::stop() {
    endTime = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    duration = d.count() / 1000; // Store in Milliseconds
}

double PerformanceTimer::getDuration() const {
    return duration;
}

void PerformanceTimer::reset() {
    duration = 0.0;
}

std::string PerformanceTimer::getFormattedResults() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Execution time: " << getDuration() << " ms";
    return ss.str();
} 