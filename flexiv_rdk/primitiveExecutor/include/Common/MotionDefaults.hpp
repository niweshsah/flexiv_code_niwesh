#pragma once
#include <string>

namespace flexiv_executor::common {

/**
 * @brief Default motion parameters applied unless overridden by CLI.
 */
struct MotionDefaults {
    double vel = 0.30;
    double acc = 1.50;
    double angVel = 150.0;
    double jerk = 50.0;
    std::string zoneRadius = "ZFine";
    int targetTolerance = 3;

    bool hold = false; // <-- Add this new flag
};

} // namespace flexiv_executor::common