#include "laser_distance_curve.h"

#include <cmath>

namespace laser_distance_curve {

Command calculateCommand(float Power,
                         std::int32_t CurrentDistanceMm,
                         float TargetDistanceMm,
                         float DecelDistanceMm) {
    constexpr float kArrivalDistanceMm = 10.0f;
    constexpr float kBreakawayPower = 0.02f;
    constexpr std::int32_t kNoObjectDistanceMm = 9999;

    float maxPower = std::fabs(Power);
    if (maxPower <= 0.0f || TargetDistanceMm <= 0.0f || DecelDistanceMm <= 0.0f) {
        return {0.0f, CommandState::INVALID_ARGUMENT};
    }
    if (CurrentDistanceMm <= 0 || CurrentDistanceMm >= kNoObjectDistanceMm) {
        return {0.0f, CommandState::INVALID_READING};
    }
    if (maxPower > 1.0f) maxPower = 1.0f;

    // 前置传感器距离偏大时前进，距离偏小时后退。
    const float error = static_cast<float>(CurrentDistanceMm) - TargetDistanceMm;
    const float absoluteError = std::fabs(error);
    if (absoluteError < kArrivalDistanceMm) {
        return {0.0f, CommandState::ARRIVED};
    }

    const float direction = error > 0.0f ? 1.0f : -1.0f;
    float outputMagnitude = maxPower;
    if (absoluteError <= DecelDistanceMm) {
        outputMagnitude *= absoluteError / DecelDistanceMm;
        if (outputMagnitude < kBreakawayPower) outputMagnitude = kBreakawayPower;
    }

    return {direction * outputMagnitude, CommandState::DRIVE};
}

}  // namespace laser_distance_curve
