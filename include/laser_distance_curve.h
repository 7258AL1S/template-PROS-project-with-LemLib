#pragma once

#include <cstdint>

namespace laser_distance_curve {

enum class CommandState {
    DRIVE,
    ARRIVED,
    INVALID_READING,
    INVALID_ARGUMENT,
};

struct Command {
    float power;
    CommandState state;
};

/**
 * @brief 根据前置激光距离计算底盘直行功率
 * @param Power             最大功率绝对值 [0, 1.0]
 * @param CurrentDistanceMm 激光当前读数（毫米）
 * @param TargetDistanceMm  目标激光距离（毫米，恒为正）
 * @param DecelDistanceMm   目标两侧的线性减速区（毫米，恒为正）
 */
Command calculateCommand(float Power,
                         std::int32_t CurrentDistanceMm,
                         float TargetDistanceMm,
                         float DecelDistanceMm);

}  // namespace laser_distance_curve
