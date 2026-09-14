#include "laser_distance_curve.h"

#include <cassert>
#include <cmath>

namespace {

bool nearlyEqual(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 0.0001f;
}

}  // namespace

int main() {
    using laser_distance_curve::CommandState;
    using laser_distance_curve::calculateCommand;

    // A larger front distance means the target is farther away, so drive forward.
    auto command = calculateCommand(1.0f, 700, 500.0f, 100.0f);
    assert(command.state == CommandState::DRIVE);
    assert(nearlyEqual(command.power, 1.0f));

    // A smaller front distance means the robot is too close, so drive backward.
    command = calculateCommand(1.0f, 300, 500.0f, 100.0f);
    assert(command.state == CommandState::DRIVE);
    assert(nearlyEqual(command.power, -1.0f));

    // The same curve must decelerate on either side of the target.
    command = calculateCommand(1.0f, 550, 500.0f, 100.0f);
    assert(command.state == CommandState::DRIVE);
    assert(nearlyEqual(command.power, 0.5f));

    command = calculateCommand(1.0f, 450, 500.0f, 100.0f);
    assert(command.state == CommandState::DRIVE);
    assert(nearlyEqual(command.power, -0.5f));

    // Within 10 mm, stop instead of repeatedly reversing around the target.
    command = calculateCommand(1.0f, 509, 500.0f, 100.0f);
    assert(command.state == CommandState::ARRIVED);
    assert(nearlyEqual(command.power, 0.0f));

    // PROS reports 9999 when no object is detected; fail closed in that case.
    command = calculateCommand(1.0f, 9999, 500.0f, 100.0f);
    assert(command.state == CommandState::INVALID_READING);
    assert(nearlyEqual(command.power, 0.0f));

    // Target and deceleration distances are required to be positive millimetres.
    command = calculateCommand(1.0f, 500, 0.0f, 100.0f);
    assert(command.state == CommandState::INVALID_ARGUMENT);
    assert(nearlyEqual(command.power, 0.0f));

    command = calculateCommand(1.0f, 500, 400.0f, 0.0f);
    assert(command.state == CommandState::INVALID_ARGUMENT);
    assert(nearlyEqual(command.power, 0.0f));

    return 0;
}
