#include "lemlib/motions/moveToPoint.hpp"
#include "LemLog/logger/Helper.hpp"
#include "lemlib/MotionCancelHelper.hpp"
#include "lemlib/Timer.hpp"
#include "lemlib/util.hpp"

using namespace units;

namespace lemlib {

static logger::Helper logHelper("lemlib/motions/moveToPoint");

void moveToPoint(V2Position target, Time timeout, MoveToPointParams params, MoveToPointSettings settings) {
    logHelper.info("moving to point {}", target);

    // 初始化持久变量
    const Angle initialAngle = settings.poseGetter().angleTo(target);
    Timer timer(timeout);
    bool close = false;
    std::optional<bool> prevSide = std::nullopt;
    Number prevLateralOut = 0;
    Number prevAngularOut = 0;

    lemlib::MotionCancelHelper helper(10_msec); // 取消辅助器
    // 循环直到运动被取消或超时
    while (helper.wait() && !timer.isDone()) {
        // 获取位姿
        const Pose pose = settings.poseGetter();

        // 检查机器人是否足够接近目标，进入沉降阶段
        if (!close && pose.distanceTo(target) < 7.5_in) {
            close = true;
            params.maxLateralSpeed = max(abs(prevLateralOut), 4.7);
            params.maxAngularSpeed = max(abs(prevLateralOut), 4.7);
        }

        // 计算误差
        const Length lateralError = pose.distanceTo(target) * cos(angleError(pose.orientation, pose.angleTo(target)));
        const Angle angularError = [&] {
            const Angle adjustedTheta = params.reversed ? pose.orientation + 180_stDeg : pose.orientation;
            return angleError(adjustedTheta, pose.angleTo(target));
        }();

        // 检查退出条件
        if (settings.exitConditions.update(lateralError) && close) break;
        {
            const bool side = (pose.y - target.y) * -sin(initialAngle) <=
                              (pose.x - target.x) * cos(initialAngle) + params.earlyExitRange;
            if (prevSide == std::nullopt) prevSide = side;
            const bool sameSide = side == prevSide;
            // 已越过目标则退出
            if (!sameSide && params.minLateralSpeed != 0) break;
            prevSide = side;
        }

        // 计算横向和角向输出
        const Number lateralOut = [&] -> Number {
            // 从 PID 获取原始输出
            auto out = settings.lateralPID.update(to_m(lateralError));
            // 限制最大速度
            out = clamp(out, -params.maxLateralSpeed, params.maxLateralSpeed);
            // 非沉降阶段限制加速度
            out = close ? out : slew(out, prevLateralOut, params.lateralSlew, helper.getDelta());
            // 限制最小速度
            if (!close && params.reversed) out = clamp(out, -params.maxLateralSpeed, -params.minLateralSpeed);
            else if (!close && !params.reversed) out = clamp(out, params.minLateralSpeed, params.maxLateralSpeed);
            // 更新前值
            prevLateralOut = out;
            return out;
        }();
        const Number angularOut = [&] -> Number {
            // 沉降阶段关闭转向
            if (close) return 0;
            // 从 PID 获取原始输出
            auto out = settings.angularPID.update(to_stRad(angularError));
            // 限制最大速度
            out = clamp(out, -params.maxAngularSpeed, params.maxAngularSpeed);
            // 非沉降阶段限制加速度
            out = slew(out, prevAngularOut, params.angularSlew, helper.getDelta());
            // 更新前值
            prevAngularOut = out;
            return out;
        }();

        // 输出调试信息
        logHelper.debug("Moving with {:.4f} lateral power, {:.4f} angular power, {:.4f} lateral error, {:.4f} angular "
                        "error, {:.4f} dt",
                        lateralOut, angularOut, lateralError, angularError, helper.getDelta());

        // 计算底盘输出
        const auto out = desaturate(lateralOut, angularOut);
        // 驱动底盘
        settings.leftMotors.move(out.left);
        settings.rightMotors.move(out.right);
    }
    // 停止电机
    settings.leftMotors.brake();
    settings.rightMotors.brake();
}

}; // namespace lemlib