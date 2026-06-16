#include "lemlib/motions/moveToPose.hpp"
#include "LemLog/logger/Helper.hpp"
#include "lemlib/MotionCancelHelper.hpp"
#include "lemlib/Timer.hpp"
#include "lemlib/util.hpp"

using namespace units;

namespace lemlib {

static logger::Helper logHelper("lemlib/motions/moveToPose");

void moveToPose(Pose target, Time timeout, MoveToPoseParams params, MoveToPoseSettings settings) {
    // 初始化持久变量
    Pose lastPose = settings.poseGetter();
    Timer timer(timeout);
    bool close = false;
    bool prevSameSide = false;
    Number prevLateralOut = 0;
    Number prevAngularOut = 0;

    lemlib::MotionCancelHelper helper(10_msec);
    // 循环直到运动被取消或超时
    while (helper.wait() && !timer.isDone()) {
        const Pose pose = settings.poseGetter();

        // 检查机器人是否足够接近目标，进入沉降阶段
        if (pose.distanceTo(target) < 7.5_in && close == false) {
            close = true;
            params.maxLateralSpeed = max(abs(prevLateralOut), 0.47);
        }

        // 计算胡萝卜点（引导点）
        const V2Position carrot = [&] -> V2Position {
            if (close) return target;
            return target - V2Position::fromPolar(target.orientation, params.lead * pose.distanceTo(target));
        }();

        // 计算误差
        const Length lateralError = [&] {
            Length out = pose.distanceTo(target);
            // 沉降阶段使用全余弦缩放，远距离仅用符号
            const Number scalar = cos(angleError(pose.orientation, pose.angleTo(carrot)));
            if (close) out *= scalar;
            else out *= sgn(scalar);
            return out;
        }();
        const Angle angularError = [&] {
            const Angle adjustedTheta = params.reversed ? pose.orientation + 180_stDeg : pose.orientation;
            if (close) return angleError(adjustedTheta, target.orientation);
            else return angleError(adjustedTheta, pose.angleTo(carrot));
        }();

        // 检查退出条件（需同时满足横向和角向）
        if (settings.lateralExitConditions.update(lateralError) &&
            settings.angularExitConditions.update(angularError) && close) {
            break;
        }
        {
            const bool robotSide = (pose.y - target.y) * -sin(target.orientation) <=
                                   (pose.x - target.x) * cos(target.orientation) + params.earlyExitRange;
            const bool carrotSide = (carrot.y - target.y) * -sin(target.orientation) <=
                                    (carrot.x - target.x) * cos(target.orientation) + params.earlyExitRange;
            const bool sameSide = robotSide == carrotSide;
            // 机器人越过目标朝向垂线则退出
            if (!sameSide && prevSameSide && close && params.minLateralSpeed != 0) break;
            prevSameSide = sameSide;
        }

        // 计算横向和角向输出
        const Number angularOut = [&] -> Number {
            // 从 PID 获取原始输出
            Number out = settings.angularPID.update(to_stRad(angularError));
            // 限制最大速度
            out = clamp(out, -params.maxAngularSpeed, params.maxAngularSpeed);
            // 更新前次角向输出
            prevAngularOut = out;
            return out;
        }();
        const Number lateralOut = [&] -> Number {
            // 从 PID 获取原始输出
            Number out = settings.lateralPID.update(to_m(lateralError));
            // 限制最大速度
            out = clamp(out, -params.maxLateralSpeed, params.maxLateralSpeed);
            // 非沉降阶段限制加速度
            if (!close) out = slew(out, prevLateralOut, params.lateralSlew, helper.getDelta());
            // 防侧滑：根据转弯半径限制速度
            const Length radius = 1 / abs(getSignedTangentArcCurvature(pose, carrot));
            const Number maxSlipSpeed = sqrt(params.driftCompensation * to_m(radius));
            out = clamp(out, -maxSlipSpeed, maxSlipSpeed);
            // 角向优先：角向功率不足时削减横向
            const Number overturn = abs(angularOut) + abs(out) - params.maxLateralSpeed;
            if (overturn > 0) out -= out > 0 ? overturn : -overturn;
            // 防止反向运动（远距离严格单向）
            if (params.reversed && !close) out = units::min(out, 0);
            else if (!params.reversed && !close) out = max(out, 0);
            // 限制最小速度（克服静摩擦）
            if (params.reversed && -out < abs(params.minLateralSpeed) && out < 0) out = -abs(params.minLateralSpeed);
            else if (!params.reversed && out < abs(params.minLateralSpeed) && out > 0)
                out = abs(params.minLateralSpeed);
            // 更新前次横向输出
            prevLateralOut = out;
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
} // namespace lemlib
