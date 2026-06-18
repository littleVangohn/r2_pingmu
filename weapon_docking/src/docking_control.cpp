#include "weapon_docking/docking_control.hpp"

#include <algorithm>
#include <cmath>

namespace weapon_docking {

namespace {

constexpr double kMinVisualAlignChassisXSpeed = 0.04;
constexpr double kMaxVisualAlignChassisXSpeed = 0.08;

double ClampAbs(double value, double limit)
{
    const double abs_limit = std::max(0.0, limit);
    return std::clamp(value, -abs_limit, abs_limit);
}

double ComputePid(
    double error,
    const DockingPidConfig & config,
    DockingPidState & state,
    double dt)
{
    const double safe_dt = dt > 1e-6 ? dt : 1e-6;
    state.integral += error * safe_dt;
    const double derivative = state.has_previous_error
        ? (error - state.previous_error) / safe_dt
        : error / safe_dt;
    state.previous_error = error;
    state.has_previous_error = true;
    return config.kp * error + config.ki * state.integral + config.kd * derivative;
}

}  // namespace

geometry_msgs::msg::Twist MakeZeroCmdVel()
{
    return geometry_msgs::msg::Twist();
}

bool IsValidDockingMeasurement(float x_error_mm, float y_error_mm, float z_error_mrad)
{
    constexpr float kInvalidMeasurement = 999.0f;
    return x_error_mm != kInvalidMeasurement
        && y_error_mm != kInvalidMeasurement
        && z_error_mrad != kInvalidMeasurement;
}

DockingTargetPose MakeDockingTargetPose(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad)
{
    return MakeDockingTargetPose(x_error_mm, y_error_mm, z_error_mrad, 0.0);
}

DockingTargetPose MakeDockingTargetPose(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    double target_distance_m)
{
    return DockingTargetPose{
        static_cast<double>(x_error_mm) / 1000.0 - target_distance_m,
        static_cast<double>(y_error_mm) / 1000.0,
        static_cast<double>(z_error_mrad) / 1000.0
    };
}

bool IsDockingTargetWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params)
{
    return std::hypot(target.x_error, target.y_error) <= params.hold_position_tolerance
        && std::abs(target.theta_error) <= params.hold_angle_tolerance;
}

bool IsDockingAlignmentWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params)
{
    return std::abs(target.y_error) <= params.hold_position_tolerance
        && std::abs(target.theta_error) <= params.hold_angle_tolerance;
}

bool IsDockingXWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params)
{
    return std::abs(target.x_error) <= params.hold_position_tolerance;
}

DockingStage NextDockingStageAfterTargetCheck(
    DockingStage current_stage,
    const DockingTargetPose & target,
    const DockingControlParams & params)
{
    if (current_stage == DockingStage::kVisualAlign
        && IsDockingAlignmentWithinTolerance(target, params)) {
        return DockingStage::kOpenLoopX;
    }
    return current_stage;
}

void ResetDockingControlState(DockingControlState & state)
{
    state.linear_x = DockingPidState();
    state.linear_y = DockingPidState();
    state.angular = DockingPidState();
}

geometry_msgs::msg::Twist ComputeCmdVelFromDockingTarget(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt)
{
    double lateral_v = ComputePid(target.x_error, params.linear_x_pid, state.linear_x, dt);
    double forward_v = ComputePid(target.y_error, params.linear_y_pid, state.linear_y, dt);

    const double linear_norm = std::hypot(lateral_v, forward_v);
    if (params.max_linear_speed <= 0.0) {
        lateral_v = 0.0;
        forward_v = 0.0;
    } else if (linear_norm > params.max_linear_speed) {
        const double scale = params.max_linear_speed / linear_norm;
        lateral_v *= scale;
        forward_v *= scale;
    }

    const double wz = ClampAbs(
        ComputePid(target.theta_error, params.angular_pid, state.angular, dt),
        params.max_angular_speed);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_v;
    cmd.linear.y = -lateral_v;
    cmd.angular.z = wz;
    return cmd;
}

geometry_msgs::msg::Twist ComputeVisualAlignCmdVel(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt)
{
    double lateral_v = ComputePid(target.y_error, params.linear_y_pid, state.linear_y, dt);

    if (params.max_linear_speed <= 0.0) {
        lateral_v = 0.0;
    } else {
        lateral_v = ClampAbs(
            lateral_v,
            std::min(params.max_linear_speed, kMaxVisualAlignChassisXSpeed));
        if (std::abs(target.y_error) > params.hold_position_tolerance
            && std::abs(lateral_v) < kMinVisualAlignChassisXSpeed) {
            lateral_v = std::copysign(kMinVisualAlignChassisXSpeed, target.y_error);
        }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = lateral_v;
    cmd.angular.z = ClampAbs(
        ComputePid(target.theta_error, params.angular_pid, state.angular, dt),
        params.max_angular_speed);
    return cmd;
}

geometry_msgs::msg::Twist ComputeXMoveCmdVel(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt)
{
    double forward_v = ComputePid(target.x_error, params.linear_x_pid, state.linear_x, dt);

    if (params.max_linear_speed <= 0.0) {
        forward_v = 0.0;
    } else {
        forward_v = ClampAbs(forward_v, params.max_linear_speed);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.y = forward_v;
    return cmd;
}

geometry_msgs::msg::Twist ComputeCmdVelFromDockingTarget(
    const DockingTargetPose & target,
    const DockingControlParams & params)
{
    DockingControlState state;
    return ComputeCmdVelFromDockingTarget(target, params, state, 0.1);
}

geometry_msgs::msg::Twist ComputeCmdVelFromDockingError(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt)
{
    if (!IsValidDockingMeasurement(x_error_mm, y_error_mm, z_error_mrad)) {
        ResetDockingControlState(state);
        return MakeZeroCmdVel();
    }

    return ComputeCmdVelFromDockingTarget(
        MakeDockingTargetPose(x_error_mm, y_error_mm, z_error_mrad),
        params,
        state,
        dt);
}

geometry_msgs::msg::Twist ComputeCmdVelFromDockingError(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    const DockingControlParams & params)
{
    DockingControlState state;
    return ComputeCmdVelFromDockingError(
        x_error_mm,
        y_error_mm,
        z_error_mrad,
        params,
        state,
        0.1);
}

}  // namespace weapon_docking
