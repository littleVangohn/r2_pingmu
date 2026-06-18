#ifndef WEAPON_DOCKING_DOCKING_CONTROL_HPP_
#define WEAPON_DOCKING_DOCKING_CONTROL_HPP_

#include "geometry_msgs/msg/twist.hpp"

namespace weapon_docking {

struct DockingPidConfig {
    double kp;
    double ki;
    double kd;
};

struct DockingPidState {
    double integral = 0.0;
    double previous_error = 0.0;
    bool has_previous_error = false;
};

struct DockingTargetPose {
    double x_error;
    double y_error;
    double theta_error;
};

struct DockingControlParams {
    DockingPidConfig linear_x_pid;
    DockingPidConfig linear_y_pid;
    DockingPidConfig angular_pid;
    double max_linear_speed;
    double max_angular_speed;
    double hold_position_tolerance;
    double hold_angle_tolerance;
};

struct DockingControlState {
    DockingPidState linear_x;
    DockingPidState linear_y;
    DockingPidState angular;
};

enum class DockingStage {
    kVisualAlign,
    kOpenLoopX,
    kDone,
};

geometry_msgs::msg::Twist MakeZeroCmdVel();

bool IsValidDockingMeasurement(float x_error_mm, float y_error_mm, float z_error_mrad);

DockingTargetPose MakeDockingTargetPose(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad);

DockingTargetPose MakeDockingTargetPose(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    double target_distance_m);

bool IsDockingTargetWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params);

bool IsDockingAlignmentWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params);

bool IsDockingXWithinTolerance(
    const DockingTargetPose & target,
    const DockingControlParams & params);

DockingStage NextDockingStageAfterTargetCheck(
    DockingStage current_stage,
    const DockingTargetPose & target,
    const DockingControlParams & params);

void ResetDockingControlState(DockingControlState & state);

geometry_msgs::msg::Twist ComputeCmdVelFromDockingTarget(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt);

geometry_msgs::msg::Twist ComputeCmdVelFromDockingTarget(
    const DockingTargetPose & target,
    const DockingControlParams & params);

geometry_msgs::msg::Twist ComputeVisualAlignCmdVel(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt);

geometry_msgs::msg::Twist ComputeXMoveCmdVel(
    const DockingTargetPose & target,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt);

geometry_msgs::msg::Twist ComputeCmdVelFromDockingError(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    const DockingControlParams & params,
    DockingControlState & state,
    double dt);

geometry_msgs::msg::Twist ComputeCmdVelFromDockingError(
    float x_error_mm,
    float y_error_mm,
    float z_error_mrad,
    const DockingControlParams & params);

}  // namespace weapon_docking

#endif  // WEAPON_DOCKING_DOCKING_CONTROL_HPP_
