#include "weapon_docking/docking_control.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace weapon_docking {
namespace {

TEST(DockingControl, ConvertsMillimeterErrorsToCmdVel)
{
    const DockingControlParams params{
        {1.5, 0.0, 0.0},
        {1.5, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto target = MakeDockingTargetPose(
        200.0f,
        -100.0f,
        300.0f);
    const auto cmd = ComputeCmdVelFromDockingTarget(
        target,
        params,
        state,
        0.1);

    EXPECT_NEAR(cmd.linear.x, -0.15, 1e-9);
    EXPECT_NEAR(cmd.linear.y, -0.3, 1e-9);
    EXPECT_NEAR(cmd.angular.z, 0.6, 1e-9);
}

TEST(DockingControl, LimitsLinearAndAngularSpeed)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        0.5,
        0.8,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeCmdVelFromDockingTarget(
        DockingTargetPose{3.0, 4.0, 1.0},
        params);

    EXPECT_NEAR(std::hypot(cmd.linear.x, cmd.linear.y), 0.5, 1e-9);
    EXPECT_NEAR(cmd.angular.z, 0.8, 1e-9);
}

TEST(DockingControl, PidUsesIntegralAndDerivativeTerms)
{
    const DockingControlParams params{
        {1.0, 0.5, 0.2},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto first = ComputeCmdVelFromDockingTarget(
        DockingTargetPose{0.0, 1.0, 0.0},
        params,
        state,
        0.1);
    const auto second = ComputeCmdVelFromDockingTarget(
        DockingTargetPose{0.0, 0.6, 0.0},
        params,
        state,
        0.1);

    EXPECT_NEAR(first.linear.x, 1.0, 1e-9);
    EXPECT_NEAR(second.linear.x, 0.6, 1e-9);
}

TEST(DockingControl, InvalidMeasurementReturnsZeroCmdVel)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeCmdVelFromDockingError(
        999.0f,
        0.0f,
        0.0f,
        params);

    EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(DockingControl, ZeroLimitsDisableMotion)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        0.0,
        0.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeCmdVelFromDockingTarget(
        DockingTargetPose{0.2, 0.3, 0.4},
        params);

    EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(DockingControl, DetectsDockingTargetTolerance)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.02
    };

    EXPECT_TRUE(IsDockingTargetWithinTolerance(
        DockingTargetPose{0.006, 0.008, 0.01},
        params));
    EXPECT_FALSE(IsDockingTargetWithinTolerance(
        DockingTargetPose{0.011, 0.0, 0.01},
        params));
    EXPECT_FALSE(IsDockingTargetWithinTolerance(
        DockingTargetPose{0.006, 0.008, 0.03},
        params));
}

TEST(DockingControl, DetectsVisualAlignmentToleranceUsingOnlyCameraY)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.02
    };

    EXPECT_TRUE(IsDockingAlignmentWithinTolerance(
        DockingTargetPose{0.5, 0.006, 0.01},
        params));
    EXPECT_FALSE(IsDockingAlignmentWithinTolerance(
        DockingTargetPose{0.0, 0.011, 0.01},
        params));
    EXPECT_FALSE(IsDockingAlignmentWithinTolerance(
        DockingTargetPose{0.0, 0.006, 0.03},
        params));
}

TEST(DockingControl, DetectsCameraDistanceTolerance)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.02
    };

    EXPECT_TRUE(IsDockingXWithinTolerance(
        DockingTargetPose{0.006, 0.5, 0.5},
        params));
    EXPECT_FALSE(IsDockingXWithinTolerance(
        DockingTargetPose{0.011, 0.0, 0.0},
        params));
}

TEST(DockingControl, AdvancesVisualAlignStageOnlyWhenTargetIsWithinTolerance)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.02
    };

    EXPECT_EQ(NextDockingStageAfterTargetCheck(
        DockingStage::kVisualAlign,
        DockingTargetPose{0.5, 0.008, 0.01},
        params), DockingStage::kOpenLoopX);
    EXPECT_EQ(NextDockingStageAfterTargetCheck(
        DockingStage::kVisualAlign,
        DockingTargetPose{0.02, 0.011, 0.01},
        params), DockingStage::kVisualAlign);
    EXPECT_EQ(NextDockingStageAfterTargetCheck(
        DockingStage::kDone,
        DockingTargetPose{0.02, 0.0, 0.01},
        params), DockingStage::kDone);
}

TEST(DockingControl, BuildsTargetPoseFromMeasuredDistanceToRequestedDistance)
{
    const auto target = MakeDockingTargetPose(
        1120.0f,
        -20.0f,
        5.0f,
        0.60);

    EXPECT_NEAR(target.x_error, 0.52, 1e-9);
    EXPECT_NEAR(target.y_error, -0.02, 1e-9);
    EXPECT_NEAR(target.theta_error, 0.005, 1e-9);
}

TEST(DockingControl, VisualAlignCmdVelMapsCameraYToChassisX)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.5, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeVisualAlignCmdVel(
        DockingTargetPose{0.4, 0.2, 0.3},
        params,
        state,
        0.1);

    EXPECT_NEAR(cmd.linear.x, 0.08, 1e-9);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
    EXPECT_NEAR(cmd.angular.z, 0.6, 1e-9);
}

TEST(DockingControl, VisualAlignCmdVelUsesMinimumYSpeedOutsideTolerance)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {0.45, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        0.3,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeVisualAlignCmdVel(
        DockingTargetPose{0.0, 0.018, 0.0},
        params,
        state,
        0.1);

    EXPECT_NEAR(cmd.linear.x, 0.04, 1e-9);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(DockingControl, XMoveCmdVelMapsCameraXToChassisY)
{
    const DockingControlParams params{
        {1.0, 0.0, 0.0},
        {1.5, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        1.0,
        1.0,
        0.01,
        0.01
    };
    DockingControlState state;

    const auto cmd = ComputeXMoveCmdVel(
        DockingTargetPose{0.4, 0.2, 0.3},
        params,
        state,
        0.1);

    EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
    EXPECT_NEAR(cmd.linear.y, 0.4, 1e-9);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

}  // namespace
}  // namespace weapon_docking
