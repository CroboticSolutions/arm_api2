// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2024-2026 Crobotic Solutions d.o.o.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Unit tests for the pure decision logic of moveit2_iface (arm_api2/motion_logic.hpp).

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "arm_api2/motion_logic.hpp"

using namespace arm_api2::logic;  // NOLINT

namespace
{
trajectory_msgs::msg::JointTrajectory traj(
  const std::vector<std::string> & names, const std::vector<std::vector<double>> & pts)
{
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = names;
  for (const auto & p : pts) {
    trajectory_msgs::msg::JointTrajectoryPoint q;
    q.positions = p;
    t.points.push_back(q);
  }
  return t;
}
}  // namespace

TEST(FmtRad, FourDecimals) {
        EXPECT_EQ(fmtRad(0.0), "0.0000");
        EXPECT_EQ(fmtRad(0.123456), "0.1235");
        EXPECT_EQ(fmtRad(-1.5), "-1.5000");
}

TEST(JointError, PicksLargestJoint) {
        JointMap a{{"j1", 0.10}, {"j2", -0.30}, {"j3", 1.0}};
        double err = -1;
        std::string j;
        ASSERT_TRUE(jointError(a, {"j1", "j2"}, {0.0, 0.0}, err, j));
        EXPECT_DOUBLE_EQ(err, 0.30);
        EXPECT_EQ(j, "j2");
}

TEST(JointError, AllZeroReportsFirstJoint) {
        // regression: every error 0 used to name the last joint
        JointMap a{{"j1", 0.0}, {"j2", 0.0}};
        double err = -1;
        std::string j;
        ASSERT_TRUE(jointError(a, {"j1", "j2"}, {0.0, 0.0}, err, j));
        EXPECT_EQ(err, 0.0);
        EXPECT_EQ(j, "j1");
}

TEST(JointError, MissingJointOrSizeMismatchFails) {
        JointMap a{{"j1", 0.0}};
        double err;
        std::string j;
        EXPECT_FALSE(jointError(a, {"j1", "j2"}, {0.0, 0.0}, err, j));
        EXPECT_FALSE(jointError(a, {"j1"}, {0.0, 0.0}, err, j));
        EXPECT_FALSE(jointError(a, {}, {}, err, j));
}

TEST(PathDeviation, OnPathIsZero) {
        auto t = traj({"a", "b"}, {{0, 0}, {1, 1}, {2, 0}});
        double dev;
        std::string j;
        ASSERT_TRUE(pathDeviation({{"a", 0.5}, {"b", 0.5}}, t, dev, j));
        EXPECT_NEAR(dev, 0.0, 1e-12);
        ASSERT_TRUE(pathDeviation({{"a", 2.0}, {"b", 0.0}}, t, dev, j));
        EXPECT_NEAR(dev, 0.0, 1e-12);
}

TEST(PathDeviation, TimeIndependentLagIsNotDeviation) {
        // robot still at the start while the plan is "half way": no deviation
        auto t = traj({"a"}, {{0}, {1}, {2}});
        double dev;
        std::string j;
        ASSERT_TRUE(pathDeviation({{"a", 0.0}}, t, dev, j));
        EXPECT_NEAR(dev, 0.0, 1e-12);
}

TEST(PathDeviation, OffsetJointIsReported) {
        auto t = traj({"a", "b"}, {{0, 0}, {1, 0}});
        double dev;
        std::string j;
        ASSERT_TRUE(pathDeviation({{"a", 0.5}, {"b", 0.3}}, t, dev, j));
        EXPECT_NEAR(dev, 0.3, 1e-12);
        EXPECT_EQ(j, "b");
}

TEST(PathDeviation, BeyondTheEndMeasuresToLastPoint) {
        auto t = traj({"a"}, {{0}, {1}});
        double dev;
        std::string j;
        ASSERT_TRUE(pathDeviation({{"a", 1.4}}, t, dev, j));
        EXPECT_NEAR(dev, 0.4, 1e-12);
}

TEST(PathDeviation, SinglePointAndBadInput) {
        double dev;
        std::string j;
        ASSERT_TRUE(pathDeviation({{"a", 0.2}}, traj({"a"}, {{0}}), dev, j));
        EXPECT_NEAR(dev, 0.2, 1e-12);
        EXPECT_FALSE(pathDeviation({{"x", 0.0}}, traj({"a"}, {{0}}), dev, j));
        EXPECT_FALSE(pathDeviation({{"a", 0.0}}, traj({"a"}, {}), dev, j));
        EXPECT_FALSE(pathDeviation({{"a", 0.0}}, traj({"a"}, {{0, 1}}), dev, j));
}

TEST(JointPathContinuous, DisabledAndSmoothPass) {
        std::string r;
        auto t = traj({"a"}, {{0}, {3.0}});
        EXPECT_TRUE(jointPathContinuous(t, 0.0, r));
        EXPECT_TRUE(jointPathContinuous(traj({"a"}, {{0}, {0.1}, {0.2}}), 0.5, r));
}

TEST(JointPathContinuous, BranchFlipDetected) {
        std::string r;
        EXPECT_FALSE(jointPathContinuous(traj({"a", "b"}, {{0, 0}, {0, 1.5}}), 0.5, r));
        EXPECT_NE(r.find("joint b"), std::string::npos);
        EXPECT_NE(r.find("IK branch flip"), std::string::npos);
        EXPECT_NE(r.find("points 0 and 1"), std::string::npos);
}

TEST(JointPathContinuous, FullTurnWrapNamed) {
        std::string r;
        EXPECT_FALSE(jointPathContinuous(traj({"a"}, {{0}, {2.0 * M_PI}}), 0.5, r));
        EXPECT_NE(r.find("2*pi wrap"), std::string::npos);
}

TEST(ExecErrorCode, Mapping) {
        EXPECT_EQ(execErrorCode(true, "anything"), "SUCCESS");
        EXPECT_EQ(execErrorCode(false, "GOAL_NOT_REACHED: final joint error"), "GOAL_NOT_REACHED");
        EXPECT_EQ(execErrorCode(false, "STOPPED: PATH_DEVIATION 0.3 rad (ABORTED)"),
    "PATH_DEVIATION");
        EXPECT_EQ(execErrorCode(false, "STOPPED: action goal canceled (CANCELED)"), "CANCELED");
        EXPECT_EQ(execErrorCode(false, "ABORTED (CONTROL_FAILED)"), "EXECUTION_FAILED");
        // GOAL_NOT_REACHED only as prefix
        EXPECT_EQ(execErrorCode(false, "x GOAL_NOT_REACHED"), "EXECUTION_FAILED");
}

TEST(State, ParseAndName) {
        for (auto s : {ArmState::IDLE, ArmState::JOINT_TRAJ_CTL, ArmState::CART_TRAJ_CTL,
      ArmState::SERVO_CTL})
  {
    auto p = parseState(stateName(s));
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(*p, s);
        }
        EXPECT_FALSE(parseState("idle").has_value());
        EXPECT_FALSE(parseState("").has_value());
        EXPECT_STREQ(stateName(static_cast<ArmState>(7)), "UNKNOWN");
}

TEST(State, ChangeDecision) {
        using S = ArmState;
        EXPECT_EQ(decideStateChange(S::IDLE, S::IDLE, true, true), StateChange::SAME_STATE);
        EXPECT_EQ(decideStateChange(S::IDLE, S::SERVO_CTL, false, false), StateChange::SWITCH);
        EXPECT_EQ(decideStateChange(S::JOINT_TRAJ_CTL, S::IDLE, true, true),
                  StateChange::STOP_AND_SWITCH);
        EXPECT_EQ(decideStateChange(S::JOINT_TRAJ_CTL, S::IDLE, true, false),
                  StateChange::REJECT_MOVING);
        // same state while moving never stops the motion
        EXPECT_EQ(decideStateChange(S::CART_TRAJ_CTL, S::CART_TRAJ_CTL, true, false),
                  StateChange::SAME_STATE);
}

TEST(Watchdog, GoalResponseAndResultTimeout) {
        EXPECT_EQ(execWatchdog(false, 4.9, 1.0), Watchdog::OK);
        EXPECT_EQ(execWatchdog(false, 5.1, 100.0), Watchdog::NO_GOAL_RESPONSE);
        EXPECT_EQ(execWatchdog(true, 13.9, 2.0), Watchdog::OK);
        EXPECT_EQ(execWatchdog(true, 14.1, 2.0), Watchdog::RESULT_TIMEOUT);
        EXPECT_EQ(execWatchdog(true, 9.9, 0.0), Watchdog::OK);
}

TEST(Progress, ClampedAndZeroDuration) {
        auto p = execProgress(1.0, 4.0);
        EXPECT_DOUBLE_EQ(p.progress, 0.25);
        EXPECT_DOUBLE_EQ(p.remaining, 3.0);
        p = execProgress(5.0, 4.0);
        EXPECT_DOUBLE_EQ(p.progress, 1.0);
        EXPECT_DOUBLE_EQ(p.remaining, 0.0);
        p = execProgress(1.0, 0.0);
        EXPECT_DOUBLE_EQ(p.progress, 0.0);
}

TEST(GoalCheck, PassWaitFail) {
        EXPECT_EQ(goalCheck(true, 0.01, 0.02, false), GoalCheck::PASS);
        EXPECT_EQ(goalCheck(true, 0.02, 0.02, true), GoalCheck::PASS);
        EXPECT_EQ(goalCheck(true, 0.04, 0.02, false), GoalCheck::WAIT);
        EXPECT_EQ(goalCheck(true, 0.04, 0.02, true), GoalCheck::FAIL);
        EXPECT_EQ(goalCheck(false, 0.0, 0.02, false), GoalCheck::WAIT);
        EXPECT_EQ(goalCheck(false, 0.0, 0.02, true), GoalCheck::FAIL);
}
