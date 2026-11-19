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

// Unit tests for utils (pose comparison, waypoints, orientation normalization).

#include <gtest/gtest.h>

#include <cmath>

#include "arm_api2/utils.hpp"

namespace
{
geometry_msgs::msg::PoseStamped pose(
  double x, double y, double z, double qx = 0, double qy = 0, double qz = 0, double qw = 1)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "world";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.x = qx;
  p.pose.orientation.y = qy;
  p.pose.orientation.z = qz;
  p.pose.orientation.w = qw;
  return p;
}
geometry_msgs::msg::PoseStamped yaw(double rad)
{
  return pose(0, 0, 0, 0, 0, std::sin(rad / 2), std::cos(rad / 2));
}
}  // namespace

TEST(ComparePosition, OneMillimetreThreshold) {
        EXPECT_TRUE(utils::comparePosition(pose(1, 2, 3), pose(1.0009, 2, 3)));
        EXPECT_FALSE(utils::comparePosition(pose(1, 2, 3), pose(1.0011, 2, 3)));
        EXPECT_FALSE(utils::comparePosition(pose(1, 2, 3), pose(1, 2, 3.002)));
}

TEST(CompareOrientation, PointOneDegreeThreshold) {
        const double deg = M_PI / 180.0;
        EXPECT_TRUE(utils::compareOrientation(yaw(0), yaw(0.05 * deg)));
        EXPECT_FALSE(utils::compareOrientation(yaw(0), yaw(0.2 * deg)));
}

TEST(CompareOrientation, QuaternionSignAndWrapAreSame) {
        // q and -q are the same rotation; +-pi yaw are the same rotation
        EXPECT_TRUE(utils::compareOrientation(pose(0, 0, 0, 0, 0, 0, 1),
    pose(0, 0, 0, 0, 0, 0, -1)));
        EXPECT_TRUE(utils::compareOrientation(yaw(M_PI - 1e-5), yaw(-M_PI + 1e-5)));
}

TEST(CompareOrientation, UnnormalizedAndZeroQuaternions) {
        EXPECT_TRUE(utils::compareOrientation(pose(0, 0, 0, 0, 0, 0, 2),
    pose(0, 0, 0, 0, 0, 0, 1)));
        EXPECT_TRUE(utils::compareOrientation(pose(0, 0, 0, 0, 0, 0, 0),
    pose(0, 0, 0, 0, 0, 0, 0)));
        EXPECT_FALSE(utils::compareOrientation(pose(0, 0, 0, 0, 0, 0, 0),
    pose(0, 0, 0, 0, 0, 0, 1)));
}

TEST(ComparePose, NeedsBoth) {
        EXPECT_TRUE(utils::comparePose(pose(1, 1, 1), pose(1, 1, 1)));
        EXPECT_FALSE(utils::comparePose(pose(1, 1, 1), pose(1, 1, 1.01)));
        EXPECT_FALSE(utils::comparePose(yaw(0), yaw(0.1)));
}

TEST(NormalizeOrientation, UnitQuaternionAndKeepsHeader) {
        auto p = pose(1, 2, 3, 0, 0, 0, 2);
        p.header.frame_id = "base_link";
        const auto n = utils::normalizeOrientation(p);
        EXPECT_NEAR(n.pose.orientation.w, 1.0, 1e-12);
        EXPECT_EQ(n.pose.position.x, 1.0);
        EXPECT_EQ(n.header.frame_id, "base_link");
}

TEST(CartesianWaypoints, EvenSpacingWithoutStart) {
        auto a = pose(0, 0, 0).pose;
        auto b = pose(1, 2, 3).pose;
        auto w = utils::createCartesianWaypoints(a, b, 5);
        ASSERT_FALSE(w.empty());
        EXPECT_NEAR(w.back().position.x, 1.0, 1e-9);
        EXPECT_NEAR(w.back().position.z, 3.0, 1e-9);
        for (size_t i = 1; i < w.size(); ++i) {
    EXPECT_NEAR(w[i].position.x - w[i - 1].position.x, 0.25, 1e-9);
        }
        EXPECT_EQ(utils::createCartesianWaypoints(a, b, 1).size(), 1u);
}
