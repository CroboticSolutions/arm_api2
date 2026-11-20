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

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "arm_api2/task_logic.hpp"

using namespace arm_api2::logic;  // NOLINT

namespace
{
Eigen::Isometry3d pose(double x, double y, double z, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  t.linear() = q.toRotationMatrix();
  return t;
}
const std::vector<std::string> kJoints = {"j1", "j2", "j3"};
}  // namespace

TEST(RelFrame, Parse) {
  EXPECT_EQ(parseRelFrame(""), RelFrame::TCP);
  EXPECT_EQ(parseRelFrame("TCP"), RelFrame::TCP);
  EXPECT_EQ(parseRelFrame("base"), RelFrame::BASE);
  EXPECT_FALSE(parseRelFrame("flange").has_value());
}

TEST(RelativeTarget, TcpBackAlongToolZ) {
  // tool Z points down (rotated 180 deg about X): -0.1 along tool Z = +0.1 in base Z
  const auto tcp = pose(0.3, 0.0, 0.2, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI,
    Eigen::Vector3d::UnitX())));
  const auto t = relativeTarget(tcp, RelFrame::TCP, {0, 0, -0.1}, {0, 0, 0});
  EXPECT_NEAR(t.translation().z(), 0.3, 1e-9);
  EXPECT_NEAR(t.translation().x(), 0.3, 1e-9);
  EXPECT_TRUE(t.linear().isApprox(tcp.linear()));
}

TEST(RelativeTarget, BaseTranslationKeepsOrientation) {
  const auto tcp = pose(0.3, 0.1, 0.2, rpyToQuat(0.1, 0.2, 0.3));
  const auto t = relativeTarget(tcp, RelFrame::BASE, {0.05, 0, -0.02}, {0, 0, 0});
  EXPECT_NEAR(t.translation().x(), 0.35, 1e-9);
  EXPECT_NEAR(t.translation().z(), 0.18, 1e-9);
  EXPECT_TRUE(t.linear().isApprox(tcp.linear()));
}

TEST(RelativeTarget, BaseRotationAboutTcpPoint) {
  const auto tcp = pose(0.3, 0.1, 0.2, Eigen::Quaterniond::Identity());
  const auto t = relativeTarget(tcp, RelFrame::BASE, {0, 0, 0}, {0, 0, M_PI / 2});
  EXPECT_TRUE(t.translation().isApprox(tcp.translation()));
  // tool X now points along base Y
  EXPECT_NEAR((t.linear() * Eigen::Vector3d::UnitX()).y(), 1.0, 1e-9);
}

TEST(RelativeTarget, TcpRotationAboutToolAxis) {
  const auto tcp = pose(0, 0, 0, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2,
    Eigen::Vector3d::UnitY())));
  const auto t = relativeTarget(tcp, RelFrame::TCP, {0, 0, 0}, {0, 0, M_PI / 2});
  // tool Z unchanged by a yaw about tool Z
  EXPECT_TRUE((t.linear() * Eigen::Vector3d::UnitZ()).isApprox(
      tcp.linear() * Eigen::Vector3d::UnitZ()));
}

TEST(NamedPoses, ParseListAndMap) {
  const auto y = YAML::Load(
    "home: [0, 0.5, -1]\n"
    "ready: {j2: 0.3}\n"
    "bad_len: [1, 2]\n"
    "'bad name': [0, 0, 0]\n");
  std::vector<std::string> errors;
  const auto m = parseNamedPoses(y, kJoints, "config", errors);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m.at("home").positions[2], -1.0);
  EXPECT_EQ(m.at("home").joint_names, kJoints);
  EXPECT_EQ(m.at("ready").joint_names, std::vector<std::string>{"j2"});
  EXPECT_EQ(m.at("ready").source, "config");
  EXPECT_EQ(errors.size(), 2u);
}

TEST(NamedPoses, DumpRoundTrip) {
  NamedPoseMap m;
  m["park"] = NamedPose{kJoints, {0.1, -0.2, 0.3}, "saved"};
  std::vector<std::string> errors;
  const auto back = parseNamedPoses(YAML::Load(dumpNamedPoses(m)), kJoints, "saved", errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_EQ(back.count("park"), 1u);
  EXPECT_EQ(back.at("park").joint_names, kJoints);
  EXPECT_NEAR(back.at("park").positions[1], -0.2, 1e-9);
}

TEST(NamedPoses, ToGroupFillsMissingJoints) {
  NamedPose p{{"j3"}, {1.5}, "config"};
  std::vector<double> out;
  std::string err;
  ASSERT_TRUE(namedPoseToGroup(p, kJoints, {0.1, 0.2, 0.3}, out, err));
  EXPECT_EQ(out, (std::vector<double>{0.1, 0.2, 1.5}));
  NamedPose bad{{"jx"}, {1.0}, "config"};
  EXPECT_FALSE(namedPoseToGroup(bad, kJoints, {0, 0, 0}, out, err));
  EXPECT_NE(err.find("jx"), std::string::npos);
}

TEST(NamedPoses, ValidName) {
  EXPECT_TRUE(validPoseName("home_1-a"));
  EXPECT_FALSE(validPoseName(""));
  EXPECT_FALSE(validPoseName("a b"));
  EXPECT_FALSE(validPoseName("../x"));
}

TEST(WriteFileAtomic, CreatesDirAndReplaces) {
  const auto dir = std::filesystem::temp_directory_path() / ("arm_api2_tl_" +
    std::to_string(::getpid()));
  const auto path = (dir / "sub" / "poses.yaml").string();
  std::string err;
  ASSERT_TRUE(writeFileAtomic(path, "a: 1\n", err)) << err;
  ASSERT_TRUE(writeFileAtomic(path, "b: 2\n", err)) << err;
  std::ifstream f(path);
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  EXPECT_EQ(s, "b: 2\n");
  std::filesystem::remove_all(dir);
}

TEST(PlanStore, PutGetExpire) {
  PlanStore<int> store(2, 10.0);
  const auto t0 = std::chrono::steady_clock::now();
  const auto a = store.put(std::make_shared<int>(1), t0);
  const auto b = store.put(std::make_shared<int>(2), t0 + std::chrono::seconds(1));
  EXPECT_NE(a, b);
  std::string why;
  ASSERT_TRUE(store.get(a, why, t0 + std::chrono::seconds(2)));
  // capacity 2: a third plan drops the oldest
  const auto c = store.put(std::make_shared<int>(3), t0 + std::chrono::seconds(3));
  EXPECT_FALSE(store.get(a, why, t0 + std::chrono::seconds(3)));
  EXPECT_NE(why.find("unknown"), std::string::npos);
  EXPECT_TRUE(store.get(c, why, t0 + std::chrono::seconds(3)));
  // expiry
  EXPECT_FALSE(store.get(b, why, t0 + std::chrono::seconds(20)));
  EXPECT_NE(why.find("expired"), std::string::npos);
}

TEST(TaskProgress, TimeWeighted) {
  const std::vector<double> d = {2.0, 0.0, 6.0};
  EXPECT_DOUBLE_EQ(taskProgress(d, 0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(taskProgress(d, 0, 1.0), 0.25);
  EXPECT_DOUBLE_EQ(taskProgress(d, 2, 0.5), 0.625);
  EXPECT_DOUBLE_EQ(taskProgress(d, 3, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(taskProgress({0.0, 0.0}, 1, 0.0), 0.5);
}
