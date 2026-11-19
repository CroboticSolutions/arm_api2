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

// Unit tests for the permanent motion log (arm_api2/motion_log.hpp).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "arm_api2/motion_log.hpp"

using arm_api2::MotionLog;
using arm_api2::MotionRecord;

TEST(MotionLogJson, EscapesControlAndQuotes) {
        EXPECT_EQ(arm_api2::jsonEscape("a\"b\\c\nd\te\x01"), "a\\\"b\\\\c\\nd\\te\\u0001");
}

TEST(MotionLogJson, UnknownValuesAreNull) {
        MotionRecord r;
        r.command = "move_to_joint";
        const std::string j = arm_api2::toJson(r);
        EXPECT_NE(j.find("\"command\":\"move_to_joint\""), std::string::npos);
        EXPECT_NE(j.find("\"plan_time_sec\":null"), std::string::npos);
        EXPECT_NE(j.find("\"planned_points\":null"), std::string::npos);
        EXPECT_NE(j.find("\"target_pose\":null"), std::string::npos);
        EXPECT_NE(j.find("\"start_joints\":null"), std::string::npos);
        EXPECT_NE(j.find("\"time\":null"), std::string::npos);
        EXPECT_EQ(j.front(), '{');
        EXPECT_EQ(j.back(), '}');
        EXPECT_EQ(j.find('\n'), std::string::npos);
}

TEST(MotionLogJson, FilledRecord) {
        MotionRecord r;
        r.success = true;
        r.executed = true;
        r.code = "SUCCESS";
        r.planner = "pilz/LIN";
        r.plan_time_sec = 0.125;
        r.planned_points = 12;
        r.planned_duration_sec = 2.5;
        r.actual_duration_sec = 2.61;
        r.final_joint_error = 0.0012;
        r.joint_names = {"j1", "j2"};
        r.start_joints = {0.0, -1.5};
        r.goal_joints = {0.5, 1.0};
        r.has_target_pose = true;
        const std::string j = arm_api2::toJson(r);
        EXPECT_NE(j.find("\"success\":true"), std::string::npos);
        EXPECT_NE(j.find("\"executed\":true"), std::string::npos);
        EXPECT_NE(j.find("\"planner\":\"pilz/LIN\""), std::string::npos);
        EXPECT_NE(j.find("\"plan_time_sec\":0.125"), std::string::npos);
        EXPECT_NE(j.find("\"planned_points\":12"), std::string::npos);
        EXPECT_NE(j.find("\"actual_duration_sec\":2.61"), std::string::npos);
        EXPECT_NE(j.find("\"joint_names\":[\"j1\",\"j2\"]"), std::string::npos);
        EXPECT_NE(j.find("\"start_joints\":[0,-1.5]"), std::string::npos);
        EXPECT_NE(j.find("\"target_pose\":[0,0,0,0,0,0,1]"), std::string::npos);
}

TEST(MotionLogJson, IsoTime) {
        const std::string t = arm_api2::isoTimeNow();
        ASSERT_EQ(t.size(), 24u); // 2026-11-13T10:20:30.123Z
        EXPECT_EQ(t[10], 'T');
        EXPECT_EQ(t.back(), 'Z');
}

TEST(MotionLogFile, AppendsOneLinePerRecordAcrossReopen) {
        const auto dir = std::filesystem::temp_directory_path() /
    ("arm_api2_motion_log_test_" + std::to_string(::getpid()));
        std::filesystem::remove_all(dir);
        std::string err;
  {
    MotionLog log;
    ASSERT_TRUE(log.open((dir / "nested").string(), "ur1", err)) << err;
    EXPECT_TRUE(log.enabled());
    MotionRecord r;
    r.command = "a";
    EXPECT_TRUE(log.write(r));
    r.command = "b";
    EXPECT_TRUE(log.write(r));
        }
        MotionLog log2;
        ASSERT_TRUE(log2.open((dir / "nested").string(), "ur1", err)) << err;
        MotionRecord r;
        r.command = "c";
        EXPECT_TRUE(log2.write(r));
        const std::string path = log2.currentPath();
        EXPECT_NE(path.find("motions_ur1_"), std::string::npos);
        EXPECT_NE(path.find(".jsonl"), std::string::npos);
        std::ifstream in(path);
        std::string line;
        int n = 0;
        std::string cmds;
        while (std::getline(in, line)) {
    ++n;
    cmds += line.substr(line.find("\"command\":\"") + 11, 1);
        }
        EXPECT_EQ(n, 3);
        EXPECT_EQ(cmds, "abc");
        std::filesystem::remove_all(dir); // test scratch only
}

TEST(MotionLogFile, DisabledWhenDirCannotBeCreated) {
        MotionLog log;
        std::string err;
        EXPECT_FALSE(log.open("/proc/arm_api2_cannot_exist", "x", err));
        EXPECT_FALSE(err.empty());
        EXPECT_FALSE(log.enabled());
        EXPECT_FALSE(log.write(MotionRecord()));
        EXPECT_TRUE(log.currentPath().empty());
}
