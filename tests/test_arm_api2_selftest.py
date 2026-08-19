# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2024-2026 Crobotic Solutions d.o.o.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Unit tests for the non-ROS-runtime parts of arm_api2_selftest."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
from geometry_msgs.msg import Pose


SELFTEST_PATH = Path(__file__).parent / "arm_api2_selftest" / "arm_api2_selftest.py"
SPEC = importlib.util.spec_from_file_location("arm_api2_selftest", SELFTEST_PATH)
assert SPEC is not None and SPEC.loader is not None
SELFTEST = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SELFTEST
SPEC.loader.exec_module(SELFTEST)


def test_deep_merge_preserves_nested_defaults():
    merged = SELFTEST._deep_merge(
        {"motion": {"nudge": 0.02, "tolerance": 0.01}},
        {"motion": {"nudge": 0.03}},
    )
    assert merged == {"motion": {"nudge": 0.03, "tolerance": 0.01}}


def test_config_rejects_non_finite_scaling():
    config = SELFTEST.SelfTestConfig(vel_acc_max_vel=math.nan)
    with pytest.raises(ValueError, match="finite"):
        config.validate()


def test_config_rejects_tolerance_larger_than_nudge():
    config = SELFTEST.SelfTestConfig(
        cartesian_nudge_m=0.01,
        position_tolerance_m=0.01,
    )
    with pytest.raises(ValueError, match="position_tolerance_m"):
        config.validate()


def test_config_rejects_empty_planner_list():
    config = SELFTEST.SelfTestConfig.from_yaml({"planners": []})
    with pytest.raises(ValueError, match="planners"):
        config.validate()


def test_config_rejects_quoted_boolean():
    with pytest.raises(ValueError, match="skip.gripper"):
        SELFTEST.SelfTestConfig.from_yaml({"skip": {"gripper": "false"}})


def test_quaternion_angle_normalizes_inputs():
    assert SELFTEST._quaternion_angle_rad(
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.0, 1.0),
    ) == pytest.approx(0.0)


def test_quaternion_angle_rejects_zero_quaternion():
    with pytest.raises(ValueError, match="quaternion"):
        SELFTEST._quaternion_angle_rad(
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )


def test_pose_close_checks_orientation_and_position():
    current = Pose()
    current.orientation.w = 1.0
    target = Pose()
    target.orientation.w = 1.0
    target.position.x = 0.02
    assert SELFTEST._pose_close(current, target, 0.01, 0.01) is False
    assert SELFTEST._pose_close(current, target, 0.03, 0.01) is True
