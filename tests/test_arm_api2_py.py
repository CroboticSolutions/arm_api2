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
"""Unit tests for the arm_api2_py task builder and helpers (no ROS graph needed)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from arm_api2_msgs.msg import TaskStep  # noqa: E402

from arm_api2_py.client import ActionHandle, ArmResult, _from_action_result  # noqa: E402
from arm_api2_py.task import Task, pose, quat_from_rpy, relative_move  # noqa: E402


def test_quat_from_rpy_matches_known_rotations():
    x, y, z, w = quat_from_rpy(math.pi, 0.0, 0.0)
    assert abs(abs(x) - 1.0) < 1e-9 and abs(w) < 1e-9
    x, y, z, w = quat_from_rpy(0.0, 0.0, math.pi / 2)
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9


def test_pose_helper():
    p = pose(0.1, 0.2, 0.3, rpy=(0, 0, 0), frame="base_link")
    assert p.header.frame_id == "base_link"
    assert p.pose.orientation.w == pytest.approx(1.0)
    with pytest.raises(ValueError):
        pose(0, 0, 0, quat=(0, 0, 0, 1), rpy=(0, 0, 0))


def test_relative_move_validates_frame():
    r = relative_move(z=-0.1)
    assert r.frame == "tcp" and r.translation.z == pytest.approx(-0.1) and r.linear
    with pytest.raises(ValueError):
        relative_move(frame="flange")


def test_task_builder_steps_and_types():
    t = (Task()
         .move_named("ready")
         .move_pose(pose(0.3, 0, 0.2), planner="pilz_PTP")
         .move_relative(z=0.05)
         .gripper_close(max_effort=50)
         .attach("box")
         .wait(0.5)
         .set_speed(vel=0.2)
         .move_joint([0, 0.1, 0.2], names=["a", "b", "c"])
         .move_path([pose(0.3, 0, 0.1)])
         .detach())
    types = [s.type for s in t.steps]
    assert types == [TaskStep.MOVE_NAMED, TaskStep.MOVE_POSE, TaskStep.MOVE_RELATIVE,
                     TaskStep.GRIPPER_CLOSE, TaskStep.ATTACH, TaskStep.WAIT,
                     TaskStep.SET_SPEED, TaskStep.MOVE_JOINT,
                     TaskStep.MOVE_CARTESIAN_PATH, TaskStep.DETACH]
    assert t.steps[0].named_pose == "ready" and t.steps[0].name == "ready"
    assert t.steps[1].planner == "pilz_PTP"
    assert t.steps[3].gripper_max_effort == pytest.approx(50.0)
    assert t.steps[4].object_id == "box"
    assert list(t.steps[7].joint_state.name) == ["a", "b", "c"]
    assert t.steps[9].object_id == ""
    assert len(t) == 10


def test_task_builder_rejects_bad_values():
    with pytest.raises(ValueError):
        Task().gripper(1.5)
    with pytest.raises(ValueError):
        Task().set_speed(vel=2.0)
    with pytest.raises(ValueError):
        Task().wait(-1)
    with pytest.raises(ValueError):
        Task().move_joint([0, 1], names=["a"])
    with pytest.raises(RuntimeError):
        Task().run()


class _Res:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_result_mapping():
    r = _from_action_result(True, _Res(success=True, error_code="SUCCESS", message="ok",
                                       failed_step=-1, plan_id="plan_1"), "SUCCEEDED")
    assert r and r.plan_id == "plan_1"
    # old actions only have `success`
    r = _from_action_result(False, _Res(success=False), "ABORTED")
    assert not r and r.error_code == "ABORTED" and r.failed_step == -1
    assert not ArmResult(False)


class _Fut:
    def __init__(self, value=None, exc=None):
        self._v, self._e, self._cbs = value, exc, []

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        if self._e:
            raise self._e
        return self._v


def test_action_handle_rejected_goal():
    h = ActionHandle(_Fut(_Res(accepted=False)), "arm/execute_task")
    assert h.done()
    r = h.result(0.1)
    assert not r and r.error_code == "REJECTED"


def test_action_handle_success_path():
    wrapped = _Res(status=4, result=_Res(success=True, error_code="SUCCESS", message="done",
                                         failed_step=-1))
    gh = _Res(accepted=True, get_result_async=lambda: _Fut(wrapped))
    r = ActionHandle(_Fut(gh), "x").result(0.1)
    assert r and r.message == "done"
