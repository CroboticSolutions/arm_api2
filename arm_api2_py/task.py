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
"""Task builder: a fluent list of arm_api2_msgs/TaskStep (no ROS communication here)."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence

from arm_api2_msgs.msg import RelativeMove, TaskStep
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

if TYPE_CHECKING:  # pragma: no cover
    from arm_api2_py.client import ActionHandle, Arm, ArmResult, PlanResult


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose(
    x: float,
    y: float,
    z: float,
    quat: Optional[Sequence[float]] = None,
    rpy: Optional[Sequence[float]] = None,
    frame: str = "",
) -> PoseStamped:
    """
    Build a PoseStamped from position + quaternion (x, y, z, w) or roll/pitch/yaw (rad).

    Empty frame = the planning frame of arm_api2.
    """
    if quat is not None and rpy is not None:
        raise ValueError("give quat or rpy, not both")
    q = tuple(quat) if quat is not None else quat_from_rpy(*(rpy or (0.0, 0.0, 0.0)))
    if len(q) != 4:
        raise ValueError("quat must be (x, y, z, w)")
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
    (
        p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w,
    ) = (float(v) for v in q)
    return p


def joint_state(positions: Sequence[float], names: Optional[Sequence[str]] = None) -> JointState:
    js = JointState()
    js.position = [float(v) for v in positions]
    if names:
        if len(names) != len(positions):
            raise ValueError("names and positions differ in length")
        js.name = list(names)
    return js


def relative_move(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    frame: str = "tcp",
    linear: bool = True,
) -> RelativeMove:
    if frame not in ("tcp", "base"):
        raise ValueError("frame must be 'tcp' or 'base'")
    r = RelativeMove()
    r.frame = frame
    r.translation.x, r.translation.y, r.translation.z = float(x), float(y), float(z)
    r.rotation_rpy.x, r.rotation_rpy.y, r.rotation_rpy.z = float(roll), float(pitch), float(yaw)
    r.linear = bool(linear)
    return r


class Task:
    """Fluent builder. Every method appends one step and returns the task."""

    def __init__(self, arm: Optional["Arm"] = None):
        self._arm = arm
        self.steps: list[TaskStep] = []

    def _add(self, type_: int, name: str = "", **fields) -> "Task":
        s = TaskStep()
        s.type = type_
        s.name = name
        for k, v in fields.items():
            setattr(s, k, v)
        self.steps.append(s)
        return self

    # ---- motion ----
    def move_joint(
        self, positions: Sequence[float], names: Optional[Sequence[str]] = None, *,
        planner: str = "", name: str = "",
    ) -> "Task":
        """Joint goal; without names the values are in planning-group order."""
        return self._add(TaskStep.MOVE_JOINT, name, joint_state=joint_state(positions, names),
                         planner=planner)

    def move_pose(self, target: PoseStamped, *, planner: str = "", name: str = "") -> "Task":
        """Pose goal. planner: '' (configured list), 'pilz_PTP', 'pilz_LIN', 'ompl_<id>'."""
        return self._add(TaskStep.MOVE_POSE, name, pose=target, planner=planner)

    def move_path(self, waypoints: Sequence[PoseStamped], *, name: str = "") -> "Task":
        """Straight Cartesian segments through the waypoints (the full path is required)."""
        if not waypoints:
            raise ValueError("at least one waypoint")
        return self._add(TaskStep.MOVE_CARTESIAN_PATH, name, waypoints=list(waypoints))

    def move_relative(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0,
        pitch: float = 0.0, yaw: float = 0.0, *, frame: str = "tcp", linear: bool = True,
        planner: str = "", name: str = "",
    ) -> "Task":
        """Offset from where the previous step ended (m, rad); frame 'tcp' or 'base'."""
        return self._add(TaskStep.MOVE_RELATIVE, name,
                         relative=relative_move(x, y, z, roll, pitch, yaw, frame, linear),
                         planner=planner)

    def move_named(self, pose_name: str, *, planner: str = "", name: str = "") -> "Task":
        return self._add(TaskStep.MOVE_NAMED, name or pose_name, named_pose=pose_name,
                         planner=planner)

    # ---- gripper / objects ----
    def gripper_open(self, *, name: str = "") -> "Task":
        return self._add(TaskStep.GRIPPER_OPEN, name)

    def gripper_close(self, *, max_effort: float = 0.0, name: str = "") -> "Task":
        return self._add(TaskStep.GRIPPER_CLOSE, name, gripper_max_effort=float(max_effort))

    def gripper(self, position: float, *, max_effort: float = 0.0, name: str = "") -> "Task":
        """0 = open .. 1 = closed (Robotiq-normalized)."""
        if not 0.0 <= position <= 1.0:
            raise ValueError("gripper position must be 0..1")
        return self._add(TaskStep.GRIPPER_POSITION, name, gripper_position=float(position),
                         gripper_max_effort=float(max_effort))

    def attach(
        self, object_id: str, *, link: str = "", touch_links: Sequence[str] = (), name: str = "",
    ) -> "Task":
        return self._add(TaskStep.ATTACH, name, object_id=object_id, attach_link=link,
                         touch_links=list(touch_links))

    def detach(self, object_id: str = "", *, name: str = "") -> "Task":
        """Empty object_id = everything attached."""
        return self._add(TaskStep.DETACH, name, object_id=object_id)

    # ---- flow ----
    def wait(self, seconds: float, *, name: str = "") -> "Task":
        if not 0.0 <= seconds <= 3600.0:
            raise ValueError("wait must be 0..3600 s")
        return self._add(TaskStep.WAIT, name, duration_sec=float(seconds))

    def set_speed(self, vel: float = 0.0, acc: float = 0.0, *, name: str = "") -> "Task":
        """Velocity/acceleration scaling (0..1, 0 = keep) for the following motion steps."""
        for v in (vel, acc):
            if not 0.0 <= v <= 1.0:
                raise ValueError("scaling must be 0..1")
        return self._add(TaskStep.SET_SPEED, name, vel_scaling=float(vel), acc_scaling=float(acc))

    # ---- run ----
    def _need_arm(self) -> "Arm":
        if self._arm is None:
            raise RuntimeError("Task was created without an Arm; use arm.task()")
        return self._arm

    def plan(self, timeout: Optional[float] = 120.0) -> "PlanResult":
        """Plan everything (arm/plan); the robot does not move."""
        return self._need_arm().plan(self.steps, timeout=timeout)

    def run(self, *, wait: bool = True, feedback=None,
            timeout: Optional[float] = None) -> "ArmResult | ActionHandle":
        """Plan and execute (arm/execute_task): moves only if every step planned."""
        return self._need_arm().execute_task(self.steps, wait=wait, feedback=feedback,
                                             timeout=timeout)

    def __len__(self) -> int:
        return len(self.steps)
