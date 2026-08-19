#!/usr/bin/env python3
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

"""Functional smoke test for a running arm_api2 `moveit2_iface` node.

Exercises services, topics and actions against a live node and prints a
PASS/FAIL/SKIP report. All motion checks are RELATIVE nudges computed from
the robot's current state, so no per-robot absolute poses are hard-coded.

By default only non-moving checks run (services, topics, a planonly
plan-only round trip). Pass --move to additionally command small, real
motions (joint, Cartesian, Cartesian path action, servo twist).
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters

from arm_api2_msgs.action import MoveCartesian, MoveCartesianPath, MoveJoint

try:
    # Humble doesn't expose ServoStatus; jazzy does.
    from moveit_msgs.msg import ServoStatus
except ImportError:  # pragma: no cover - environment dependent
    ServoStatus = None  # type: ignore[assignment,misc]

from arm_api2_msgs.msg import CartesianWaypoints, PlanStatus
from arm_api2_msgs.srv import (
    CheckCartesianPath,
    CheckReachability,
    ChangeState,
    SetStringParam,
    SetVelAcc,
)


class SkipTest(Exception):
    """Raised inside a test function to mark it SKIP instead of PASS/FAIL."""


@dataclass
class TestResult:
    name: str
    status: str  # "PASS" | "FAIL" | "SKIP"
    detail: str = ""


@dataclass
class SelfTestConfig:
    robot_namespace: str = ""
    skip_gripper: bool = False
    skip_servo: bool = False
    eelink_default: str = ""
    warmup_group_state: str = "test_configuration"
    arm_controller_name: str = ""
    # Deliberately gentle (matches the common config default, e.g.
    # config/ur/ur_sim.yaml's max_vel_scaling_factor/max_acc_scaling_factor:
    # 0.1 each) - arm_api2 has no "get current vel/acc" or reset-to-default
    # call, so whatever this test sets stays in effect for every subsequent
    # real-motion check in the same run. A higher test value here previously
    # left later small nudges executing at 2x the robot's own configured
    # speed, which combined with the stall-detect-then-republish pause in
    # arm/cmd/pose|traj (a multi-second stop followed by a sudden move) to
    # look like a sharp jerk rather than a smooth motion.
    vel_acc_max_vel: float = 0.1
    vel_acc_max_acc: float = 0.1
    cartesian_nudge_m: float = 0.02
    joint_nudge_rad: float = 0.05
    position_tolerance_m: float = 0.012
    orientation_tolerance_rad: float = 0.08
    joint_tolerance_rad: float = 0.03
    settle_time_sec: float = 1.0
    max_wait_sec_per_step: float = 60.0
    servo_twist_linear_z: float = 0.02
    servo_burst_duration_sec: float = 2.0
    servo_min_expected_displacement_m: float = 0.003
    planners: list[str] = field(default_factory=lambda: ["ompl_EST", "ompl_PRM", "pilz_LIN"])
    wait_for_service_sec: float = 15.0
    service_call_sec: float = 20.0
    action_result_sec: float = 30.0
    baseline_wait_sec: float = 15.0
    state_freshness_sec: float = 1.0
    topic_max_retries: int = 2

    @classmethod
    def from_yaml(cls, data: dict[str, Any]) -> "SelfTestConfig":
        def section(name: str) -> dict[str, Any]:
            value = data.get(name, {})
            if value is None:
                return {}
            if not isinstance(value, dict):
                raise ValueError(f"{name} must be a mapping")
            return value

        def boolean(
            section_data: dict[str, Any], key: str, default: bool, label: str
        ) -> bool:
            value = section_data.get(key, default)
            if not isinstance(value, bool):
                raise ValueError(f"{label} must be a YAML boolean, not {value!r}")
            return value

        skip = section("skip")
        motion = section("motion")
        servo = section("servo")
        timeouts = section("timeouts")
        planners_value = data.get("planners", ["ompl_EST", "ompl_PRM", "pilz_LIN"])
        if not isinstance(planners_value, list):
            raise ValueError("planners must be a YAML list")
        retries_value = timeouts.get("topic_max_retries", 2)
        if not isinstance(retries_value, int) or isinstance(retries_value, bool):
            raise ValueError("timeouts.topic_max_retries must be an integer")
        return cls(
            robot_namespace=str(data.get("robot_namespace", "")),
            skip_gripper=boolean(skip, "gripper", False, "skip.gripper"),
            skip_servo=boolean(skip, "servo", False, "skip.servo"),
            eelink_default=str(data.get("eelink_default", "")),
            warmup_group_state=str(data.get("warmup_group_state", "test_configuration")),
            arm_controller_name=str(data.get("arm_controller_name", "")),
            vel_acc_max_vel=float(motion.get("vel_acc_max_vel", 0.1)),
            vel_acc_max_acc=float(motion.get("vel_acc_max_acc", 0.1)),
            cartesian_nudge_m=float(motion.get("cartesian_nudge_m", 0.02)),
            joint_nudge_rad=float(motion.get("joint_nudge_rad", 0.05)),
            position_tolerance_m=float(motion.get("position_tolerance_m", 0.012)),
            orientation_tolerance_rad=float(motion.get("orientation_tolerance_rad", 0.08)),
            joint_tolerance_rad=float(motion.get("joint_tolerance_rad", 0.03)),
            settle_time_sec=float(motion.get("settle_time_sec", 1.0)),
            max_wait_sec_per_step=float(motion.get("max_wait_sec_per_step", 60.0)),
            servo_twist_linear_z=float(servo.get("twist_linear_z", 0.02)),
            servo_burst_duration_sec=float(servo.get("burst_duration_sec", 2.0)),
            servo_min_expected_displacement_m=float(
                servo.get("min_expected_displacement_m", 0.003)
            ),
            planners=list(planners_value),
            wait_for_service_sec=float(timeouts.get("wait_for_service_sec", 15.0)),
            service_call_sec=float(timeouts.get("service_call_sec", 20.0)),
            action_result_sec=float(timeouts.get("action_result_sec", 30.0)),
            baseline_wait_sec=float(timeouts.get("baseline_wait_sec", 15.0)),
            state_freshness_sec=float(timeouts.get("state_freshness_sec", 1.0)),
            topic_max_retries=retries_value,
        )

    def validate(self) -> None:
        finite_positive = {
            "motion.vel_acc_max_vel": self.vel_acc_max_vel,
            "motion.vel_acc_max_acc": self.vel_acc_max_acc,
            "motion.cartesian_nudge_m": self.cartesian_nudge_m,
            "motion.joint_nudge_rad": self.joint_nudge_rad,
            "motion.position_tolerance_m": self.position_tolerance_m,
            "motion.orientation_tolerance_rad": self.orientation_tolerance_rad,
            "motion.joint_tolerance_rad": self.joint_tolerance_rad,
            "motion.settle_time_sec": self.settle_time_sec,
            "motion.max_wait_sec_per_step": self.max_wait_sec_per_step,
            "servo.twist_linear_z": self.servo_twist_linear_z,
            "servo.burst_duration_sec": self.servo_burst_duration_sec,
            "servo.min_expected_displacement_m": self.servo_min_expected_displacement_m,
            "timeouts.wait_for_service_sec": self.wait_for_service_sec,
            "timeouts.service_call_sec": self.service_call_sec,
            "timeouts.action_result_sec": self.action_result_sec,
            "timeouts.baseline_wait_sec": self.baseline_wait_sec,
            "timeouts.state_freshness_sec": self.state_freshness_sec,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0 (got {value!r})")
        if self.vel_acc_max_vel > 1.0 or self.vel_acc_max_acc > 1.0:
            raise ValueError("motion vel/acc scaling factors must be <= 1.0")
        if self.position_tolerance_m >= self.cartesian_nudge_m:
            raise ValueError("position_tolerance_m must be smaller than cartesian_nudge_m")
        if self.joint_tolerance_rad >= self.joint_nudge_rad:
            raise ValueError("joint_tolerance_rad must be smaller than joint_nudge_rad")
        if self.topic_max_retries < 0:
            raise ValueError("timeouts.topic_max_retries must be >= 0")
        if not self.planners or not all(isinstance(p, str) and p.strip() for p in self.planners):
            raise ValueError("planners must contain at least one non-empty planner name")


def _quaternion_angle_rad(
    q1: tuple[float, float, float, float], q2: tuple[float, float, float, float]
) -> float:
    norm1 = math.sqrt(sum(v * v for v in q1))
    norm2 = math.sqrt(sum(v * v for v in q2))
    if not math.isfinite(norm1) or not math.isfinite(norm2) or norm1 < 1e-9 or norm2 < 1e-9:
        raise ValueError("pose contains an invalid zero/non-finite quaternion")
    dot = abs(
        (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
        / (norm1 * norm2)
    )
    dot = min(1.0, max(0.0, dot))
    return 2.0 * math.acos(dot)


def _pose_distance_m(a: Pose, b: Pose) -> float:
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _pose_close(current: Pose, target: Pose, pos_tol: float, ang_tol: float) -> bool:
    if _pose_distance_m(current, target) > pos_tol:
        return False
    q1 = (
        current.orientation.x,
        current.orientation.y,
        current.orientation.z,
        current.orientation.w,
    )
    q2 = (target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w)
    return _quaternion_angle_rad(q1, q2) <= ang_tol


def _copy_pose(p: Pose) -> Pose:
    out = Pose()
    out.position.x, out.position.y, out.position.z = p.position.x, p.position.y, p.position.z
    out.orientation.x = p.orientation.x
    out.orientation.y = p.orientation.y
    out.orientation.z = p.orientation.z
    out.orientation.w = p.orientation.w
    return out


class SelfTestNode(Node):
    def __init__(self, cfg: SelfTestConfig, run_motion: bool) -> None:
        super().__init__("arm_api2_selftest")
        self._cfg = cfg
        self._run_motion = run_motion
        self._results: list[TestResult] = []
        self._motion_block_reason = ""
        self._active_goal_handle = None
        self._initial_ctl_state: str | None = None

        ns = cfg.robot_namespace.strip("/")
        self._prefix = f"/{ns}" if ns else ""

        self._latest_pose: PoseStamped | None = None
        self._latest_joint_state: JointState | None = None
        self._latest_ctl_state: String | None = None
        self._latest_gripper_state: String | None = None
        self._latest_plan_status: PlanStatus | None = None
        self._pose_seq = 0
        self._joint_seq = 0
        self._ctl_state_seq = 0
        self._gripper_state_seq = 0
        self._plan_status_seq = 0
        self._pose_received_at = 0.0
        self._joint_received_at = 0.0

        self._sub_pose = self.create_subscription(
            PoseStamped, f"{self._prefix}/arm/state/current_pose", self._on_pose, 10
        )
        self._sub_joint = self.create_subscription(
            JointState, f"{self._prefix}/joint_states", self._on_joint, 10
        )
        self._sub_ctl_state = self.create_subscription(
            String, f"{self._prefix}/arm/state/ctl_state", self._on_ctl_state, 10
        )
        self._sub_gripper_state = self.create_subscription(
            String, f"{self._prefix}/arm/state/gripper_state", self._on_gripper_state, 10
        )
        plan_status_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._sub_plan_status = self.create_subscription(
            PlanStatus, f"{self._prefix}/arm/state/plan_status", self._on_plan_status,
            plan_status_qos,
        )
        # moveit2_iface's own servo halt/singularity status - HALT_FOR_SINGULARITY (2),
        # HALT_FOR_COLLISION (4) and INVALID (-1) mean the servo output is being
        # intentionally suppressed, not that servoing is broken.
        self._sub_servo_status = None
        if not self._cfg.skip_servo and ServoStatus is not None:
            self._sub_servo_status = self.create_subscription(
                ServoStatus,
                f"{self._prefix}/moveit2_iface/status",
                self._on_servo_status,
                10,
            )
        self._servo_halt_seen = False

        self._cli_change_state = self.create_client(
            ChangeState, f"{self._prefix}/arm/change_state"
        )
        self._cli_set_vel_acc = self.create_client(SetVelAcc, f"{self._prefix}/arm/set_vel_acc")
        self._cli_set_eelink = self.create_client(
            SetStringParam, f"{self._prefix}/arm/set_eelink"
        )
        self._cli_set_planonly = self.create_client(
            SetBool, f"{self._prefix}/arm/set_planonly"
        )
        self._cli_set_planner = self.create_client(
            SetStringParam, f"{self._prefix}/arm/set_planner"
        )
        self._cli_open_gripper = self.create_client(
            Trigger, f"{self._prefix}/arm/open_gripper"
        )
        self._cli_close_gripper = self.create_client(
            Trigger, f"{self._prefix}/arm/close_gripper"
        )
        self._cli_check_reachability = self.create_client(
            CheckReachability, f"{self._prefix}/arm/check_reachability"
        )
        self._cli_check_cartesian_path = self.create_client(
            CheckCartesianPath, f"{self._prefix}/arm/check_cartesian_path"
        )
        self._cli_list_controllers = self.create_client(
            ListControllers, f"{self._prefix}/controller_manager/list_controllers"
        )
        self._arm_joint_names_cache: set[str] | None = None
        # move_group (not moveit2_iface_node) is where robot_description_semantic is
        # actually populated - used to read named SRDF group_states like "test_configuration".
        self._cli_move_group_params = self.create_client(
            GetParameters, f"{self._prefix}/move_group/get_parameters"
        )

        self._ac_move_to_pose = ActionClient(
            self, MoveCartesian, f"{self._prefix}/arm/move_to_pose"
        )
        self._ac_move_to_joint = ActionClient(
            self, MoveJoint, f"{self._prefix}/arm/move_to_joint"
        )
        self._ac_move_to_pose_path = ActionClient(
            self, MoveCartesianPath, f"{self._prefix}/arm/move_to_pose_path"
        )
        self._ac_gripper = ActionClient(
            self, GripperCommand, f"{self._prefix}/arm/gripper_control"
        )

        self._pub_servo_twist = self.create_publisher(
            TwistStamped, f"{self._prefix}/moveit2_iface/servo_twist_cmd", 10
        )
        # Topic-based "simple interface" - a separate code path (pose_cmd_cb /
        # cart_poses_cb) from the move_to_pose / move_to_pose_path actions.
        self._pub_cmd_pose = self.create_publisher(
            PoseStamped, f"{self._prefix}/arm/cmd/pose", 10
        )
        self._pub_cmd_traj = self.create_publisher(
            CartesianWaypoints, f"{self._prefix}/arm/cmd/traj", 10
        )

    # --- subscriptions -----------------------------------------------------

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        self._pose_seq += 1
        self._pose_received_at = time.monotonic()

    def _on_joint(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        self._joint_seq += 1
        self._joint_received_at = time.monotonic()

    def _on_ctl_state(self, msg: String) -> None:
        self._latest_ctl_state = msg
        self._ctl_state_seq += 1

    def _on_gripper_state(self, msg: String) -> None:
        self._latest_gripper_state = msg
        self._gripper_state_seq += 1

    def _on_plan_status(self, msg: PlanStatus) -> None:
        self._latest_plan_status = msg
        self._plan_status_seq += 1

    _SERVO_HALT_CODES = (
        ()
        if ServoStatus is None
        else (
            ServoStatus.INVALID,
            ServoStatus.HALT_FOR_SINGULARITY,
            ServoStatus.HALT_FOR_COLLISION,
        )
    )

    def _on_servo_status(self, msg: ServoStatus) -> None:
        if msg.code in self._SERVO_HALT_CODES:
            self._servo_halt_seen = True

    def _spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    # --- low-level call helpers --------------------------------------------

    def _call(self, client, request, timeout: float | None = None):
        if not client.wait_for_service(timeout_sec=self._cfg.wait_for_service_sec):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._cfg.service_call_sec if timeout is None else timeout
        )
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service {client.srv_name} call timed out")
        return future.result()

    def _send_action(self, client: ActionClient, goal, timeout: float | None = None):
        if not client.wait_for_server(timeout_sec=self._cfg.wait_for_service_sec):
            raise RuntimeError(f"action server {client._action_name} not available")
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future,
            timeout_sec=self._cfg.service_call_sec if timeout is None else timeout,
        )
        if not send_future.done():
            raise RuntimeError(f"action {client._action_name} goal response timed out")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"action {client._action_name} goal rejected")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(
                self, result_future,
                timeout_sec=self._cfg.action_result_sec if timeout is None else timeout,
            )
            if not result_future.done() or result_future.result() is None:
                self._cancel_goal(goal_handle, client._action_name)
                raise RuntimeError(
                    f"action {client._action_name} result timed out; goal cancellation requested"
                )
            wrapped_result = result_future.result()
            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(
                    f"action {client._action_name} finished with status "
                    f"{wrapped_result.status}, expected SUCCEEDED"
                )
            return wrapped_result.result
        except BaseException:
            if not result_future.done():
                self._cancel_goal(goal_handle, client._action_name)
            raise
        finally:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None

    def _expect_action_rejected(self, client: ActionClient, goal) -> None:
        if not client.wait_for_server(timeout_sec=self._cfg.wait_for_service_sec):
            raise RuntimeError(f"action server {client._action_name} not available")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._cfg.service_call_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"action {client._action_name} goal response timed out")
        goal_handle = future.result()
        if goal_handle.accepted:
            self._cancel_goal(goal_handle, client._action_name)
            raise RuntimeError(f"action {client._action_name} accepted an invalid goal")

    def _cancel_goal(self, goal_handle, action_name: str) -> None:
        try:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            if not cancel_future.done() or cancel_future.result() is None:
                self.get_logger().error(f"Cancellation of {action_name} was not acknowledged")
        except Exception as e:  # noqa: BLE001 - cleanup must not mask the original failure
            self.get_logger().error(f"Failed to cancel {action_name}: {e}")

    def cancel_active_goal(self) -> None:
        if self._active_goal_handle is not None:
            self._cancel_goal(self._active_goal_handle, "active selftest action")
            self._active_goal_handle = None

    def _change_state(self, state: str) -> None:
        seq_before = self._ctl_state_seq
        resp = self._call(self._cli_change_state, ChangeState.Request(state=state))
        if not resp.success:
            raise RuntimeError(f"change_state({state}) returned success=false")
        end = time.monotonic() + self._cfg.service_call_sec
        while time.monotonic() < end:
            self._spin(0.05)
            if (
                self._ctl_state_seq > seq_before
                and self._latest_ctl_state is not None
                and self._latest_ctl_state.data == state
            ):
                return
        actual = self._latest_ctl_state.data if self._latest_ctl_state else "<none>"
        raise RuntimeError(f"ctl_state did not change to {state}; last reported state is {actual}")

    def _wait_for_pose(self, target: Pose, timeout: float) -> None:
        stable = 0
        last_seq = self._pose_seq
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._spin(0.05)
            if self._latest_pose is None or self._pose_seq == last_seq:
                continue
            last_seq = self._pose_seq
            if _pose_close(
                self._latest_pose.pose, target, self._cfg.position_tolerance_m,
                self._cfg.orientation_tolerance_rad,
            ):
                stable += 1
                if stable >= 5:
                    return
            else:
                stable = 0
        raise RuntimeError(f"pose did not converge within {timeout:.1f}s")

    def _wait_for_joints(self, target: JointState, timeout: float) -> None:
        stable = 0
        last_seq = self._joint_seq
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._spin(0.05)
            if self._latest_joint_state is None or self._joint_seq == last_seq:
                continue
            last_seq = self._joint_seq
            close = all(
                abs(self._joint_position_by_name(self._latest_joint_state, name) - wanted)
                <= self._cfg.joint_tolerance_rad
                for name, wanted in zip(target.name, target.position)
            )
            stable = stable + 1 if close else 0
            if stable >= 5:
                return
        raise RuntimeError(f"joints did not converge within {timeout:.1f}s")

    def _wait_for_next_joint_state(self, seq_before: int, timeout: float) -> JointState:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._spin(0.05)
            if self._joint_seq > seq_before and self._latest_joint_state is not None:
                return self._latest_joint_state
        raise RuntimeError("no fresh joint_states message received")

    def _publish_and_wait_for_pose(
        self, publish_fn: Callable[[], None], target: Pose, timeout: float,
        stall_check_interval: float = 3.0,
    ) -> None:
        """Publish once via publish_fn(), then poll for convergence like
        _wait_for_pose - but if the arm isn't making net PROGRESS toward
        `target` (distance-to-target shrinking), republish instead of waiting
        out the full timeout in silence.

        This tracks distance-to-target rather than raw movement between
        samples: a raw "did it move at all" check republishes even while a
        legitimate, still-in-flight move is under way (e.g. a multi-waypoint
        path detours through an intermediate waypoint before reaching
        `target`) - restarting a move that was about to succeed, forever.
        Comparing distance-to-target across the whole check window survives a
        detour as long as the arm ends up net closer than it started.

        Single-shot topic commands (arm/cmd/pose, arm/cmd/traj) get silently
        dropped ("Trajectory still executing; ignoring...") if they arrive in
        the few ms before arm_api2 considers the previous move fully finished
        - with no rejection fed back to the client, so lack of progress is the
        only signal available that a republish is needed."""
        publish_count = 0
        status_seq_at_publish = self._plan_status_seq
        last_status_seq = self._plan_status_seq
        latest_error = ""

        def publish_command() -> None:
            nonlocal publish_count, status_seq_at_publish
            publish_fn()
            publish_count += 1
            status_seq_at_publish = self._plan_status_seq

        publish_command()
        last_check_time = time.monotonic()
        last_check_distance = (
            _pose_distance_m(self._latest_pose.pose, target) if self._latest_pose else None
        )
        # Ignore a distance change smaller than this as noise, not progress.
        progress_epsilon_m = self._cfg.position_tolerance_m / 4
        stable = 0
        last_pose_seq = self._pose_seq
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._spin(0.05)
            if self._plan_status_seq > last_status_seq and self._latest_plan_status is not None:
                status = self._latest_plan_status
                last_status_seq = self._plan_status_seq
                if (
                    last_status_seq > status_seq_at_publish
                    and _pose_close(
                        status.requested_pose.pose,
                        target,
                        self._cfg.position_tolerance_m,
                        self._cfg.orientation_tolerance_rad,
                    )
                ):
                    if not status.success:
                        latest_error = f"{status.error_code}: {status.reason}"
                        if status.error_code not in {"BUSY_TRAJECTORY_EXECUTING", "PLAN_NOT_FOUND"}:
                            raise RuntimeError(f"topic command failed: {latest_error}")

            if self._latest_pose is not None and self._pose_seq != last_pose_seq:
                last_pose_seq = self._pose_seq
                cur = self._latest_pose.pose
                if _pose_close(
                    cur, target, self._cfg.position_tolerance_m,
                    self._cfg.orientation_tolerance_rad,
                ):
                    stable += 1
                    if stable >= 5:
                        return
                else:
                    stable = 0

            if time.monotonic() - last_check_time >= stall_check_interval:
                if self._latest_pose is None:
                    continue
                cur = self._latest_pose.pose
                cur_distance = _pose_distance_m(cur, target)
                if (
                    last_check_distance is not None
                    and last_check_distance - cur_distance < progress_epsilon_m
                ):
                    retries_used = publish_count - 1
                    if retries_used >= self._cfg.topic_max_retries:
                        suffix = f"; last plan status: {latest_error}" if latest_error else ""
                        raise RuntimeError(
                            f"topic command made no progress after {publish_count} publish(es)"
                            f"{suffix}"
                        )
                    self.get_logger().info(
                        f"No net progress toward target in the last "
                        f"{stall_check_interval:.0f}s (still {cur_distance:.4f}m away) - "
                        f"republishing ({retries_used + 1}/{self._cfg.topic_max_retries})."
                    )
                    publish_command()
                last_check_distance = cur_distance
                last_check_time = time.monotonic()
        suffix = f"; last plan status: {latest_error}" if latest_error else ""
        raise RuntimeError(f"pose did not converge within {timeout:.1f}s{suffix}")

    def _discover_arm_joint_names(self) -> set[str] | None:
        """Ask controller_manager which joints the arm's own trajectory controller
        claims, so a gripper joint published on the same /joint_states topic
        (e.g. a Robotiq finger joint) doesn't end up in a MoveJoint goal."""
        if self._arm_joint_names_cache is not None:
            return self._arm_joint_names_cache
        if not self._cli_list_controllers.wait_for_service(timeout_sec=3.0):
            return None
        future = self._cli_list_controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            return None
        best = None
        for c in result.controller:
            if c.state != "active" or not c.claimed_interfaces:
                continue
            if self._cfg.arm_controller_name:
                if c.name == self._cfg.arm_controller_name:
                    best = c
                    break
                continue
            type_lower = c.type.lower()
            if "gripper" in type_lower or "broadcaster" in type_lower:
                continue
            if best is None or len(c.claimed_interfaces) > len(best.claimed_interfaces):
                best = c
        if best is None:
            if self._cfg.arm_controller_name:
                self.get_logger().warn(
                    f"Configured arm controller '{self._cfg.arm_controller_name}' is not active"
                )
            return None
        self._arm_joint_names_cache = {iface.split("/")[0] for iface in best.claimed_interfaces}
        return self._arm_joint_names_cache

    def _arm_only_joint_state(self) -> JointState:
        """Latest /joint_states filtered down to the arm's own joints (see
        _discover_arm_joint_names). Falls back to the raw message, unfiltered,
        if discovery isn't possible - e.g. no controller_manager under this
        namespace."""
        baseline = self._latest_joint_state
        if baseline is None:
            raise RuntimeError("no joint state available")
        arm_names = self._discover_arm_joint_names()
        if not arm_names:
            self.get_logger().warn(
                "Could not discover arm-only joint names via controller_manager/"
                "list_controllers; using the full joint_states message as-is "
                "(this will misbehave if it includes non-arm joints, e.g. a gripper)."
            )
            return baseline
        names, positions = [], []
        for name, pos in zip(baseline.name, baseline.position):
            if name in arm_names:
                names.append(name)
                positions.append(pos)
        if not names:
            self.get_logger().warn(
                "No joint_states entries matched the discovered arm joint names; "
                "using the full joint_states message as-is."
            )
            return baseline
        return JointState(name=names, position=positions)

    @staticmethod
    def _joint_position_by_name(joint_state: JointState, name: str) -> float:
        """Look up a joint's position BY NAME. /joint_states ordering is not
        guaranteed to match any particular goal array's ordering (e.g. once a
        gripper joint is filtered out for the goal but still present, at a
        different index, in the raw message) - indexing blindly (e.g.
        position[-1]) can silently read a different joint's value than the one
        actually commanded, which would validate nothing and could mask a real
        stuck/non-moving arm."""
        try:
            idx = list(joint_state.name).index(name)
        except ValueError as e:
            raise RuntimeError(f"joint '{name}' not found in latest joint_states") from e
        return joint_state.position[idx]

    # --- test bookkeeping ----------------------------------------------------

    def _run(self, name: str, fn: Callable[[], None]) -> TestResult:
        try:
            fn()
        except SkipTest as e:
            result = TestResult(name, "SKIP", str(e))
            self._results.append(result)
            self.get_logger().info(f"[SKIP] {name} - {e}")
            return result
        except Exception as e:  # noqa: BLE001 - deliberately broad, this is a test runner
            self.cancel_active_goal()
            result = TestResult(name, "FAIL", str(e))
            self._results.append(result)
            self.get_logger().error(f"[FAIL] {name} - {e}")
            return result
        result = TestResult(name, "PASS")
        self._results.append(result)
        self.get_logger().info(f"[PASS] {name}")
        return result

    def _require_motion(self) -> None:
        if not self._run_motion:
            raise SkipTest("real motion tests disabled; pass --move to enable")
        if self._motion_block_reason:
            raise SkipTest(f"motion tests blocked: {self._motion_block_reason}")

    # --- individual tests ----------------------------------------------------

    def test_baseline(self) -> None:
        end = time.monotonic() + self._cfg.baseline_wait_sec
        while time.monotonic() < end and (
            self._latest_pose is None or self._latest_joint_state is None
        ):
            self._spin(0.1)
        if self._latest_pose is None:
            raise RuntimeError("no message received on arm/state/current_pose")
        if self._latest_joint_state is None:
            raise RuntimeError("no message received on joint_states")
        now = time.monotonic()
        if now - self._pose_received_at > self._cfg.state_freshness_sec:
            raise RuntimeError("arm/state/current_pose is stale")
        if now - self._joint_received_at > self._cfg.state_freshness_sec:
            raise RuntimeError("joint_states is stale")
        pose = self._latest_pose
        if not pose.header.frame_id:
            raise RuntimeError("current_pose has an empty frame_id")
        pose_values = (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
        )
        if not all(math.isfinite(v) for v in pose_values):
            raise RuntimeError("current_pose contains non-finite values")
        _quaternion_angle_rad(
            (
                pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w,
            ),
            (0.0, 0.0, 0.0, 1.0),
        )
        joint = self._latest_joint_state
        if not joint.name or len(joint.name) != len(joint.position):
            raise RuntimeError("joint_states name/position arrays are empty or have different sizes")
        if len(set(joint.name)) != len(joint.name):
            raise RuntimeError("joint_states contains duplicate joint names")
        if not all(math.isfinite(v) for v in joint.position):
            raise RuntimeError("joint_states contains non-finite positions")

    def test_initial_warmup(self) -> None:
        """Move to the configured SRDF named state (default "test_configuration")
        before any other physical-motion check, so motion starts from a known-good,
        non-singular pose - rather than reactively probing/recovering later.
        Two things make this necessary as a first step, not just a fallback:
        the robot's default spawn pose can itself be a genuine kinematic
        singularity (observed repeatedly this session), and the simulation is
        NOT reset between separate selftest invocations, so a fresh run starts
        wherever the PREVIOUS run's real-motion tests happened to leave the
        arm - an arbitrary, not-necessarily-safe pose either way. The reactive
        probing in _probe_cartesian_direction/_bent_waypoints stays in place
        as a safety net for later in the run, since a chain of small real
        moves can still drift back into a locally awkward configuration even
        after starting clean here."""
        if not self._move_to_named_state(self._cfg.warmup_group_state):
            raise RuntimeError(
                f"could not move to SRDF state '{self._cfg.warmup_group_state}' - check "
                "that this name exists in the robot's SRDF group_states, or set "
                "warmup_group_state to a valid one (or \"\" to skip this step)"
            )

    def test_ctl_state_topic(self) -> None:
        end = time.monotonic() + 5.0
        while time.monotonic() < end and self._latest_ctl_state is None:
            self._spin(0.1)
        if self._latest_ctl_state is None:
            raise RuntimeError("no message received on arm/state/ctl_state")
        if self._latest_ctl_state.data not in {
            "IDLE", "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL",
        }:
            raise RuntimeError(f"unexpected ctl_state value: {self._latest_ctl_state.data!r}")
        self._initial_ctl_state = self._latest_ctl_state.data

    def test_invalid_change_state(self) -> None:
        before = self._latest_ctl_state.data if self._latest_ctl_state else None
        resp = self._call(
            self._cli_change_state,
            ChangeState.Request(state="__ARM_API2_SELFTEST_INVALID_STATE__"),
        )
        if resp.success:
            raise RuntimeError("change_state accepted an invalid state")
        self._spin(0.2)
        after = self._latest_ctl_state.data if self._latest_ctl_state else None
        if before is not None and after != before:
            raise RuntimeError(f"invalid change_state changed ctl_state from {before} to {after}")

    def test_action_state_rejections(self) -> None:
        if self._latest_pose is None or self._latest_joint_state is None:
            raise RuntimeError("baseline state unavailable")
        original = self._latest_ctl_state.data if self._latest_ctl_state else "IDLE"
        arm_joint_state = self._arm_only_joint_state()
        try:
            self._change_state("CART_TRAJ_CTL")
            self._expect_action_rejected(
                self._ac_move_to_joint,
                MoveJoint.Goal(joint_state=arm_joint_state),
            )
            self._expect_action_rejected(
                self._ac_move_to_pose_path,
                MoveCartesianPath.Goal(poses=[self._latest_pose]),
            )
            self._change_state("JOINT_TRAJ_CTL")
            self._expect_action_rejected(
                self._ac_move_to_pose,
                MoveCartesian.Goal(goal=self._latest_pose),
            )
        finally:
            self._change_state(original)

    def test_set_vel_acc(self) -> None:
        # Deliberately a gentle value (see SelfTestConfig.vel_acc_max_vel) - there's
        # no "get current vel/acc" or reset-to-default call, so whatever this sets
        # stays in effect for every real-motion check later in the same run.
        accepted_invalid = []
        for max_vel, max_acc in ((-0.1, 0.1), (math.nan, 0.1)):
            invalid = self._call(
                self._cli_set_vel_acc,
                SetVelAcc.Request(max_vel=max_vel, max_acc=max_acc),
            )
            if invalid.success:
                accepted_invalid.append((max_vel, max_acc))
        # Always restore a finite known-safe value before reporting an invalid-input bug.
        resp = self._call(
            self._cli_set_vel_acc,
            SetVelAcc.Request(max_vel=self._cfg.vel_acc_max_vel, max_acc=self._cfg.vel_acc_max_acc),
        )
        if not resp.success:
            raise RuntimeError("set_vel_acc returned success=false")
        if accepted_invalid:
            raise RuntimeError(f"set_vel_acc accepted invalid values: {accepted_invalid!r}")

    def test_set_eelink(self) -> None:
        if not self._cfg.eelink_default:
            raise SkipTest(
                "eelink_default not set in config; set it to the robot's ee_link_name to enable"
            )
        invalid = self._call(
            self._cli_set_eelink,
            SetStringParam.Request(value="__arm_api2_selftest_invalid_link__"),
        )
        # Restore the configured valid link even when an older/broken server accepted
        # the invalid request, so the negative test cannot poison later motion checks.
        resp = self._call(
            self._cli_set_eelink, SetStringParam.Request(value=self._cfg.eelink_default)
        )
        if not resp.success:
            raise RuntimeError("set_eelink returned success=false")
        if invalid.success:
            raise RuntimeError("set_eelink accepted a link that is not in the robot model")

    def test_check_reachability(self) -> None:
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        resp = self._call(
            self._cli_check_reachability,
            CheckReachability.Request(pose=self._latest_pose, ik_attempts=0, ik_timeout_sec=0.0),
        )
        if not resp.reachable:
            raise RuntimeError(f"current pose reported unreachable: {resp.reason}")

    def test_check_cartesian_path(self) -> None:
        """Exercises check_cartesian_path via the same 6-axis probe the Cartesian
        motion tests use (falling back to the SRDF warm-up state under --move) so a
        robot that happens to spawn at/near a singularity doesn't produce a false
        failure here - this test still genuinely fails if check_cartesian_path
        can't validate ANY direction (with a warm-up if --move allows one), which
        is a real problem worth surfacing rather than routing around."""
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        self._probe_cartesian_direction()

    def test_gripper_services(self) -> None:
        self._require_motion()
        if self._cfg.skip_gripper:
            raise SkipTest("gripper checks disabled in config (skip.gripper)")
        seq = self._gripper_state_seq
        resp = self._call(self._cli_open_gripper, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"open_gripper failed: {resp.message}")
        self._wait_for_gripper_state("open", seq)
        seq = self._gripper_state_seq
        resp = self._call(self._cli_close_gripper, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"close_gripper failed: {resp.message}")
        self._wait_for_gripper_state("closed", seq)

        action_goal = GripperCommand.Goal()
        action_goal.command.position = 0.0
        self._send_action(self._ac_gripper, action_goal, timeout=self._cfg.max_wait_sec_per_step)

        # Leave the gripper in the same state as the service sequence did.
        seq = self._gripper_state_seq
        resp = self._call(self._cli_close_gripper, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"final close_gripper failed: {resp.message}")
        self._wait_for_gripper_state("closed", seq)

    def _wait_for_gripper_state(self, expected: str, seq_before: int) -> None:
        end = time.monotonic() + self._cfg.service_call_sec
        while time.monotonic() < end:
            self._spin(0.05)
            if self._gripper_state_seq > seq_before and self._latest_gripper_state is not None:
                if self._latest_gripper_state.data != expected:
                    raise RuntimeError(
                        f"expected gripper_state={expected!r}, got "
                        f"{self._latest_gripper_state.data!r}"
                    )
                return
        raise RuntimeError(f"no fresh gripper_state={expected!r} message received")

    def test_planner_and_planonly(self) -> None:
        if self._latest_joint_state is None:
            raise RuntimeError("no baseline joint state available")
        enable = self._call(self._cli_set_planonly, SetBool.Request(data=True))
        if not enable.success:
            raise RuntimeError("set_planonly(true) returned success=false")
        try:
            self._change_state("JOINT_TRAJ_CTL")
            invalid_planner = self._call(
                self._cli_set_planner,
                SetStringParam.Request(value="__invalid_planner__"),
            )
            if invalid_planner.success:
                raise RuntimeError("set_planner accepted an invalid planner prefix")
            baseline_joint = self._arm_only_joint_state()
            if not baseline_joint.name:
                raise RuntimeError("baseline joint state has no joints to plan for")
            goal_joint = JointState()
            goal_joint.name = list(baseline_joint.name)
            goal_joint.position = list(baseline_joint.position)
            goal_joint.position[-1] += self._cfg.joint_nudge_rad
            baseline_positions = dict(zip(baseline_joint.name, baseline_joint.position))

            failures = []
            for planner in self._cfg.planners:
                resp = self._call(
                    self._cli_set_planner, SetStringParam.Request(value=planner)
                )
                if not resp.success:
                    failures.append(f"{planner}: set_planner returned success=false")
                    continue
                self._latest_plan_status = None
                joint_seq_before = self._joint_seq
                result = self._send_action(
                    self._ac_move_to_joint,
                    MoveJoint.Goal(joint_state=goal_joint),
                    timeout=self._cfg.max_wait_sec_per_step,
                )
                if not result.success:
                    failures.append(f"{planner}: move_to_joint (planonly) returned success=false")
                    continue
                # The whole point of set_planonly(true) is that nothing actually
                # executes - verify the arm really did stay put, not just that the
                # action reported success.
                try:
                    latest_joint_state = self._wait_for_next_joint_state(
                        joint_seq_before, self._cfg.settle_time_sec
                    )
                except RuntimeError as e:
                    failures.append(f"{planner}: {e}")
                    continue
                moved = []
                for joint_name, baseline_position in baseline_positions.items():
                    actual = self._joint_position_by_name(latest_joint_state, joint_name)
                    if abs(actual - baseline_position) > self._cfg.joint_tolerance_rad:
                        moved.append(f"{joint_name}: {baseline_position:.4f}->{actual:.4f}")
                if moved:
                    failures.append(
                        f"{planner}: set_planonly(true) but joints moved ({', '.join(moved)})"
                    )
            if failures:
                raise RuntimeError("; ".join(failures))
        finally:
            disable = self._call(self._cli_set_planonly, SetBool.Request(data=False))
            if not disable.success:
                raise RuntimeError("set_planonly(false) returned success=false")

    def test_joint_motion(self) -> None:
        self._require_motion()
        if self._latest_joint_state is None:
            raise RuntimeError("no baseline joint state available")
        self._change_state("JOINT_TRAJ_CTL")
        arm_joint_state = self._arm_only_joint_state()
        baseline = list(arm_joint_state.position)
        names = list(arm_joint_state.name)
        if not baseline:
            raise RuntimeError("baseline joint state has no positions")

        joint_index, delta = self._select_joint_nudge(names, baseline)
        target_joint_name = names[joint_index]
        position_before_move = baseline[joint_index]
        out_positions = list(baseline)
        out_positions[joint_index] += delta

        # Move out once, then return exactly to the captured baseline.
        for goal_positions in (out_positions, baseline):
            goal = JointState(name=names, position=goal_positions)
            result = self._send_action(
                self._ac_move_to_joint,
                MoveJoint.Goal(joint_state=goal),
                timeout=self._cfg.max_wait_sec_per_step,
            )
            if not result.success:
                raise RuntimeError("move_to_joint returned success=false")
            self._wait_for_joints(goal, self._cfg.max_wait_sec_per_step)
            actual = self._joint_position_by_name(self._latest_joint_state, target_joint_name)
            # Compare against where the joint was BEFORE this specific move (not the
            # overall baseline) - the second leg of this round trip moves back
            # towards baseline, so it would look like "no movement" against that.
            expected_travel = abs(goal_positions[joint_index] - position_before_move)
            actual_travel = abs(actual - position_before_move)
            if expected_travel > self._cfg.joint_tolerance_rad and actual_travel < expected_travel / 2:
                raise RuntimeError(
                    f"'{target_joint_name}' reported success but barely moved "
                    f"(was {position_before_move:.4f}, expected to travel "
                    f"{expected_travel:.4f} rad, only moved {actual_travel:.4f} rad)"
                )
            if abs(actual - goal_positions[joint_index]) > self._cfg.joint_tolerance_rad:
                raise RuntimeError(
                    f"'{target_joint_name}' did not converge: "
                    f"expected {goal_positions[joint_index]:.4f}, "
                    f"got {actual:.4f}"
                )
            position_before_move = actual

    def _select_joint_nudge(
        self, names: list[str], positions: list[float]
    ) -> tuple[int, float]:
        limits = self._fetch_joint_limits()
        margin = self._cfg.joint_tolerance_rad
        for index in reversed(range(len(names))):
            lower, upper = limits.get(names[index], (-math.inf, math.inf))
            for delta in (self._cfg.joint_nudge_rad, -self._cfg.joint_nudge_rad):
                target = positions[index] + delta
                if target >= lower + margin and target <= upper - margin:
                    return index, delta
        raise RuntimeError("no arm joint has enough limit margin for the configured joint nudge")

    _CANDIDATE_AXES = (
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
    )

    def _bent_waypoints(
        self, header, baseline: Pose, dx: float, dy: float, dz: float
    ) -> tuple[Pose, Pose]:
        """Two waypoints along a validated direction, then a bend onto a
        non-parallel axis - NOT a straight out-and-back to baseline. MoveIt's
        time-optimal trajectory generation cannot parameterize a path that
        reverses ~180 degrees at a waypoint ("The path requires a 180 deg.
        turn..."), which a pure there-and-back path always does at its
        turnaround point.

        Unlike the primary direction (dx, dy, dz), which _probe_cartesian_direction
        already validated via check_cartesian_path, a bend axis picked purely for
        being "not parallel" is untested - if it turns out infeasible from via1
        (e.g. a local manipulability issue, same class of problem the primary
        probe exists to avoid), computeCartesianPath silently truncates at via1
        every time, and no amount of republishing the same waypoints ever
        reaches via2. So probe candidate bend axes here too, the same way."""
        via1 = _copy_pose(baseline)
        via1.position.x += dx * self._cfg.cartesian_nudge_m
        via1.position.y += dy * self._cfg.cartesian_nudge_m
        via1.position.z += dz * self._cfg.cartesian_nudge_m
        via1_stamped = PoseStamped(header=header, pose=via1)

        tried = []
        for bx, by, bz in self._CANDIDATE_AXES:
            if (bx, by, bz) in ((dx, dy, dz), (-dx, -dy, -dz)):
                continue
            via2 = _copy_pose(via1)
            via2.position.x += bx * self._cfg.cartesian_nudge_m
            via2.position.y += by * self._cfg.cartesian_nudge_m
            via2.position.z += bz * self._cfg.cartesian_nudge_m
            resp = self._call(
                self._cli_check_cartesian_path,
                CheckCartesianPath.Request(
                    start_pose=via1_stamped, target_pose=PoseStamped(header=header, pose=via2)
                ),
            )
            tried.append(
                f"({bx:.0f},{by:.0f},{bz:.0f})->{resp.fraction:.3f} "
                f"({resp.reason})"
            )
            if resp.full_path and resp.start_reachable and resp.trajectory_points > 0:
                return via1, via2
        raise RuntimeError(f"no viable bend axis found from via1 ({', '.join(tried)})")

    def _try_axis_directions(self) -> tuple[tuple[float, float, float] | None, list[str]]:
        start = self._latest_pose
        tried = []
        for dx, dy, dz in self._CANDIDATE_AXES:
            target = PoseStamped()
            target.header = start.header
            target.pose = _copy_pose(start.pose)
            target.pose.position.x += dx * self._cfg.cartesian_nudge_m
            target.pose.position.y += dy * self._cfg.cartesian_nudge_m
            target.pose.position.z += dz * self._cfg.cartesian_nudge_m
            resp = self._call(
                self._cli_check_cartesian_path,
                CheckCartesianPath.Request(start_pose=start, target_pose=target),
            )
            tried.append(
                f"({dx:.0f},{dy:.0f},{dz:.0f})->{resp.fraction:.3f} "
                f"({resp.reason})"
            )
            if resp.full_path and resp.start_reachable and resp.trajectory_points > 0:
                return (dx, dy, dz), tried
        return None, tried

    def _fetch_srdf_group_state(self, state_name: str) -> JointState | None:
        """Read the named <group_state> from the live SRDF (robot_description_semantic),
        which move_group - not moveit2_iface - actually populates. Lets the selftest move
        to a robot-specific, known-good pose (e.g. "test_configuration") without hard-coding
        any joint values for any particular arm."""
        if not state_name:
            return None
        if not self._cli_move_group_params.wait_for_service(timeout_sec=3.0):
            return None
        future = self._cli_move_group_params.call_async(
            GetParameters.Request(names=["robot_description_semantic"])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.values or not result.values[0].string_value:
            return None
        try:
            root = ET.fromstring(result.values[0].string_value)
        except ET.ParseError:
            return None
        for group_state in root.findall("group_state"):
            if group_state.get("name") != state_name:
                continue
            names = [j.get("name") for j in group_state.findall("joint")]
            positions = [float(j.get("value")) for j in group_state.findall("joint")]
            if names:
                return JointState(name=names, position=positions)
        return None

    def _fetch_joint_limits(self) -> dict[str, tuple[float, float]]:
        """Read URDF joint limits used only to choose a safe nudge direction."""
        if not self._cli_move_group_params.wait_for_service(timeout_sec=3.0):
            return {}
        future = self._cli_move_group_params.call_async(
            GetParameters.Request(names=["robot_description"])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.values or not result.values[0].string_value:
            return {}
        try:
            root = ET.fromstring(result.values[0].string_value)
        except ET.ParseError:
            return {}
        limits: dict[str, tuple[float, float]] = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            joint_type = joint.get("type")
            if not name or joint_type in {"fixed", "floating", "planar"}:
                continue
            if joint_type == "continuous":
                limits[name] = (-math.inf, math.inf)
                continue
            limit = joint.find("limit")
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                continue
            try:
                limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
            except (TypeError, ValueError):
                continue
        return limits

    def _move_to_named_state(self, state_name: str) -> bool:
        goal_state = self._fetch_srdf_group_state(state_name)
        if goal_state is None:
            self.get_logger().warn(
                f"SRDF named state '{state_name}' not found (or SRDF unavailable via "
                f"{self._prefix}/move_group) - skipping warm-up"
            )
            return False
        arm_names = self._discover_arm_joint_names()
        if arm_names:
            filtered = [
                (n, p) for n, p in zip(goal_state.name, goal_state.position) if n in arm_names
            ]
            if filtered:
                goal_state = JointState(
                    name=[n for n, _ in filtered], position=[p for _, p in filtered]
                )
        try:
            self._change_state("JOINT_TRAJ_CTL")
            result = self._send_action(
                self._ac_move_to_joint,
                MoveJoint.Goal(joint_state=goal_state),
                timeout=self._cfg.max_wait_sec_per_step,
            )
        except Exception as e:  # noqa: BLE001 - best-effort warm-up, never fatal
            self.get_logger().warn(f"Could not move to '{state_name}': {e}")
            return False
        if not result.success:
            self.get_logger().warn(f"move_to_joint to '{state_name}' returned success=false")
            return False
        try:
            self._wait_for_joints(goal_state, self._cfg.max_wait_sec_per_step)
        except RuntimeError as e:
            self.get_logger().warn(f"Move to '{state_name}' did not converge: {e}")
            return False
        return True

    def _probe_cartesian_direction(self) -> tuple[float, float, float]:
        """Find a unit direction from the current pose that check_cartesian_path
        reports as fully reachable. The robot's current configuration may be
        weak in every direction if it is sitting at/near a singularity or the
        edge of its workspace (e.g. a fully-extended elbow) - if so, try moving
        to the configured SRDF named state (default "test_configuration", a
        pose plenty of MoveIt configs define specifically for this purpose)
        before giving up."""
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        direction, tried = self._try_axis_directions()
        if direction is not None:
            return direction

        if self._run_motion and not self._motion_block_reason and self._cfg.warmup_group_state:
            self.get_logger().warn(
                "No viable Cartesian direction from the current pose "
                f"({', '.join(tried)}); trying SRDF state "
                f"'{self._cfg.warmup_group_state}' to escape it."
            )
            if self._move_to_named_state(self._cfg.warmup_group_state):
                direction, tried = self._try_axis_directions()
                if direction is not None:
                    return direction

        raise RuntimeError(
            "no viable Cartesian direction found from the current pose in any of the 6 "
            f"axis directions ({', '.join(tried)}); the arm is likely at/near a singularity "
            "or joint limit - jog it to a different pose (or set warmup_group_state to a "
            "valid SRDF named state) and rerun"
        )

    def test_cartesian_motion(self) -> None:
        self._require_motion()
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        # Probe (and any SRDF warm-up move it triggers) may switch control state and
        # move the robot - run it BEFORE (re-)asserting CART_TRAJ_CTL and capturing the
        # baseline pose, so neither is stale by the time we command a Cartesian move.
        dx, dy, dz = self._probe_cartesian_direction()
        self._change_state("CART_TRAJ_CTL")
        baseline = _copy_pose(self._latest_pose.pose)
        header = self._latest_pose.header

        out = _copy_pose(baseline)
        out.position.x += dx * self._cfg.cartesian_nudge_m
        out.position.y += dy * self._cfg.cartesian_nudge_m
        out.position.z += dz * self._cfg.cartesian_nudge_m

        # Out along the one direction the probe validated, then back to baseline -
        # NOT to an offset the opposite way, which the probe never validated and
        # would require double the travel in an untested direction.
        for target in (out, baseline):
            goal_pose = PoseStamped(header=header, pose=target)
            result = self._send_action(
                self._ac_move_to_pose,
                MoveCartesian.Goal(goal=goal_pose),
                timeout=self._cfg.max_wait_sec_per_step,
            )
            if not result.success:
                raise RuntimeError(f"move_to_pose returned success=false (target={target})")
            self._wait_for_pose(target, self._cfg.max_wait_sec_per_step)

    def test_cartesian_path_action(self) -> None:
        self._require_motion()
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        # Same ordering rationale as test_cartesian_motion: probe/warm-up first.
        dx, dy, dz = self._probe_cartesian_direction()
        self._change_state("CART_TRAJ_CTL")
        header = self._latest_pose.header
        baseline = _copy_pose(self._latest_pose.pose)
        via1, via2 = self._bent_waypoints(header, baseline, dx, dy, dz)

        waypoints = [
            PoseStamped(header=header, pose=via1),
            PoseStamped(header=header, pose=via2),
        ]
        result = self._send_action(
            self._ac_move_to_pose_path,
            MoveCartesianPath.Goal(poses=waypoints),
            timeout=self._cfg.max_wait_sec_per_step,
        )
        if not result.success:
            raise RuntimeError("move_to_pose_path returned success=false")
        self._wait_for_pose(via2, self._cfg.max_wait_sec_per_step)

    def test_cmd_pose_topic(self) -> None:
        """Exercises arm/cmd/pose, the topic-based "simple interface" pose command
        (pose_cmd_cb) - a different code path from the move_to_pose ACTION tested
        above, which goes through the action goal callback instead. mode:=advanced
        (the default) keeps this topic live alongside the actions, so no separate
        mode:=simple launch is needed to test it."""
        self._require_motion()
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        dx, dy, dz = self._probe_cartesian_direction()
        self._change_state("CART_TRAJ_CTL")
        baseline = _copy_pose(self._latest_pose.pose)
        header = self._latest_pose.header

        out = _copy_pose(baseline)
        out.position.x += dx * self._cfg.cartesian_nudge_m
        out.position.y += dy * self._cfg.cartesian_nudge_m
        out.position.z += dz * self._cfg.cartesian_nudge_m

        # planTopicPoseFast (the handler behind this topic) only tries pilz_LIN
        # (0.25s) then ompl_EST (0.75s), ignoring whatever arm/set_planner
        # configured, and a command is consumed once - silently dropped if it
        # arrives while a previous move is still finishing, or if planning
        # fails outright. _publish_and_wait_for_pose republishes as soon as it
        # notices no movement, instead of waiting out a long timeout in silence.
        # Out along the one direction the probe validated, then back to baseline
        # - not an offset the opposite way (untested direction, double travel).
        for target in (out, baseline):
            msg = PoseStamped(header=header, pose=target)
            self._publish_and_wait_for_pose(
                lambda m=msg: self._pub_cmd_pose.publish(m),
                target,
                self._cfg.max_wait_sec_per_step,
            )

    def test_cmd_pose_topic_joint_mode(self) -> None:
        """Exercise the separate planAndExecTopicPoseJoint implementation."""
        self._require_motion()
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        dx, dy, dz = self._probe_cartesian_direction()
        self._change_state("JOINT_TRAJ_CTL")
        baseline = _copy_pose(self._latest_pose.pose)
        header = self._latest_pose.header
        out = _copy_pose(baseline)
        out.position.x += dx * self._cfg.cartesian_nudge_m
        out.position.y += dy * self._cfg.cartesian_nudge_m
        out.position.z += dz * self._cfg.cartesian_nudge_m
        for target in (out, baseline):
            msg = PoseStamped(header=header, pose=target)
            self._publish_and_wait_for_pose(
                lambda m=msg: self._pub_cmd_pose.publish(m),
                target,
                self._cfg.max_wait_sec_per_step,
            )

    def test_cmd_traj_topic(self) -> None:
        """Exercises arm/cmd/traj, the topic-based Cartesian waypoint path
        (cart_poses_cb) - a different code path from the move_to_pose_path
        ACTION tested above."""
        self._require_motion()
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        dx, dy, dz = self._probe_cartesian_direction()
        self._change_state("CART_TRAJ_CTL")
        header = self._latest_pose.header
        baseline = _copy_pose(self._latest_pose.pose)
        via1, via2 = self._bent_waypoints(header, baseline, dx, dy, dz)

        # Single-shot like arm/cmd/pose: a command arriving while the arm is still
        # finishing a previous move (e.g. right after test_cmd_pose_topic) is
        # silently dropped ("Trajectory still executing; ignoring Cartesian
        # waypoint trajectory"). _publish_and_wait_for_pose republishes as soon
        # as it notices no movement, instead of waiting out a long timeout.
        msg = CartesianWaypoints(poses=[via1, via2])
        self._publish_and_wait_for_pose(
            lambda: self._pub_cmd_traj.publish(msg), via2, self._cfg.max_wait_sec_per_step,
        )

    def _servo_burst(self, dx: float, dy: float, dz: float, duration: float) -> tuple[float, bool]:
        """Returns (displacement, halted). `halted` means moveit2_iface's own servo
        status reported INVALID/HALT_FOR_SINGULARITY/HALT_FOR_COLLISION at some point
        during the burst - i.e. servo correctly refused to move, not that it's broken."""
        if self._latest_pose is None:
            raise RuntimeError("lost current_pose during servo test")
        pre_pose = _copy_pose(self._latest_pose.pose)
        self._servo_halt_seen = False
        end = time.time() + duration
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = dx * self._cfg.servo_twist_linear_z
            msg.twist.linear.y = dy * self._cfg.servo_twist_linear_z
            msg.twist.linear.z = dz * self._cfg.servo_twist_linear_z
            self._pub_servo_twist.publish(msg)
            self._spin(0.05)

        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self._pub_servo_twist.publish(zero)
        self._spin(self._cfg.settle_time_sec)

        if self._latest_pose is None:
            raise RuntimeError("lost current_pose during servo test")
        return _pose_distance_m(pre_pose, self._latest_pose.pose), self._servo_halt_seen

    def _try_servo_directions(self) -> tuple[bool, list[str], bool]:
        """Returns (succeeded, tried_log, any_halt_seen)."""
        tried = []
        any_halt = False
        for dx, dy, dz in self._CANDIDATE_AXES:
            displacement, halted = self._servo_burst(dx, dy, dz, self._cfg.servo_burst_duration_sec)
            any_halt = any_halt or halted
            tag = " [servo halted: singularity/collision]" if halted else ""
            tried.append(f"({dx:.0f},{dy:.0f},{dz:.0f})->{displacement:.4f}m{tag}")
            if displacement >= self._cfg.servo_min_expected_displacement_m:
                return True, tried, any_halt
        return False, tried, any_halt

    def test_servo_motion(self) -> None:
        self._require_motion()
        if self._cfg.skip_servo:
            raise SkipTest("servo checks disabled in config (skip.servo)")
        if self._latest_pose is None:
            raise RuntimeError("no baseline pose available")
        # A direction validated by check_cartesian_path only proves IK exists there -
        # it says nothing about manipulability for velocity control. MoveIt Servo
        # silently scales down its output near a low-manipulability direction or a
        # joint limit, without raising a halt status, so reusing that direction here
        # can produce near-zero measured movement despite servo working correctly.
        # Try each candidate axis directly, measuring actual displacement, until one
        # clears the threshold.
        self._change_state("SERVO_CTL")
        try:
            # moveit2_iface ignores twist commands received in the first 0.5s after
            # entering SERVO_CTL, to drop stale buffered commands.
            self._spin(0.6)

            ok, tried, any_halt = self._try_servo_directions()
            if ok:
                return

            if any_halt and self._cfg.warmup_group_state:
                # moveit2_iface's own singularity/collision guard is correctly refusing
                # to move from here in any direction - not a servo defect. Move to the
                # configured SRDF state and retry from there.
                self.get_logger().warn(
                    "Servo halted (singularity/collision) in every direction from the "
                    f"current pose ({', '.join(tried)}); trying SRDF state "
                    f"'{self._cfg.warmup_group_state}' to escape it."
                )
                if self._move_to_named_state(self._cfg.warmup_group_state):
                    self._change_state("SERVO_CTL")
                    self._spin(0.6)
                    ok, tried, _ = self._try_servo_directions()
                    if ok:
                        return

            raise RuntimeError(
                f"no direction produced >= {self._cfg.servo_min_expected_displacement_m}m of "
                f"movement during a {self._cfg.servo_burst_duration_sec}s burst "
                f"({', '.join(tried)})"
            )
        finally:
            try:
                self._change_state("JOINT_TRAJ_CTL")
            except Exception as e:  # noqa: BLE001 - preserve the primary test failure
                self.get_logger().error(f"Failed to leave SERVO_CTL during cleanup: {e}")

    def test_joy_and_keyboard(self) -> None:
        raise SkipTest(
            "joy_ctl/keyboard_ctl need physical joystick/keyboard input; "
            "verify manually (see tests/arm_api2_selftest/README.md)"
        )

    # --- driver ---------------------------------------------------------------

    def run(self) -> int:
        self._run("baseline (current_pose + joint_states)", self.test_baseline)
        if self._results[-1].status != "PASS":
            self.get_logger().error(
                "Baseline state not available - aborting remaining tests. Is moveit2_iface "
                "running and reachable under the configured namespace?"
            )
            self._print_summary()
            return 1

        ctl_result = self._run("arm/state/ctl_state topic", self.test_ctl_state_topic)
        self._run("arm/change_state rejects invalid state", self.test_invalid_change_state)
        self._run("action servers reject invalid state/goal combinations", self.test_action_state_rejections)
        if ctl_result.status != "PASS":
            self._motion_block_reason = "control-state topic is not healthy"

        if self._run_motion and not self._motion_block_reason and self._cfg.warmup_group_state:
            warmup = self._run(
                f"move to '{self._cfg.warmup_group_state}' (initial warm-up)",
                self.test_initial_warmup,
            )
            if warmup.status != "PASS":
                self._motion_block_reason = "initial warm-up failed"

        self._run("arm/set_vel_acc", self.test_set_vel_acc)
        self._run("arm/set_eelink", self.test_set_eelink)
        self._run("arm/check_reachability", self.test_check_reachability)
        self._run("arm/check_cartesian_path", self.test_check_cartesian_path)
        self._run(
            "gripper services + arm/gripper_control action (real motion)",
            self.test_gripper_services,
        )
        self._run(
            "arm/set_planner x arm/move_to_joint (planonly, no motion)",
            self.test_planner_and_planonly,
        )
        self._run("JOINT_TRAJ_CTL + arm/move_to_joint (real motion)", self.test_joint_motion)
        self._run("CART_TRAJ_CTL + arm/move_to_pose (real motion)", self.test_cartesian_motion)
        self._run("arm/move_to_pose_path (real motion)", self.test_cartesian_path_action)
        self._run("arm/cmd/pose topic (simple interface, real motion)", self.test_cmd_pose_topic)
        self._run(
            "arm/cmd/pose in JOINT_TRAJ_CTL (real motion)",
            self.test_cmd_pose_topic_joint_mode,
        )
        self._run("arm/cmd/traj topic (simple interface, real motion)", self.test_cmd_traj_topic)
        self._run("SERVO_CTL + servo_twist_cmd (real motion)", self.test_servo_motion)
        self._run("joy_ctl / keyboard_ctl", self.test_joy_and_keyboard)

        return self._print_summary()

    def cleanup(self) -> None:
        self.cancel_active_goal()
        if self._initial_ctl_state is None or self._latest_ctl_state is None:
            return
        if self._latest_ctl_state.data == self._initial_ctl_state:
            return
        try:
            self._change_state(self._initial_ctl_state)
        except Exception as e:  # noqa: BLE001 - shutdown cleanup is best effort
            self.get_logger().error(
                f"Could not restore initial ctl_state {self._initial_ctl_state}: {e}"
            )

    def _print_summary(self) -> int:
        name_width = max((len(r.name) for r in self._results), default=10)
        print("\n=== arm_api2 selftest summary ===")
        for r in self._results:
            marker = {"PASS": "PASS", "FAIL": "FAIL", "SKIP": "SKIP"}[r.status]
            line = f"[{marker}] {r.name.ljust(name_width)}"
            if r.detail:
                line += f"  - {r.detail}"
            print(line)
        n_pass = sum(1 for r in self._results if r.status == "PASS")
        n_fail = sum(1 for r in self._results if r.status == "FAIL")
        n_skip = sum(1 for r in self._results if r.status == "SKIP")
        print(f"\n{n_pass} passed, {n_fail} failed, {n_skip} skipped\n")
        return 1 if n_fail else 0


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        loaded = yaml.safe_load(f)
    if loaded is None:
        return {}
    if not isinstance(loaded, dict):
        raise ValueError(f"YAML root in {path} must be a mapping")
    return loaded


def _load_config(path: Path | None) -> SelfTestConfig:
    default_path = (
        Path(get_package_share_directory("arm_api2"))
        / "tests"
        / "arm_api2_selftest"
        / "selftest_defaults.yaml"
    )
    data: dict[str, Any] = {}
    if default_path.is_file():
        data = _load_yaml_mapping(default_path)
    if path is not None:
        data = _deep_merge(data, _load_yaml_mapping(path))
    config = SelfTestConfig.from_yaml(data)
    config.validate()
    return config


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run functional smoke tests against a live arm_api2 moveit2_iface node."
    )
    parser.add_argument(
        "config_file",
        nargs="?",
        default=None,
        help="Path to a YAML file overriding selftest_defaults.yaml",
    )
    parser.add_argument(
        "--move",
        action="store_true",
        help="Also run tests that command real motion (joint, Cartesian, path, servo)",
    )
    parser.add_argument(
        "--robot-ns",
        default=None,
        help="Override robot_namespace from the config file",
    )
    args, ros_rest = parser.parse_known_args(args=sys.argv[1:])

    cfg_path = Path(args.config_file).expanduser() if args.config_file else None
    try:
        cfg = _load_config(cfg_path)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as e:
        parser.error(f"invalid selftest configuration: {e}")
    if args.robot_ns is not None:
        cfg.robot_namespace = args.robot_ns

    rclpy.init(args=[sys.argv[0]] + ros_rest)
    node = SelfTestNode(cfg, run_motion=args.move)
    try:
        exit_code = node.run()
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
