#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Run arm_api2 pick-and-place steps from YAML; wait for each motion to converge before the next.

moveit2_simple_iface drives planning/execution; this script waits until /arm/state/current_pose
matches each target (stable samples).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_srvs.srv import Trigger

try:
    from arm_api2_msgs.srv import ChangeState
except ImportError:
    ChangeState = None  # type: ignore

try:
    from arm_api2_msgs.msg import PlanStatus
except ImportError:
    PlanStatus = None  # type: ignore

try:
    from arm_api2_msgs.srv import SetStringParam
except ImportError:
    SetStringParam = None  # type: ignore


def _quaternion_angle_rad(q1: tuple[float, float, float, float], q2: tuple[float, float, float, float]) -> float:
    """Minimal rotation angle between two unit quaternions (uses absolute dot for q/-q equivalence)."""
    dot = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    dot = min(1.0, max(0.0, dot))
    return 2.0 * math.acos(dot)


def _pose_close(
    current: Pose,
    target: Pose,
    pos_tol: float,
    ang_tol: float,
) -> bool:
    dx = current.position.x - target.position.x
    dy = current.position.y - target.position.y
    dz = current.position.z - target.position.z
    if math.sqrt(dx * dx + dy * dy + dz * dz) > pos_tol:
        return False
    q1 = (
        current.orientation.x,
        current.orientation.y,
        current.orientation.z,
        current.orientation.w,
    )
    q2 = (
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w,
    )
    return _quaternion_angle_rad(q1, q2) <= ang_tol


def _load_yaml(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Config must be a mapping: {path}")
    return data


def _pose_from_dict(d: dict[str, Any]) -> Pose:
    p = d.get("position") or {}
    o = d.get("orientation") or {}
    pose = Pose()
    pose.position.x = float(p.get("x", 0.0))
    pose.position.y = float(p.get("y", 0.0))
    pose.position.z = float(p.get("z", 0.0))
    pose.orientation.x = float(o.get("x", 0.0))
    pose.orientation.y = float(o.get("y", 0.0))
    pose.orientation.z = float(o.get("z", 0.0))
    pose.orientation.w = float(o.get("w", 1.0))
    return pose


class PickPlaceSequence(Node):
    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__("pick_place_sequence")
        if ChangeState is None:
            raise RuntimeError("arm_api2_msgs is not importable; source the workspace overlay that built arm_api2.")
        self._config = config
        ns = str(config.get("robot_namespace", "ur1")).strip("/")
        frame = str(config.get("planning_frame", "world"))
        self._frame_id = frame
        self._ns = ns

        conv = config.get("convergence") or {}
        self._pos_tol = float(conv.get("position_tolerance_m", 0.012))
        self._ang_tol = float(conv.get("orientation_tolerance_rad", 0.08))
        self._stable_need = int(conv.get("stable_samples", 6))
        self._spin_period = float(conv.get("spin_period_sec", 0.05))
        self._max_wait = float(conv.get("max_wait_sec_per_step", 120.0))

        self._gripper_settle = float(config.get("gripper_settle_sec", 0.8))

        prefix = f"/{ns}" if ns else ""
        self._pub_pose = self.create_publisher(PoseStamped, f"{prefix}/arm/cmd/pose", 10)
        self._sub_pose = self.create_subscription(
            PoseStamped,
            f"{prefix}/arm/state/current_pose",
            self._on_current_pose,
            10,
        )
        self._latest_pose: PoseStamped | None = None
        self._latest_plan_status: Any | None = None

        if PlanStatus is not None:
            plan_status_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
            self._sub_plan_status = self.create_subscription(
                PlanStatus,
                f"{prefix}/arm/state/plan_status",
                self._on_plan_status,
                plan_status_qos,
            )

        self._cli_change_state = self.create_client(ChangeState, f"{prefix}/arm/change_state")
        self._cli_open = self.create_client(Trigger, f"{prefix}/arm/open_gripper")
        self._cli_close = self.create_client(Trigger, f"{prefix}/arm/close_gripper")
        self._cli_set_planner = (
            self.create_client(SetStringParam, f"{prefix}/arm/set_planner") if SetStringParam is not None else None
        )
        self._current_planner: str | None = None

    def _on_current_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _on_plan_status(self, msg: Any) -> None:
        self._latest_plan_status = msg

    def _wait_services(self, timeout: float = 60.0) -> None:
        self._cli_change_state.wait_for_service(timeout_sec=timeout)
        self._cli_open.wait_for_service(timeout_sec=timeout)
        self._cli_close.wait_for_service(timeout_sec=timeout)
        if self._cli_set_planner is not None:
            self._cli_set_planner.wait_for_service(timeout_sec=timeout)

    def _set_planner(self, planner: str) -> None:
        if self._cli_set_planner is None or planner == self._current_planner:
            return
        req = SetStringParam.Request()
        req.value = planner
        fut = self._cli_set_planner.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result() is None or not fut.result().success:
            raise RuntimeError(f"set_planner to {planner!r} failed")
        self._current_planner = planner
        self.get_logger().info(f"Planner switched to {planner}")

    def _call_change_state(self, state: str) -> None:
        req = ChangeState.Request()
        req.state = state
        fut = self._cli_change_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result() is None or not fut.result().success:
            raise RuntimeError(f"change_state to {state!r} failed")

    def _call_trigger(self, client: Any, name: str) -> None:
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result() is None:
            raise RuntimeError(f"{name} service call failed (no response)")

    def _publish_pose_target(self, pose: Pose) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose = pose
        self._pub_pose.publish(msg)
        self.get_logger().info(
            f"Published pose target position=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})"
        )

    def _publish_pose_target_until_accepted(self, pose: Pose, timeout: float) -> float:
        """Publish pose, retrying while the controller rejects it (e.g. still executing the
        previous trajectory, or MoveIt planning is still in flight). Returns elapsed seconds
        spent, so callers can budget the remaining time against the same per-step timeout."""
        has_feedback = hasattr(self, "_sub_plan_status")
        if not has_feedback:
            # No PlanStatus feedback available (e.g. message type missing); fire-and-forget.
            self._publish_pose_target(pose)
            return 0.0

        retry_period = 0.3
        # MoveIt planning alone can take up to ~pose_plan_time_sec_ (default 1.5s) before a
        # PlanStatus response is published; wait comfortably longer before re-publishing so we
        # don't race ahead of a still-pending plan result.
        accept_wait = 3.0
        elapsed = 0.0
        while True:
            self._latest_plan_status = None
            self._publish_pose_target(pose)

            waited = 0.0
            while waited < accept_wait:
                rclpy.spin_once(self, timeout_sec=self._spin_period)
                waited += self._spin_period
                elapsed += self._spin_period
                if self._latest_plan_status is not None:
                    break

            status = self._latest_plan_status
            if status is not None and status.success:
                return elapsed

            if status is None:
                self.get_logger().warn("No plan_status response received in time; retrying.")
            else:
                self.get_logger().warn(
                    f"Pose command rejected ({status.error_code}): {status.reason}; retrying."
                )
            if elapsed >= timeout:
                last = f"last error: {status.error_code}" if status is not None else "no response"
                raise TimeoutError(
                    f"Pose command kept being rejected for {elapsed:.1f}s ({last})"
                )
            time.sleep(retry_period)
            elapsed += retry_period

    def _wait_pose_converged(self, target: Pose, timeout: float) -> None:
        stable = 0
        elapsed = 0.0
        while elapsed < timeout:
            rclpy.spin_once(self, timeout_sec=self._spin_period)
            elapsed += self._spin_period
            if self._latest_pose is None:
                continue
            cur = self._latest_pose.pose
            if _pose_close(cur, target, self._pos_tol, self._ang_tol):
                stable += 1
                if stable >= self._stable_need:
                    self.get_logger().info("Target pose reached (within tolerance).")
                    return
            else:
                stable = 0
        raise TimeoutError(
            f"Pose did not converge within {timeout:.1f}s "
            f"(pos_tol={self._pos_tol}, ang_tol={self._ang_tol})"
        )

    def run(self) -> None:
        self._wait_services()
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        default_planner = self._config.get("default_planner")
        if default_planner:
            self._set_planner(str(default_planner))

        steps = self._config.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("config must contain non-empty 'steps' list")

        for i, step in enumerate(steps):
            if not isinstance(step, dict):
                raise ValueError(f"steps[{i}] must be a mapping")
            name = step.get("name", f"step_{i}")
            wait_override = step.get("wait_timeout_sec")
            step_timeout = float(wait_override) if wait_override is not None else self._max_wait

            self.get_logger().info(f"--- Step {i + 1}/{len(steps)}: {name} ---")

            planner = step.get("planner")
            if planner:
                self._set_planner(str(planner))

            cs = step.get("change_state")
            if cs:
                self._call_change_state(str(cs))
                rclpy.spin_once(self, timeout_sec=0.05)

            if step.get("open_gripper"):
                self._call_trigger(self._cli_open, "open_gripper")
                self._sleep_gripper()

            if step.get("close_gripper"):
                self._call_trigger(self._cli_close, "close_gripper")
                self._sleep_gripper()

            pose_data = step.get("pose")
            if pose_data:
                if not isinstance(pose_data, dict):
                    raise ValueError(f"steps[{i}].pose must be a mapping")
                target = _pose_from_dict(pose_data)
                spent = self._publish_pose_target_until_accepted(target, timeout=step_timeout)
                self._wait_pose_converged(target, timeout=max(step_timeout - spent, 1.0))

        self.get_logger().info("All steps completed.")

    def _sleep_gripper(self) -> None:
        time.sleep(self._gripper_settle)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run arm_api2 pick-and-place from a YAML sequence file.")
    parser.add_argument(
        "config_file",
        nargs="?",
        default="",
        help="Path to YAML sequence (default: arm_api2 share tutorials/pick_place_sequence_lab_table_one.yaml)",
    )
    args, ros_rest = parser.parse_known_args(args=sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_rest)
    default_cfg = Path(get_package_share_directory("arm_api2")) / "tutorials" / "pick_place_sequence_lab_table_one.yaml"
    cfg_path = Path(args.config_file).expanduser() if args.config_file else default_cfg
    if not cfg_path.is_file():
        print(f"Config not found: {cfg_path}", file=sys.stderr)
        sys.exit(1)

    data = _load_yaml(cfg_path)
    node = PickPlaceSequence(data)
    try:
        node.run()
    except (KeyboardInterrupt, TimeoutError, RuntimeError) as e:
        node.get_logger().error(str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
