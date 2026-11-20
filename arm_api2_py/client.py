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
"""arm_api2 client: one object for actions and services, blocking or async."""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

import rclpy
from arm_api2_msgs.action import (
    ExecutePlan,
    ExecuteTask,
    MoveCartesian,
    MoveCartesianPath,
    MoveJoint,
    MoveRelative,
    MoveToNamed,
)
from arm_api2_msgs.msg import TaskStep
from arm_api2_msgs.srv import (
    AddCollisionMesh,
    AddCollisionObject,
    AttachObject,
    ChangeState,
    ClearPathConstraints,
    DetachObject,
    ListCollisionObjects,
    ListNamedPoses,
    NameRequest,
    PlanTask,
    SaveNamedPose,
    SetPathConstraints,
    SetVelAcc,
)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, OrientationConstraint
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from arm_api2_py.task import Task, joint_state, relative_move

STATES = ("IDLE", "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL")


@dataclass
class ArmResult:
    """Outcome of an action or service. Truthy when it succeeded."""

    success: bool
    error_code: str = ""
    message: str = ""
    failed_step: int = -1
    plan_id: str = ""
    raw: Any = None

    def __bool__(self) -> bool:
        return self.success


@dataclass
class PlanResult(ArmResult):
    segments: list = field(default_factory=list)
    planned_duration_sec: float = 0.0
    start_state: Any = None


def _from_action_result(status_ok: bool, res: Any, status_text: str) -> ArmResult:
    if res is None:
        return ArmResult(False, "NO_RESULT", status_text)
    return ArmResult(
        success=bool(getattr(res, "success", False)) and status_ok,
        error_code=getattr(res, "error_code", "") or ("SUCCESS" if res.success else status_text),
        message=getattr(res, "message", "") or status_text,
        failed_step=getattr(res, "failed_step", -1),
        plan_id=getattr(res, "plan_id", ""),
        raw=res,
    )


class ActionHandle:
    """A sent goal. result() blocks; done() polls; cancel() asks the server to stop."""

    _STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

    def __init__(self, send_future, label: str):
        self.label = label
        self._event = threading.Event()
        self._goal_handle = None
        self._result: Optional[ArmResult] = None
        self._lock = threading.Lock()
        send_future.add_done_callback(self._on_goal_response)

    def _finish(self, r: ArmResult) -> None:
        with self._lock:
            if self._result is None:
                self._result = r
        self._event.set()

    def _on_goal_response(self, fut) -> None:
        try:
            gh = fut.result()
        except Exception as e:  # noqa: BLE001
            self._finish(ArmResult(False, "SEND_FAILED", f"{self.label}: {e}"))
            return
        if gh is None or not gh.accepted:
            self._finish(ArmResult(False, "REJECTED",
                                   f"{self.label}: goal rejected (state, busy or invalid goal; "
                                   "see the moveit2_iface log)"))
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        try:
            wrapped = fut.result()
        except Exception as e:  # noqa: BLE001
            self._finish(ArmResult(False, "NO_RESULT", f"{self.label}: {e}"))
            return
        status = self._STATUS.get(wrapped.status, str(wrapped.status))
        self._finish(_from_action_result(status == "SUCCEEDED", wrapped.result, status))

    def done(self) -> bool:
        return self._event.is_set()

    def result(self, timeout: Optional[float] = None) -> ArmResult:
        if not self._event.wait(timeout):
            return ArmResult(False, "TIMEOUT", f"{self.label}: no result within {timeout} s")
        assert self._result is not None
        return self._result

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


class Arm:
    """
    Client for one arm_api2 instance.

    namespace: robot namespace (e.g. "ur1" for ur1/arm/...); "" for a single robot.
    node: an existing node; it must then be spun by the caller unless spin=True.
    """

    def __init__(
        self,
        namespace: str = "",
        node: Optional[Node] = None,
        spin: bool = True,
        server_timeout: float = 5.0,
        iface_node: str = "moveit2_iface",
    ):
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self.server_timeout = server_timeout
        ns = namespace.strip("/")
        self._prefix = (ns + "/") if (ns and node is not None) else ""
        self.node = node or Node(f"arm_api2_py_{os.getpid()}", namespace=ns or None)
        self._iface_node = iface_node
        self._clients: dict = {}
        self._actions: dict = {}
        self._pose: Optional[PoseStamped] = None
        self._state: str = ""
        self._executor = None
        self._thread = None
        from std_msgs.msg import String

        self.node.create_subscription(PoseStamped, self._n("arm/state/current_pose"),
                                      self._on_pose, qos_profile_sensor_data)
        self.node.create_subscription(String, self._n("arm/state/ctl_state"),
                                      lambda m: setattr(self, "_state", m.data), 10)
        if spin:
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self.node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()

    # ---------------------------------------------------------------- plumbing
    def _n(self, name: str) -> str:
        return self._prefix + name

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def close(self) -> None:
        """Stop the background executor and destroy the node (call once, at the end)."""
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._executor = None
            self._thread = None
        for c in self._actions.values():
            c.destroy()
        self._actions.clear()
        self.node.destroy_node()
        if self._owns_context:
            rclpy.try_shutdown()

    def __enter__(self) -> "Arm":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _call(self, srv_type, name: str, request, timeout: Optional[float] = 30.0):
        """Service call; returns (response or None, error text)."""
        key = (srv_type, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(srv_type, self._n(name))
        cli = self._clients[key]
        if not cli.wait_for_service(timeout_sec=self.server_timeout):
            return None, f"service {self._n(name)} not available"
        fut = cli.call_async(request)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None, f"{name}: no response within {timeout} s"
        try:
            return fut.result(), ""
        except Exception as e:  # noqa: BLE001
            return None, f"{name}: {e}"

    def _srv(self, srv_type, name: str, request, timeout: Optional[float] = 30.0) -> ArmResult:
        res, err = self._call(srv_type, name, request, timeout)
        if res is None:
            return ArmResult(False, "NO_SERVICE", err)
        ok = bool(getattr(res, "success", True))
        return ArmResult(ok, "SUCCESS" if ok else "FAILED", getattr(res, "message", ""), raw=res)

    def _send(
        self, action_type, name: str, goal, wait: bool, feedback: Optional[Callable],
        timeout: Optional[float],
    ):
        key = (action_type, name)
        if key not in self._actions:
            self._actions[key] = ActionClient(self.node, action_type, self._n(name))
        cli = self._actions[key]
        if not cli.wait_for_server(timeout_sec=self.server_timeout):
            r = ArmResult(False, "NO_SERVER", f"action {self._n(name)} not available")
            if wait:
                return r
            h = ActionHandle.__new__(ActionHandle)
            h.label, h._event, h._goal_handle, h._lock = name, threading.Event(), None, threading.Lock()
            h._result = None
            h._finish(r)
            return h
        fb = (lambda m: feedback(m.feedback)) if feedback else None
        handle = ActionHandle(cli.send_goal_async(goal, feedback_callback=fb), name)
        return handle.result(timeout) if wait else handle

    # ---------------------------------------------------------------- state
    @property
    def state(self) -> str:
        """Return the last control state published on arm/state/ctl_state, e.g. JOINT_TRAJ_CTL."""
        return self._state

    def current_pose(self, timeout: float = 2.0) -> Optional[PoseStamped]:
        """Return the latest TCP pose from arm/state/current_pose, or None if none arrived."""
        ev = threading.Event()
        end = timeout
        while self._pose is None and end > 0:
            ev.wait(0.05)
            end -= 0.05
        return self._pose

    def set_state(self, state: str) -> ArmResult:
        if state not in STATES:
            raise ValueError(f"state must be one of {STATES}")
        req = ChangeState.Request()
        req.state = state
        return self._srv(ChangeState, "arm/change_state", req)

    def set_vel_acc(self, vel: float, acc: float) -> ArmResult:
        req = SetVelAcc.Request()
        req.max_vel = float(vel)
        req.max_acc = float(acc)
        return self._srv(SetVelAcc, "arm/set_vel_acc", req)

    def stop(self) -> ArmResult:
        """arm/stop: cancels arm_api2 motions and tasks, sends MoveIt stop."""
        return self._srv(Trigger, "arm/stop", Trigger.Request(), timeout=5.0)

    # ---------------------------------------------------------------- single motions
    def move_joint(self, positions: Sequence[float], names: Optional[Sequence[str]] = None, *,
                   wait: bool = True, feedback=None, timeout: Optional[float] = None):
        """arm/move_to_joint (state JOINT_TRAJ_CTL). Without names: planning-group order."""
        goal = MoveJoint.Goal()
        goal.joint_state = joint_state(positions, names)
        return self._send(MoveJoint, "arm/move_to_joint", goal, wait, feedback, timeout)

    def move_pose(self, target: PoseStamped, *, wait: bool = True, feedback=None,
                  timeout: Optional[float] = None):
        """arm/move_to_pose (state CART_TRAJ_CTL; frame must be the planning frame)."""
        goal = MoveCartesian.Goal()
        goal.goal = target
        return self._send(MoveCartesian, "arm/move_to_pose", goal, wait, feedback, timeout)

    def move_path(self, waypoints: Sequence[PoseStamped], *, wait: bool = True, feedback=None,
                  timeout: Optional[float] = None):
        """arm/move_to_pose_path (state CART_TRAJ_CTL, >= 2 poses in the planning frame)."""
        goal = MoveCartesianPath.Goal()
        goal.poses = list(waypoints)
        return self._send(MoveCartesianPath, "arm/move_to_pose_path", goal, wait, feedback,
                          timeout)

    def move_relative(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0,
                      pitch: float = 0.0, yaw: float = 0.0, *, frame: str = "tcp",
                      linear: bool = True, wait: bool = True, feedback=None,
                      timeout: Optional[float] = None):
        """arm/move_relative: offset in the TCP ('tcp') or planning ('base') frame."""
        goal = MoveRelative.Goal()
        goal.relative = relative_move(x, y, z, roll, pitch, yaw, frame, linear)
        return self._send(MoveRelative, "arm/move_relative", goal, wait, feedback, timeout)

    def move_to_named(self, name: str, *, wait: bool = True, feedback=None,
                      timeout: Optional[float] = None):
        goal = MoveToNamed.Goal()
        goal.name = name
        return self._send(MoveToNamed, "arm/move_to_named", goal, wait, feedback, timeout)

    # ---------------------------------------------------------------- tasks / plans
    def task(self) -> Task:
        return Task(self)

    @staticmethod
    def _steps(steps) -> list:
        steps = steps.steps if isinstance(steps, Task) else list(steps)
        if not all(isinstance(s, TaskStep) for s in steps):
            raise TypeError("steps must be TaskStep messages (use arm.task())")
        return steps

    def plan(self, steps, timeout: Optional[float] = 120.0) -> PlanResult:
        """arm/plan: plan the whole task; nothing moves. Keeps the plan by plan_id."""
        req = PlanTask.Request()
        req.steps = self._steps(steps)
        res, err = self._call(PlanTask, "arm/plan", req, timeout)
        if res is None:
            return PlanResult(False, "NO_SERVICE", err)
        return PlanResult(
            success=res.success, error_code=res.error_code, message=res.message,
            failed_step=res.failed_step, plan_id=res.plan_id, raw=res,
            segments=list(res.segments), planned_duration_sec=res.planned_duration_sec,
            start_state=res.start_state,
        )

    def execute_plan(self, plan_id: str, *, wait: bool = True, feedback=None,
                     timeout: Optional[float] = None):
        """arm/execute_plan: run a stored plan (refused if the robot moved since planning)."""
        goal = ExecutePlan.Goal()
        goal.plan_id = plan_id
        return self._send(ExecutePlan, "arm/execute_plan", goal, wait, feedback, timeout)

    def execute_task(self, steps, *, wait: bool = True, feedback=None,
                     timeout: Optional[float] = None):
        """arm/execute_task: plan everything, then move (nothing moves if a step fails)."""
        goal = ExecuteTask.Goal()
        goal.steps = self._steps(steps)
        return self._send(ExecuteTask, "arm/execute_task", goal, wait, feedback, timeout)

    # ---------------------------------------------------------------- gripper
    def open_gripper(self) -> ArmResult:
        return self._srv(Trigger, "arm/open_gripper", Trigger.Request())

    def close_gripper(self) -> ArmResult:
        return self._srv(Trigger, "arm/close_gripper", Trigger.Request())

    # ---------------------------------------------------------------- named poses
    def save_named_pose(self, name: str, positions: Optional[Sequence[float]] = None,
                        names: Optional[Sequence[str]] = None,
                        overwrite: bool = False) -> ArmResult:
        """Save the current joints (or the given ones) as a named pose."""
        req = SaveNamedPose.Request()
        req.name = name
        req.overwrite = overwrite
        if positions is not None:
            req.joint_state = joint_state(positions, names)
        return self._srv(SaveNamedPose, "arm/save_named_pose", req)

    def list_named_poses(self) -> dict:
        """Return {name: {"source": ..., "joints": {joint: value}}}."""
        res, err = self._call(ListNamedPoses, "arm/list_named_poses", ListNamedPoses.Request())
        if res is None:
            raise RuntimeError(err)
        return {
            n: {"source": s, "joints": dict(zip(js.name, js.position))}
            for n, s, js in zip(res.names, res.sources, res.joint_states)
        }

    def delete_named_pose(self, name: str) -> ArmResult:
        req = NameRequest.Request()
        req.name = name
        return self._srv(NameRequest, "arm/delete_named_pose", req)

    # ---------------------------------------------------------------- scene
    def add_box(self, object_id: str, size: Sequence[float], position: Sequence[float],
                quat: Sequence[float] = (0.0, 0.0, 0.0, 1.0)) -> ArmResult:
        return self._add_primitive(object_id, SolidPrimitive.BOX, size, position, quat)

    def add_cylinder(self, object_id: str, height: float, radius: float,
                     position: Sequence[float],
                     quat: Sequence[float] = (0.0, 0.0, 0.0, 1.0)) -> ArmResult:
        return self._add_primitive(object_id, SolidPrimitive.CYLINDER, (height, radius),
                                   position, quat)

    def add_sphere(self, object_id: str, radius: float, position: Sequence[float]) -> ArmResult:
        return self._add_primitive(object_id, SolidPrimitive.SPHERE, (radius,), position,
                                   (0.0, 0.0, 0.0, 1.0))

    def _add_primitive(self, object_id, ptype, dims, position, quat) -> ArmResult:
        """Primitives in the planning frame (arm/add_collision_object)."""
        req = AddCollisionObject.Request()
        req.id = object_id
        req.primitive_type = ptype
        req.dimensions = [float(d) for d in dims]
        req.position.x, req.position.y, req.position.z = (float(v) for v in position)
        (req.orientation.x, req.orientation.y, req.orientation.z,
         req.orientation.w) = (float(v) for v in quat)
        return self._srv(AddCollisionObject, "arm/add_collision_object", req)

    def add_mesh(self, object_id: str, mesh_path: str, pose_: Optional[PoseStamped] = None,
                 scale: float = 1.0) -> ArmResult:
        """Mesh from file/package:// URI; scale 0.001 for meshes in mm."""
        req = AddCollisionMesh.Request()
        req.id = object_id
        req.mesh_path = mesh_path
        req.scale = float(scale)
        if pose_ is not None:
            req.pose = pose_
        else:
            req.pose.pose.orientation.w = 1.0
        return self._srv(AddCollisionMesh, "arm/add_collision_mesh", req)

    def remove_object(self, object_id: str) -> ArmResult:
        req = NameRequest.Request()
        req.name = object_id
        return self._srv(NameRequest, "arm/remove_collision_object", req)

    def list_objects(self) -> dict:
        """Return {"world": [...], "attached": {id: link}}."""
        res, err = self._call(ListCollisionObjects, "arm/list_collision_objects",
                              ListCollisionObjects.Request())
        if res is None:
            raise RuntimeError(err)
        return {"world": list(res.world_ids),
                "attached": dict(zip(res.attached_ids, res.attached_links))}

    def clear_objects(self) -> ArmResult:
        return self._srv(Trigger, "arm/clear_collision_objects", Trigger.Request())

    def attach(self, object_id: str = "", *, obj: Optional[CollisionObject] = None,
               link: str = "", touch_links: Sequence[str] = ()) -> ArmResult:
        """Attach a scene object (or add `obj` and attach it) to the TCP/link."""
        req = AttachObject.Request()
        req.object_id = object_id
        if obj is not None:
            req.object = obj
        req.link = link
        req.touch_links = list(touch_links)
        return self._srv(AttachObject, "arm/attach_object", req)

    def detach(self, object_id: str = "", remove: bool = False) -> ArmResult:
        req = DetachObject.Request()
        req.object_id = object_id
        req.remove_from_scene = remove
        return self._srv(DetachObject, "arm/detach_object", req)

    # ---------------------------------------------------------------- constraints
    def set_path_constraints(self, joint=(), orientation=(), position=()) -> ArmResult:
        req = SetPathConstraints.Request()
        req.joint_constraints = list(joint)
        req.orientation_constraints = list(orientation)
        req.position_constraints = list(position)
        return self._srv(SetPathConstraints, "arm/set_path_constraints", req)

    def keep_orientation(self, tolerance_deg: float = 10.0, quat: Optional[Sequence[float]] = None,
                         link: str = "", frame: str = "",
                         z_tolerance_deg: Optional[float] = None) -> ArmResult:
        """
        Hold the tool orientation within +-tolerance for every following plan.

        Default orientation: the current one. z_tolerance_deg: separate (e.g. free)
        rotation about the tool axis.
        """
        import math

        if quat is None:
            p = self.current_pose()
            if p is None:
                return ArmResult(False, "NO_POSE", "no arm/state/current_pose received")
            o = p.pose.orientation
            quat = (o.x, o.y, o.z, o.w)
            frame = frame or p.header.frame_id
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        (oc.orientation.x, oc.orientation.y, oc.orientation.z,
         oc.orientation.w) = (float(v) for v in quat)
        tol = math.radians(tolerance_deg)
        oc.absolute_x_axis_tolerance = tol
        oc.absolute_y_axis_tolerance = tol
        oc.absolute_z_axis_tolerance = (math.radians(z_tolerance_deg)
                                        if z_tolerance_deg is not None else tol)
        oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        oc.weight = 1.0
        return self.set_path_constraints(orientation=[oc])

    def clear_path_constraints(self) -> ArmResult:
        return self._srv(ClearPathConstraints, "arm/clear_path_constraints",
                         ClearPathConstraints.Request())
