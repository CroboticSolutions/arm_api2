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
"""
Python client for arm_api2 (moveit2_iface).

Blocking by default; pass ``wait=False`` to get an :class:`ActionHandle` instead::

    from arm_api2_py import Arm, pose

    arm = Arm()                                   # own node, spun in a background thread
    arm.set_state("JOINT_TRAJ_CTL")
    arm.move_to_named("home")
    arm.move_relative(z=-0.10)                    # 10 cm back along the tool Z axis

    task = (arm.task()
            .move_named("ready")
            .move_pose(pose(0.30, 0.0, 0.20, rpy=(3.14159, 0, 0)), planner="pilz_PTP")
            .move_relative(z=0.05, linear=True)
            .gripper_close()
            .attach("box")
            .move_relative(z=-0.05)
            .detach("box"))
    plan = task.plan()                            # nothing moves; plan.segments for preview
    arm.execute_plan(plan.plan_id)                # or task.run() = plan + execute
"""

from arm_api2_py.client import ActionHandle, Arm, ArmResult, PlanResult
from arm_api2_py.task import Task, pose

__all__ = ["Arm", "ArmResult", "ActionHandle", "PlanResult", "Task", "pose"]
