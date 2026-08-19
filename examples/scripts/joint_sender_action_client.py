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

import threading
import rclpy
from arm_api2_msgs.action import MoveJoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node


class JointSenderActionClient(Node):

    def __init__(self):
        super().__init__("joint_sender_action_client")
        self._action_client = ActionClient(self, MoveJoint, "arm/move_to_joint")

        # new thread to handle the actual task without blocking the main thread
        self._execution_thread = threading.Thread(target=self._execute)
        self._execution_thread.start()

    def _send_goal(self, goal: JointState):
        goal_msg = MoveJoint.Goal()
        goal_msg.joint_state = goal

        self.get_logger().info("Waiting for action server...")

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        result = self._action_client.send_goal(goal_msg)

        self.get_logger().info("Result: {0}".format(result))
        rclpy.shutdown()

    def _execute(self):

        names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        positions = [-4.0, -2.067, 1.849, -1.428, -1.395, 2.045]
        goal = JointState()
        goal.name = names
        goal.position = positions

        self._send_goal(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = JointSenderActionClient()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
