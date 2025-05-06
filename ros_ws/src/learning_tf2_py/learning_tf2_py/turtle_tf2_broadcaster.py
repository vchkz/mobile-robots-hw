# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import TransformStamped, Twist

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        self.turtlename = self.declare_parameter(
            'turtlename', 'turtle').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription

        self.cmd_pub = self.create_publisher(Twist, f'/{self.turtlename}/cmd_vel', 10)
        self.cmd_sub = self.create_subscription(
            Twist,
            f'/{self.turtlename}/cmd_vel',
            self.handle_cmd_vel,
            10)

        self.zigzag_active = False
        self.zigzag_amplitude = 5.0
        self.zigzag_period = 0.5
        self.zigzag_direction = 1
        self.last_linear_x = 0.0
        self.zigzag_timer = None

    def handle_cmd_vel(self, msg):
        if abs(abs(msg.angular.z) - self.zigzag_amplitude) < 0.001:
            return

        if msg.linear.x > 0 and not self.zigzag_active:
            self.zigzag_active = True
            self.last_linear_x = msg.linear.x
            self.zigzag_timer = self.create_timer(self.zigzag_period / 2, self.update_zigzag)
            self.update_zigzag()
        elif msg.linear.x <= 0 and self.zigzag_active:
            self.zigzag_active = False
            if self.zigzag_timer is not None:
                self.destroy_timer(self.zigzag_timer)
                self.zigzag_timer = None
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_pub.publish(stop_msg)
        elif self.zigzag_active:
            self.last_linear_x = msg.linear.x

    def update_zigzag(self):
        if not self.zigzag_active:
            return
        self.zigzag_direction *= -1
        cmd_msg = Twist()
        cmd_msg.linear.x = self.last_linear_x
        cmd_msg.angular.z = self.zigzag_amplitude * self.zigzag_direction
        self.cmd_pub.publish(cmd_msg)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()