#!/usr/bin/env python3
"""Milestone 03 ROS 2 node: forward kinematics subscriber (Team 13).

Subscribes to the joint angles published by ik_node.py / velocity_node.py, computes the
end-effector position with forward_kinematics_func (Milestone 02) and compares it with the
pose Gazebo reports:
    /ik/joint_angles     std_msgs/Float64MultiArray   (subscribe, rad)
    /end_effector_pose   geometry_msgs/PoseStamped    (subscribe, world frame)
    /fk/ee_position      std_msgs/Float64MultiArray   (publish, world frame, m)

Usage:  python3 fk_node.py      (start it before ik_node.py; stop with Ctrl+C)
"""
import math

import rclpy
import sympy as sp
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from forward_kinematics import BASE_HEIGHT, forward_kinematics_func, q1, q2, q3, q4

ANGLES_TOPIC = '/ik/joint_angles'
EE_TOPIC = '/end_effector_pose'
FK_TOPIC = '/fk/ee_position'
STILL_TOL = 1e-4      # m: pose change below this counts as "not moving"
STILL_TIME = 1.5      # s: arm must stay still this long before comparing
CLOSE_TOL = 1e-3      # m: compare as soon as Gazebo is within 1 mm of the FK and still
GIVE_UP_TIME = 5.0    # s: if still this long but not close, compare anyway (shows the error)


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')
        X = forward_kinematics_func()              # Milestone 02 symbolic FK (about 20 s)
        self.fk = sp.lambdify((q1, q2, q3, q4), list(X), 'math')
        self.create_subscription(Float64MultiArray, ANGLES_TOPIC, self.on_angles, 10)
        self.create_subscription(PoseStamped, EE_TOPIC, self.on_pose, 10)
        self.fk_pub = self.create_publisher(Float64MultiArray, FK_TOPIC, 10)
        self.q = None
        self.fk_world = None
        self.reported = False
        self.last_pose = None
        self.still_since = None
        self.last_log = -1e9
        self.get_logger().info(f'Ready: waiting for joint angles on {ANGLES_TOPIC}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_angles(self, msg):
        q = list(msg.data)
        changed = self.q is None or max(abs(a - b) for a, b in zip(q, self.q)) > 1e-9
        self.q = q
        x, y, z = self.fk(*q)
        self.fk_world = (x, y, z + BASE_HEIGHT)
        self.fk_pub.publish(Float64MultiArray(data=list(self.fk_world)))
        if changed:
            self.reported = False
            self.still_since = self.now()      # start the "still" timer from the new command
            if self.now() - self.last_log > 1.0:          # at most one log line per second
                self.last_log = self.now()
                self.get_logger().info('q (deg) = [%.3f %.3f %.3f %.3f] -> FK EE world (mm): '
                                       'x=%.3f y=%.3f z=%.3f'
                                       % (*[math.degrees(v) for v in q],
                                          *[v * 1000 for v in self.fk_world]))

    def on_pose(self, msg):
        p = msg.pose.position
        pose = (p.x, p.y, p.z)
        t = self.now()
        if self.last_pose is None or math.dist(pose, self.last_pose) > STILL_TOL:
            self.still_since = t
        self.last_pose = pose
        if self.fk_world is None or self.reported:
            return
        still_for = t - self.still_since
        close = math.dist(pose, self.fk_world) < CLOSE_TOL
        if (close and still_for >= STILL_TIME) or still_for >= GIVE_UP_TIME:
            err = [g - f for g, f in zip(pose, self.fk_world)]
            self.get_logger().info('Gazebo EE world (mm): x=%.3f y=%.3f z=%.3f' % tuple(v * 1000 for v in pose))
            self.get_logger().info('FK     EE world (mm): x=%.3f y=%.3f z=%.3f'
                                   % tuple(v * 1000 for v in self.fk_world))
            self.get_logger().info('Error Gazebo - FK (mm): dx=%.3f dy=%.3f dz=%.3f |e|=%.3f'
                                   % (*[e * 1000 for e in err], math.dist(pose, self.fk_world) * 1000))
            self.reported = True


def main():
    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
