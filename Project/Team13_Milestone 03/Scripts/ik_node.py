#!/usr/bin/env python3
"""Milestone 03 ROS 2 node: inverse position kinematics (Team 13).

Runs inverse_kinematics_func (Newton-Raphson with inverse_jacobian_matrix) for a desired
end-effector position and publishes the joint angles:
    /Joint_1/command ... /Joint_4/command   std_msgs/Float64          -> robot in Gazebo
    /ik/joint_angles                        std_msgs/Float64MultiArray -> fk_node.py (M2 FK)

The desired position is given in mm in frame 0 (top of the base). Gazebo's world frame is
0.0438 m lower, so the world target is z + 43.8 mm.

Usage (Gazebo running with spawn_gazebo.launch.py, fk_node.py running in another terminal):
    python3 ik_node.py                       # case 1: x=150 y=120 z=250 mm
    python3 ik_node.py -180 100 200           # case 2
    python3 ik_node.py -100 -200 150          # case 3
Stop with Ctrl+C after fk_node.py prints the comparison.
"""
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from forward_kinematics import BASE_HEIGHT
from inverse_kinematics import fk_numeric, inverse_kinematics_func

JOINT_TOPICS = ['/Joint_1/command', '/Joint_2/command', '/Joint_3/command', '/Joint_4/command']
ANGLES_TOPIC = '/ik/joint_angles'
Q0 = [0.0, -0.5, 1.0, -0.5]         # initial guess (rad): a bent pose, away from the straight-up singularity


class IKNode(Node):

    def __init__(self, X_mm):
        super().__init__('ik_node')
        X = np.array(X_mm) / 1000.0
        self.get_logger().info('Desired EE position, frame 0 (mm): x=%.3f y=%.3f z=%.3f' % tuple(X_mm))
        self.get_logger().info('Desired EE position, world (mm):   x=%.3f y=%.3f z=%.3f'
                               % (X_mm[0], X_mm[1], X_mm[2] + BASE_HEIGHT * 1000))

        self.q = inverse_kinematics_func(Q0, X)
        self.get_logger().info('IK joint angles (deg): q1=%.3f q2=%.3f q3=%.3f q4=%.3f'
                               % tuple(np.degrees(self.q)))
        check = fk_numeric(self.q) * 1000
        self.get_logger().info('FK of these angles, frame 0 (mm): x=%.3f y=%.3f z=%.3f' % tuple(check))

        self.joint_pubs = [self.create_publisher(Float64, t, 10) for t in JOINT_TOPICS]
        self.angles_pub = self.create_publisher(Float64MultiArray, ANGLES_TOPIC, 10)
        # re-publish at 10 Hz so late subscribers (Gazebo driver, fk_node) still get the angles
        self.create_timer(0.1, self.publish_angles)

    def publish_angles(self):
        for pub, angle in zip(self.joint_pubs, self.q):
            pub.publish(Float64(data=float(angle)))
        self.angles_pub.publish(Float64MultiArray(data=[float(v) for v in self.q]))


def main():
    if len(sys.argv) >= 4:
        X_mm = [float(v) for v in sys.argv[1:4]]
    else:
        X_mm = [150.0, 120.0, 250.0]
    rclpy.init()
    node = IKNode(X_mm)
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
