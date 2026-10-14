#!/usr/bin/env python3
"""Milestone 02 ROS 2 node: forward kinematics vs Gazebo (Team 13).

Publishes the joint angles to the robot in Gazebo and subscribes to the end-effector
pose, then compares it with our DH forward kinematics (forward_kinematics.py).

Interface (official my_robot_gazebo package, launch/spawn_gazebo.launch.py):
    /Joint_1/command ... /Joint_4/command   std_msgs/Float64          (we publish, rad)
    /end_effector_pose                      geometry_msgs/PoseStamped (we subscribe, world frame)

Our frame 0 is on top of the base; Gazebo's world frame is on the ground 0.0438 m below,
so   z_world = z_frame0 + 0.0438.

Usage (after `source ~/ros2_ws/install/setup.bash` and launching spawn_gazebo.launch.py):
    python3 fk_gazebo_node.py                 # test case B: 30 20 -40 25 deg
    python3 fk_gazebo_node.py 0 0 0 0         # test case A: all zeros
    python3 fk_gazebo_node.py 10 -30 45 15    # any 4 angles in degrees
"""
import math
import sys

import rclpy
import sympy as sp
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64

from forward_kinematics import BASE_HEIGHT, forward_kinematics_func, q1, q2, q3, q4

JOINT_TOPICS = ['/Joint_1/command', '/Joint_2/command', '/Joint_3/command', '/Joint_4/command']
EE_TOPIC = '/end_effector_pose'
SETTLE_TOL = 1e-4      # m: pose change below this counts as "not moving"
SETTLE_TIME = 1.5      # s: must stay still this long before comparing
TIMEOUT = 20.0         # s: give up waiting after this


class FKGazeboNode(Node):

    def __init__(self, q_deg):
        super().__init__('fk_gazebo_node')
        self.q = [math.radians(v) for v in q_deg]

        # Symbolic FK from our DH table, turned into a fast numeric function
        X = forward_kinematics_func()
        self.fk = sp.lambdify((q1, q2, q3, q4), list(X), 'math')
        x, y, z = self.fk(*self.q)
        self.expected_world = (x, y, z + BASE_HEIGHT)

        self.pubs = [self.create_publisher(Float64, t, 10) for t in JOINT_TOPICS]
        self.create_subscription(PoseStamped, EE_TOPIC, self.on_pose, 10)

        self.last_pose = None
        self.still_since = None
        self.start = self.get_clock().now()
        self.done = False

        # Re-publish at 10 Hz: avoids the ROS 2 discovery race noted in the package README
        self.create_timer(0.1, self.publish_angles)

        self.get_logger().info(f'Commanding q = {q_deg} deg')
        self.get_logger().info('FK expected EE (world frame, m): x=%.5f y=%.5f z=%.5f'
                               % self.expected_world)

    def publish_angles(self):
        for pub, angle in zip(self.pubs, self.q):
            pub.publish(Float64(data=angle))
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed > TIMEOUT and not self.done:
            self.get_logger().error(f'No settled pose on {EE_TOPIC} after {TIMEOUT:.0f} s. '
                                    'Is spawn_gazebo.launch.py running?')
            self.done = True

    def on_pose(self, msg):
        p = msg.pose.position
        pose = (p.x, p.y, p.z)
        now = self.get_clock().now().nanoseconds * 1e-9
        moving = self.last_pose is None or math.dist(pose, self.last_pose) > SETTLE_TOL
        self.last_pose = pose
        if moving:
            self.still_since = now
            return
        if self.done or now - self.still_since < SETTLE_TIME:
            return

        err = [m - e for m, e in zip(pose, self.expected_world)]
        self.get_logger().info('Gazebo  EE (world, m): x=%.5f y=%.5f z=%.5f' % pose)
        self.get_logger().info('FK      EE (world, m): x=%.5f y=%.5f z=%.5f' % self.expected_world)
        self.get_logger().info('Error (mm): dx=%.3f dy=%.3f dz=%.3f |e|=%.3f'
                               % (err[0] * 1e3, err[1] * 1e3, err[2] * 1e3,
                                  math.dist(pose, self.expected_world) * 1e3))
        self.done = True


def main():
    q_deg = [float(v) for v in sys.argv[1:5]] if len(sys.argv) >= 5 else [30.0, 20.0, -40.0, 25.0]
    rclpy.init()
    node = FKGazeboNode(q_deg)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
