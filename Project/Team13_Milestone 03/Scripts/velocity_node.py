#!/usr/bin/env python3
"""Milestone 03 ROS 2 node: forward and inverse velocity kinematics vs Gazebo (Team 13).

1. Inverse kinematics:  q0 = inverse_kinematics_func(X)   -> moves the arm to the start position.
2. Inverse velocity kinematics:  q_dot = inverse_velocity_kinematics(q0, V)   (constant, computed once).
   At the start, forward_velocity_kinematics(q0, q_dot) = V.
3. For T seconds every joint follows q_i(t) = q0_i + q_dot_i * t (the same inputs as the Simulink model),
   published at 30 Hz.
4. Gazebo velocity = change of /end_effector_pose over 0.3 s. It is compared with the FVK averaged over
   the same 0.3 s, evaluated (a) with the commanded q(t), q_dot and (b) with the joint angles Gazebo
   reports on /joint_states, differentiated over the same 0.3 s (this one does not include the
   joint controllers' delay, so it tests only the FVK equations).
3. Gazebo only reports the end-effector pose, so its velocity is measured by
   differentiating the pose: (p(t) - p(t - 0.3 s)) / 0.3 s, using Gazebo's own time stamps.

Topics:
    /Joint_N/command        std_msgs/Float64           publish (rad) -> Gazebo
    /ik/joint_angles        std_msgs/Float64MultiArray publish (rad) -> fk_node.py
    /ik/joint_velocities    std_msgs/Float64MultiArray publish (rad/s)
    /end_effector_pose      geometry_msgs/PoseStamped  subscribe (world frame)
    /joint_states           sensor_msgs/JointState     subscribe (Gazebo joint angles and velocities)
Uses Gazebo simulation time (use_sim_time), so a slow simulation does not change the result.

Usage (positions in mm, frame 0; velocities in mm/s; T in s):
    python3 velocity_node.py                                     # case 1: 150 120 250, v = 20 -10 15
    python3 velocity_node.py x y z vx vy vz T
    python3 velocity_node.py -180 100 200 -15 20 10 3            # case 2
    python3 velocity_node.py -100 -200 150 10 15 -20 3            # case 3
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from forward_kinematics import BASE_HEIGHT
from inverse_kinematics import fk_numeric, inverse_kinematics_func
from velocity_kinematics import forward_velocity_numeric, inverse_velocity_kinematics

JOINT_TOPICS = ['/Joint_1/command', '/Joint_2/command', '/Joint_3/command', '/Joint_4/command']
Q0 = [0.0, -0.5, 1.0, -0.5]    # IK initial guess (rad)
RATE = 30.0                    # Hz
WINDOW = 0.3                   # s: time window for differentiating the Gazebo pose
SKIP = 0.6                     # s: ignore the start of the motion (Gazebo speeds up)
STILL_TOL = 1e-4               # m
STILL_TIME = 1.0               # s
CLOSE_TOL = 1e-3               # m


class VelocityNode(Node):

    def __init__(self, X_mm, V_mm_s, duration):
        super().__init__('velocity_node',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.V = np.array(V_mm_s) / 1000.0
        self.T = duration
        X = np.array(X_mm) / 1000.0
        self.X_start = X
        log = self.get_logger()
        log.info('Start position, frame 0 (mm): x=%.3f y=%.3f z=%.3f' % tuple(X_mm))
        log.info('Desired EE velocity (mm/s):   vx=%.3f vy=%.3f vz=%.3f for %.1f s' % (*V_mm_s, duration))
        self.q_start = inverse_kinematics_func(Q0, X)                               # IK
        self.q = self.q_start.copy()
        self.q_dot_cmd = inverse_velocity_kinematics(self.q_start, self.V)          # IVK (constant)
        self.q_dot = np.zeros(4)                                                    # published: 0 until moving
        self.v_fvk = np.zeros(3)
        v0 = forward_velocity_numeric(self.q_start, self.q_dot_cmd)                 # FVK at the start
        log.info('IK start joint angles (deg):  %.3f %.3f %.3f %.3f' % tuple(np.degrees(self.q_start)))
        log.info('IVK joint velocities (deg/s): %.3f %.3f %.3f %.3f' % tuple(np.degrees(self.q_dot_cmd)))
        log.info('FVK at the start (mm/s): vx=%.3f vy=%.3f vz=%.3f  (= desired)' % tuple(v0[0:3] * 1000))

        self.joint_pubs = [self.create_publisher(Float64, t, 10) for t in JOINT_TOPICS]
        self.angles_pub = self.create_publisher(Float64MultiArray, '/ik/joint_angles', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/ik/joint_velocities', 10)
        self.create_subscription(PoseStamped, '/end_effector_pose', self.on_pose, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.fvk_cmd_hist = []     # (time, FVK with the commanded q, q_dot)
        self.js_hist = []          # (time, Gazebo joint angles from /joint_states)
        self.create_timer(1.0 / RATE, self.on_timer)

        self.state = 'go_to_start'
        self.t_start = None
        self.t_last = None
        self.poses = []            # (gazebo time, position) during the motion
        self.measured = []         # (gazebo velocity, FVK velocity) after SKIP
        self.last_log = -1e9
        self.last_pose = None
        self.still_since = None
        self.done = False
        log.info('Moving to the start position ...')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self):
        for pub, angle in zip(self.joint_pubs, self.q):
            pub.publish(Float64(data=float(angle)))
        self.angles_pub.publish(Float64MultiArray(data=[float(v) for v in self.q]))
        self.vel_pub.publish(Float64MultiArray(data=[float(v) for v in self.q_dot]))

    def expected_world(self):
        x, y, z = fk_numeric(self.q)
        return np.array([x, y, z + BASE_HEIGHT])

    def on_timer(self):
        t = self.now()
        if self.state == 'moving':
            self.q_dot = self.q_dot_cmd
            self.q = self.q_start + self.q_dot_cmd * (t - self.t_start)            # q(t) = q0 + q_dot * t
            self.v_fvk = forward_velocity_numeric(self.q, self.q_dot)[0:3]         # FVK at the same q
            self.fvk_cmd_hist.append((t, self.v_fvk))
            if t - self.t_start >= self.T:
                self.q_dot = np.zeros(4)
                self.state = 'stopping'
                self.still_since = None
                self.get_logger().info('Motion finished, waiting for the arm to stop ...')
        self.publish()

    def on_joints(self, msg):
        names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4']
        if self.state != 'moving' or not all(n in msg.name for n in names):
            return
        q = np.array([msg.position[msg.name.index(n)] for n in names])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.js_hist.append((stamp, q))

    def fvk_from_joint_states(self, t0, t1):
        """FVK with the Gazebo joint angles: q at the middle of [t0, t1], q_dot = change over [t0, t1]."""
        if len(self.js_hist) < 2:
            return None
        ta, qa = min(self.js_hist, key=lambda h: abs(h[0] - t0))   # samples nearest to the window ends
        tb, qb = min(self.js_hist, key=lambda h: abs(h[0] - t1))
        if abs(ta - t0) > 0.05 or abs(tb - t1) > 0.05 or tb <= ta:
            return None
        return forward_velocity_numeric((qa + qb) / 2, (qb - qa) / (tb - ta))[0:3]

    @staticmethod
    def window_mean(hist, t0, t1):
        vals = [v for t, v in hist if t0 <= t <= t1]
        return np.mean(vals, axis=0) if vals else None

    def on_pose(self, msg):
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_pose is None or np.linalg.norm(pos - self.last_pose) > STILL_TOL or self.still_since is None:
            self.still_since = stamp
        self.last_pose = pos
        settled = (stamp - self.still_since >= STILL_TIME
                   and np.linalg.norm(pos - self.expected_world()) < CLOSE_TOL)

        if self.state == 'go_to_start' and settled:
            err = (pos - self.expected_world()) * 1000
            self.get_logger().info('At start. Gazebo - FK position error (mm): |e|=%.3f' % np.linalg.norm(err))
            self.state = 'moving'
            self.t_start = self.now()
            self.get_logger().info('Moving: q(t) = q0 + q_dot * t ...')

        elif self.state == 'moving':
            self.poses.append((stamp, pos))
            # Gazebo velocity: compare with the pose WINDOW seconds earlier
            old = [(ts, ps) for ts, ps in self.poses if stamp - ts >= WINDOW]
            if not old or self.now() - self.t_start < SKIP:
                return
            ts, ps = old[-1]
            v_gazebo = (pos - ps) / (stamp - ts)
            v_fvk = self.window_mean(self.fvk_cmd_hist, ts, stamp)
            if v_fvk is None:
                return
            v_js = self.fvk_from_joint_states(ts, stamp)
            self.measured.append((v_gazebo, v_fvk, v_js))
            if self.now() - self.last_log >= 0.5:
                self.last_log = self.now()
                js_txt = '' if v_js is None else '  FVK(joint_states) [%.2f %.2f %.2f]' % tuple(v_js * 1000)
                self.get_logger().info(
                    'v (mm/s)  Gazebo [%.2f %.2f %.2f]  FVK(commanded) [%.2f %.2f %.2f]%s'
                    % (*(v_gazebo * 1000), *(v_fvk * 1000), js_txt))

        elif self.state == 'stopping' and settled and not self.done:
            log = self.get_logger()
            if self.measured:
                vg = np.mean([m[0] for m in self.measured], axis=0)
                vf = np.mean([m[1] for m in self.measured], axis=0)
                diffs = [np.linalg.norm(m[0] - m[1]) for m in self.measured]
                log.info('===== Velocity results (steady part of the motion) =====')
                log.info('Mean Gazebo         v (mm/s): vx=%.3f vy=%.3f vz=%.3f' % tuple(vg * 1000))
                log.info('Mean FVK(commanded) v (mm/s): vx=%.3f vy=%.3f vz=%.3f' % tuple(vf * 1000))
                log.info('|Gazebo - FVK(commanded)|:   mean %.3f mm/s, max %.3f mm/s (|v| = %.1f mm/s)'
                         % (np.mean(diffs) * 1000, np.max(diffs) * 1000, np.linalg.norm(self.V) * 1000))
                js = [m for m in self.measured if m[2] is not None]
                if js:
                    d2 = [np.linalg.norm(m[0] - m[2]) for m in js]
                    log.info('|Gazebo - FVK(joint_states)|: mean %.3f mm/s, max %.3f mm/s'
                             % (np.mean(d2) * 1000, np.max(d2) * 1000))
                else:
                    log.info('No /joint_states received, so only the commanded comparison is shown.')
            fk_end = self.expected_world()
            log.info('End position Gazebo (world, mm):     x=%.3f y=%.3f z=%.3f' % tuple(pos * 1000))
            log.info('End position FK of q(T) (world, mm): x=%.3f y=%.3f z=%.3f   |e| = %.3f mm'
                     % (*(fk_end * 1000), np.linalg.norm(pos - fk_end) * 1000))
            log.info('Final joint angles (deg): %.3f %.3f %.3f %.3f' % tuple(np.degrees(self.q)))
            self.done = True


def main():
    if len(sys.argv) >= 8:
        args = [float(v) for v in sys.argv[1:8]]
        X_mm, V_mm_s, T = args[0:3], args[3:6], args[6]
    else:
        X_mm, V_mm_s, T = [150.0, 120.0, 250.0], [20.0, -10.0, 15.0], 3.0
    rclpy.init()
    node = VelocityNode(X_mm, V_mm_s, T)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
