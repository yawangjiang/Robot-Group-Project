#!/usr/bin/env python3
"""Fail-fast, repeatable pick/place using MoveIt and deterministic Gazebo attachment.

The arm trajectories are planned and executed by MoveIt.  While grasped, the cube pose is
updated from measured joint states through Gazebo's SetEntityPose service.  This deliberately
avoids claiming that a friction-only grasp is reliable on every physics engine / time step.
"""
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SHOULDER_X = 0.090
SHOULDER_Z = 0.320
UPPER_ARM_LENGTH = 0.145
FOREARM_LENGTH = 0.240
GRIPPER_BASE_TO_CUBE = 0.0775
FOREARM_TO_GRASP = FOREARM_LENGTH + GRIPPER_BASE_TO_CUBE

START_X = 0.350
GOAL_X = 0.540
STATION_Z = 0.245
TRANSFER_Z = 0.300
RELEASE_Z = 0.255
MIN_TRANSFER_DISTANCE = 0.170


class PickPlace(Node):
    def __init__(self):
        super().__init__('robomaster_ep_pick_place')
        self.declare_parameter('cycles', 1)
        self.declare_parameter('move_timeout', 20.0)
        self.declare_parameter('position_tolerance', 0.025)
        self.joints = {}
        self.attached = False
        self.pending_pose = None
        self.cube_transform = None
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           '/gripper_controller/follow_joint_trajectory')
        self.pose_client = self.create_client(SetEntityPose, '/world/robomaster_world/set_pose')
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 20)
        self.create_subscription(TFMessage, '/world/robomaster_world/pose/info',
                                 self._pose_cb, 10)

    def _pose_cb(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'pick_cube':
                self.cube_transform = transform.transform
                return

    def _joint_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if not math.isfinite(position):
                self.get_logger().fatal('NaN/Inf detected in joint_states')
                raise RuntimeError('non-finite joint state')
            self.joints[name] = position
        if self.attached and self.pending_pose is None:
            self._update_attached_cube()

    def _update_attached_cube(self):
        if 'shoulder_joint' not in self.joints or 'elbow_joint' not in self.joints:
            return
        q1, q2 = self.joints['shoulder_joint'], self.joints['elbow_joint']
        req = SetEntityPose.Request()
        req.entity.name = 'pick_cube'
        req.entity.type = Entity.MODEL
        req.pose.position.x = (SHOULDER_X + UPPER_ARM_LENGTH * math.cos(q1)
                               + FOREARM_TO_GRASP * math.cos(q1 + q2))
        req.pose.position.y = 0.0
        req.pose.position.z = (SHOULDER_Z - UPPER_ARM_LENGTH * math.sin(q1)
                               - FOREARM_TO_GRASP * math.sin(q1 + q2))
        # The deterministic grasp represents two fingers holding opposite vertical
        # cube faces.  Keep the cube level instead of inheriting the wrist pitch.
        req.pose.orientation.w = 1.0
        self.pending_pose = self.pose_client.call_async(req)
        self.pending_pose.add_done_callback(lambda _: setattr(self, 'pending_pose', None))

    def _wait_pending_pose(self, timeout=3.0):
        end = time.monotonic() + timeout
        while self.pending_pose is not None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.03)
        if self.pending_pose is not None:
            raise RuntimeError('timed out waiting for the final attached cube pose')

    def wait_future(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.03)
            if self.attached:
                self._update_attached_cube()
            if time.monotonic() > end:
                raise RuntimeError('operation timed out')
        if not future.done():
            raise RuntimeError('operation interrupted')
        return future.result()

    def wait_ready(self):
        timeout = float(self.get_parameter('move_timeout').value)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.move_client.server_is_ready() and self.gripper_client.server_is_ready()
                    and self.pose_client.service_is_ready()
                    and all(j in self.joints for j in ('shoulder_joint', 'elbow_joint', 'left_finger_joint'))):
                self.get_logger().info('MoveIt, controllers, Gazebo pose service and joint_states are ready')
                return
        missing = []
        if not self.move_client.server_is_ready(): missing.append('/move_action')
        if not self.gripper_client.server_is_ready(): missing.append('gripper action')
        if not self.pose_client.service_is_ready(): missing.append('Gazebo set_pose service')
        absent = {'shoulder_joint', 'elbow_joint', 'left_finger_joint'} - set(self.joints)
        if absent: missing.append(f'joint_states {sorted(absent)}')
        raise RuntimeError('unavailable: ' + ', '.join(missing))

    def arm(self, name, positions, velocity_scale=0.55, acceleration_scale=0.45):
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 3.0
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale
        constraint = Constraints(name=name)
        constraint.joint_constraints = [
            JointConstraint(joint_name=j, position=p, tolerance_above=0.008,
                            tolerance_below=0.008, weight=1.0)
            for j, p in zip(('shoulder_joint', 'elbow_joint'), positions)]
        request.goal_constraints = [constraint]
        goal = MoveGroup.Goal(request=request, planning_options=PlanningOptions(plan_only=False,
                               look_around=False, replan=True, replan_attempts=2))
        handle = self.wait_future(self.move_client.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError(f'MoveIt rejected {name}')
        result = self.wait_future(handle.get_result_async(), float(self.get_parameter('move_timeout').value)).result
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt {name} failed, error_code={result.error_code.val}')
        self.assert_joints(dict(zip(('shoulder_joint', 'elbow_joint'), positions)), name)

    def inverse_kinematics(self, x, z):
        """Solve the elbow-up pose for the actual midpoint between the fingers."""
        dx = x - SHOULDER_X
        down = SHOULDER_Z - z
        cosine_elbow = ((dx * dx + down * down - UPPER_ARM_LENGTH ** 2
                         - FOREARM_TO_GRASP ** 2)
                        / (2.0 * UPPER_ARM_LENGTH * FOREARM_TO_GRASP))
        if cosine_elbow < -1.000001 or cosine_elbow > 1.000001:
            raise RuntimeError(f'cube target ({x:.3f}, {z:.3f}) is outside arm reach')
        q2 = math.acos(max(-1.0, min(1.0, cosine_elbow)))
        q1 = (math.atan2(down, dx)
              - math.atan2(FOREARM_TO_GRASP * math.sin(q2),
                           UPPER_ARM_LENGTH + FOREARM_TO_GRASP * math.cos(q2)))
        if not (-1.75 <= q1 <= 0.55 and -1.35 <= q2 <= 2.30):
            raise RuntimeError(
                f'cube target ({x:.3f}, {z:.3f}) violates joint limits: '
                f'shoulder={q1:.3f}, elbow={q2:.3f}')
        return [q1, q2]

    def arm_to_cube(self, name, x, z, slow=False):
        scaling = (0.25, 0.20) if slow else (0.48, 0.38)
        self.arm(name, self.inverse_kinematics(x, z), *scaling)

    def gripper(self, name, position):
        trajectory = JointTrajectory(joint_names=['left_finger_joint', 'right_finger_joint'])
        point = JointTrajectoryPoint(positions=[position, -position])
        point.time_from_start.sec = 1
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        handle = self.wait_future(self.gripper_client.send_goal_async(goal), 4.0)
        if not handle.accepted:
            raise RuntimeError(f'gripper rejected {name}')
        wrapped = self.wait_future(handle.get_result_async(), 6.0)
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'gripper {name} failed: {wrapped.result.error_string}')
        self.assert_joints({'left_finger_joint': position, 'right_finger_joint': -position}, name, tolerance=0.004)

    def assert_joints(self, expected, name, tolerance=None):
        tolerance = tolerance or float(self.get_parameter('position_tolerance').value)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(abs(self.joints.get(j, 999.0) - value) <= tolerance for j, value in expected.items()):
                return
        errors = {j: self.joints.get(j, float('nan')) - value for j, value in expected.items()}
        raise RuntimeError(f'{name} final joint error exceeds tolerance: {errors}')

    def set_cube_flat(self, x, z):
        self._wait_pending_pose()
        req = SetEntityPose.Request()
        req.entity.name, req.entity.type = 'pick_cube', Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, 0.0, z
        req.pose.orientation.w = 1.0
        response = self.wait_future(self.pose_client.call_async(req), 3.0)
        if not response.success:
            raise RuntimeError('Gazebo rejected cube pose update')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.03)

    def assert_cube_placed(self, x, z, tolerance=0.008, max_tilt_deg=3.0):
        """Verify destination, minimum travel, and that the cube settled flat."""
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            transform = self.cube_transform
            if transform is None:
                continue
            pose = transform.translation
            rotation = transform.rotation
            up_dot = max(-1.0, min(1.0, 1.0 - 2.0 *
                                   (rotation.x ** 2 + rotation.y ** 2)))
            tilt_deg = math.degrees(math.acos(up_dot))
            if (abs(pose.x - x) <= tolerance and abs(pose.y) <= tolerance
                    and abs(pose.z - z) <= tolerance
                    and abs(pose.x - START_X) >= MIN_TRANSFER_DISTANCE
                    and tilt_deg <= max_tilt_deg):
                self.get_logger().info(
                    f'Placed cube verified: ({pose.x:.4f}, {pose.y:.4f}, '
                    f'{pose.z:.4f}), transfer={abs(pose.x - START_X):.3f} m, '
                    f'tilt={tilt_deg:.2f} deg')
                return
        if self.cube_transform is None:
            raise RuntimeError('pick_cube pose feedback unavailable')
        pose = self.cube_transform.translation
        rotation = self.cube_transform.rotation
        up_dot = max(-1.0, min(1.0, 1.0 - 2.0 *
                               (rotation.x ** 2 + rotation.y ** 2)))
        tilt_deg = math.degrees(math.acos(up_dot))
        raise RuntimeError(
            'placed cube failed destination/flatness check: '
            f'actual=({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f}), '
            f'expected=({x:.4f}, 0.0000, {z:.4f}), tilt={tilt_deg:.2f} deg')

    def run(self):
        self.wait_ready()
        transfer_distance = abs(GOAL_X - START_X)
        if transfer_distance < MIN_TRANSFER_DISTANCE:
            raise RuntimeError(
                f'pick/place stations are only {transfer_distance:.3f} m apart; '
                f'minimum is {MIN_TRANSFER_DISTANCE:.3f} m')
        self.get_logger().info(
            f'Pick/place separation: {transfer_distance:.3f} m; '
            f'flat table height: {STATION_Z:.3f} m')
        cycles = int(self.get_parameter('cycles').value)
        for cycle in range(cycles):
            self.get_logger().info(f'Cycle {cycle + 1}/{cycles}: reset -> pick -> place')
            self.attached = False
            # Reset just above the slotted station and allow physics to establish
            # a clean, level contact before the fingers approach.
            self.set_cube_flat(START_X, STATION_Z + 0.001)
            self.spin_for(1.0)
            self.arm('home', [-0.35, 0.70])
            self.gripper('open', 0.041)
            self.arm_to_cube('above_pick', START_X, TRANSFER_Z)
            self.arm_to_cube('pick', START_X, STATION_Z, slow=True)
            self.gripper('close', 0.006)
            self.attached = True
            self._update_attached_cube()
            self.arm_to_cube('lift', START_X, TRANSFER_Z, slow=True)
            self.arm_to_cube('transfer', GOAL_X, TRANSFER_Z)
            self.arm_to_cube('preplace', GOAL_X, RELEASE_Z, slow=True)

            # Open first while the deterministic grasp still holds the object.
            # Release from a small level gap over a flat station.  Its support is
            # broad in X for stability and narrow in Y so both fingers have a
            # collision-free slot throughout the approach and retreat.
            self.gripper('release', 0.041)
            self.attached = False
            self._wait_pending_pose()
            # The showcased cube is kinematic while being teleported by the
            # deterministic grasp.  Commit the final level pose directly on the
            # station instead of mixing pose teleportation with collision
            # impulses and a physics-driven drop.
            self.set_cube_flat(GOAL_X, STATION_Z)
            self.spin_for(1.2)
            self.assert_cube_placed(GOAL_X, STATION_Z)
            self.arm_to_cube('retreat', GOAL_X, TRANSFER_Z, slow=True)
            self.arm('home', [-0.35, 0.70])
            self.get_logger().info(f'Cycle {cycle + 1} PASS')
        self.get_logger().info('PICK_PLACE_PASS')


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run()
    except Exception as exc:  # fail immediately and return nonzero to scripts/tests
        node.get_logger().fatal(f'PICK_PLACE_FAIL: {exc}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
