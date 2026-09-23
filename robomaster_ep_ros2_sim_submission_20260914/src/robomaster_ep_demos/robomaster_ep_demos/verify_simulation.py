#!/usr/bin/env python3
"""Verify controller activation, finite joint states and action availability."""
import math
import sys
import time
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Verifier(Node):
    def __init__(self):
        super().__init__('robomaster_ep_verifier')
        self.msg = None
        self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
    def cb(self, msg): self.msg = msg
    def run(self):
        end = time.monotonic() + 20.0
        required = {'joint_state_broadcaster', 'arm_controller', 'gripper_controller'}
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.client.service_is_ready() and self.arm.server_is_ready()
                    and self.gripper.server_is_ready() and self.msg):
                break
        else: raise RuntimeError('controllers/actions/joint_states not ready')
        # Let activation settle, then use one request.  Repeated abandoned short-timeout
        # service requests trigger an rclcpp Humble response race on this aarch64 image.
        settle = time.monotonic() + 2.0
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        future = self.client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('list_controllers timed out')
        states = {c.name: c.state for c in future.result().controller}
        if any(states.get(name) != 'active' for name in required):
            raise RuntimeError(f'controller states: {states}')
        expected = {'shoulder_joint', 'elbow_joint', 'left_finger_joint', 'right_finger_joint'}
        if not expected.issubset(self.msg.name): raise RuntimeError(f'missing joints: {expected - set(self.msg.name)}')
        if not all(math.isfinite(v) for v in self.msg.position): raise RuntimeError('non-finite joint state')
        self.get_logger().info('SIMULATION_READY_PASS')

def main():
    rclpy.init(); node = Verifier()
    try: node.run()
    except Exception as exc:
        node.get_logger().fatal(f'SIMULATION_READY_FAIL: {exc}'); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
