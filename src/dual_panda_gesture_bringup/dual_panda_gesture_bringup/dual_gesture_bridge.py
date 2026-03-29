#!/usr/bin/env python3
"""
Dual Gesture-to-JointTrajectory Bridge Node

Drives TWO Franka Panda arms via JointTrajectory (7 joints each).
Drives grippers by publishing finger joint positions on /joint_states
(works with mock hardware — no separate gripper controller needed).
Publishes /clock for sim_time support.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock


# 7 arm joints per arm (what the arm controller expects)
LEFT_JOINTS = [
    'left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3',
    'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6',
    'left_panda_joint7',
]
RIGHT_JOINTS = [
    'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3',
    'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6',
    'right_panda_joint7',
]

# Finger joint names for direct JointState publishing
LEFT_FINGER_JOINTS = ['left_panda_finger_joint1', 'left_panda_finger_joint2']
RIGHT_FINGER_JOINTS = ['right_panda_finger_joint1', 'right_panda_finger_joint2']

HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

J1_RANGE = 1.2
J2_RANGE = 0.6
J4_RANGE = 0.8
J7_RANGE = 1.5

FINGER_OPEN = 0.04
FINGER_CLOSED = 0.0


class ArmController:
    def __init__(self, side, node, cb_group, params):
        self.side = side
        self.node = node
        self.logger = node.get_logger()
        self.joint_names = LEFT_JOINTS if side == 'left' else RIGHT_JOINTS
        self.finger_joints = LEFT_FINGER_JOINTS if side == 'left' else RIGHT_FINGER_JOINTS

        self.ws_x_min = params[f'{side}_ws_x_min']
        self.ws_x_max = params[f'{side}_ws_x_max']
        self.ws_z_min = params[f'{side}_ws_z_min']
        self.ws_z_max = params[f'{side}_ws_z_max']

        self.target_x = (self.ws_x_min + self.ws_x_max) / 2.0
        self.target_z = (self.ws_z_min + self.ws_z_max) / 2.0
        self.target_rotation = 0.0
        self.emergency_active = False
        self.gripper_closed = False
        self._prev_gripper_state = None
        self._lock = threading.Lock()
        self._tick_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(
            Pose, f'/{side}/gesture/target_pose',
            self._pose_cb, qos, callback_group=cb_group)
        node.create_subscription(
            Bool, f'/{side}/gesture/gripper_command',
            self._gripper_cb, qos, callback_group=cb_group)
        node.create_subscription(
            Bool, f'/{side}/gesture/emergency_stop',
            self._estop_cb, qos, callback_group=cb_group)
        node.create_subscription(
            Float64, f'/{side}/gesture/hand_rotation',
            self._rotation_cb, qos, callback_group=cb_group)

        # Arm trajectory publisher (7 joints only)
        sys_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.traj_pub = node.create_publisher(
            JointTrajectory,
            f'/{side}_arm_controller/joint_trajectory',
            sys_qos,
        )

        self.logger.info(f'{side.upper()} arm + gripper ready')

    def compute_and_publish(self):
        if self.emergency_active:
            return

        with self._lock:
            tx, tz = self.target_x, self.target_z
            rot = self.target_rotation

        nx = (tx - self.ws_x_min) / (self.ws_x_max - self.ws_x_min)
        nz = (tz - self.ws_z_min) / (self.ws_z_max - self.ws_z_min)
        nx = max(0.0, min(1.0, nx))
        nz = max(0.0, min(1.0, nz))

        self._tick_count += 1
        if self._tick_count % 10 == 1:
            self.logger.info(
                f'{self.side.upper()} nx={nx:.2f} nz={nz:.2f} '
                f'rot={rot:.2f} '
                f'gripper={"CLOSED" if self.gripper_closed else "OPEN"}')

        j1 = HOME[0] + (nx - 0.5) * 2.0 * J1_RANGE
        j2 = HOME[1] + (0.5 - nz) * 2.0 * J2_RANGE
        j3 = HOME[2]
        j4 = HOME[3] + (nz - 0.5) * 2.0 * J4_RANGE
        j5 = HOME[4]
        j6 = HOME[5]
        j7 = HOME[6] + rot * J7_RANGE

        positions = [j1, j2, j3, j4, j5, j6, j7]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        traj.points = [point]
        self.traj_pub.publish(traj)

    def get_finger_position(self):
        with self._lock:
            grip = self.gripper_closed
        return FINGER_CLOSED if grip else FINGER_OPEN

    def _pose_cb(self, msg: Pose):
        with self._lock:
            self.target_x = max(self.ws_x_min, min(self.ws_x_max, msg.position.x))
            self.target_z = max(self.ws_z_min, min(self.ws_z_max, msg.position.z))

    def _gripper_cb(self, msg: Bool):
        close = msg.data
        if close != self._prev_gripper_state:
            self._prev_gripper_state = close
            with self._lock:
                self.gripper_closed = close
            self.logger.info(
                f'{self.side.upper()} Gripper: '
                f'{"CLOSE" if close else "OPEN"}')

    def _rotation_cb(self, msg: Float64):
        with self._lock:
            self.target_rotation = msg.data

    def _estop_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.logger.warn(
                f'{self.side.upper()} EMERGENCY STOP activated!')
            self.emergency_active = True
        elif not msg.data and self.emergency_active:
            self.logger.info(
                f'{self.side.upper()} Emergency stop released.')
            self.emergency_active = False


class DualGestureBridge(Node):
    def __init__(self):
        super().__init__('dual_gesture_bridge')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, False)
        ])

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('velocity_gain', 1.5)
        self.declare_parameter('deadzone', 0.005)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('servo_publish_rate', 30.0)
        for side in ('left', 'right'):
            self.declare_parameter(f'{side}_planning_frame', 'world')
            self.declare_parameter(f'{side}_ws_x_min',
                                   -1.9 if side == 'left' else 1.1)
            self.declare_parameter(f'{side}_ws_x_max',
                                   -1.1 if side == 'left' else 1.9)
            self.declare_parameter(f'{side}_ws_z_min', 0.1)
            self.declare_parameter(f'{side}_ws_z_max', 0.7)
            self.declare_parameter(f'{side}_ws_y_fixed', 0.4)

        publish_rate = self.get_parameter('publish_rate').value

        params = {}
        for side in ('left', 'right'):
            for key in ('ws_x_min', 'ws_x_max', 'ws_z_min',
                        'ws_z_max', 'ws_y_fixed'):
                params[f'{side}_{key}'] = self.get_parameter(
                    f'{side}_{key}').value

        cb_group = ReentrantCallbackGroup()

        # Publish /clock
        self._clock_pub = self.create_publisher(Clock, '/clock', 10)
        self._start_time = time.monotonic()
        self.create_timer(0.01, self._publish_clock,
                          callback_group=cb_group)

        # Gripper JointState publisher — directly updates robot model
        self._gripper_js_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # Arm controllers
        self.arms = {}
        for side in ('left', 'right'):
            self.arms[side] = ArmController(side, self, cb_group, params)

        # Arm trajectory timer (5Hz)
        period = 1.0 / publish_rate
        self.create_timer(period, self._tick_arms, callback_group=cb_group)

        # Gripper JointState timer (10Hz)
        self.create_timer(0.1, self._tick_grippers, callback_group=cb_group)

        self.get_logger().info(
            f'DualGestureBridge ready — arms at {publish_rate} Hz, '
            f'grippers at 10 Hz (via /joint_states), /clock at 100 Hz.')

    def _publish_clock(self):
        elapsed = time.monotonic() - self._start_time
        msg = Clock()
        msg.clock.sec = int(elapsed)
        msg.clock.nanosec = int((elapsed % 1.0) * 1e9)
        self._clock_pub.publish(msg)

    def _tick_arms(self):
        for arm in self.arms.values():
            arm.compute_and_publish()

    def _tick_grippers(self):
        """Publish finger joint positions directly on /joint_states.

        This works because robot_state_publisher merges all /joint_states
        messages. The finger positions update TF and the RViz robot model
        shows the gripper opening/closing.
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        all_finger_joints = []
        all_finger_positions = []

        for arm in self.arms.values():
            val = arm.get_finger_position()
            for jname in arm.finger_joints:
                all_finger_joints.append(jname)
                all_finger_positions.append(val)

        js.name = all_finger_joints
        js.position = all_finger_positions
        self._gripper_js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DualGestureBridge()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
