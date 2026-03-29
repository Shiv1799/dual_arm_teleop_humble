#!/usr/bin/env python3
"""
Dual Hand Gesture Tracker Node with RViz Visualization

Opens the webcam, detects TWO hand landmarks via MediaPipe Hands,
identifies left vs right hand using MediaPipe handedness classification,
and publishes per-arm target pose / gripper / emergency-stop commands.

Left hand  -> /left/gesture/*  topics -> left Panda arm
Right hand -> /right/gesture/* topics -> right Panda arm
"""

import math

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, ColorRGBA, Float64, Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

_HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]


class DualGestureTracker(Node):
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20
    INDEX_MCP = 5
    MIDDLE_MCP = 9
    RING_MCP = 13
    PINKY_MCP = 17

    def __init__(self):
        super().__init__('dual_gesture_tracker')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('smoothing_factor', 0.4)
        self.declare_parameter('pinch_threshold', 0.07)
        self.declare_parameter('pinch_hysteresis', 0.02)
        self.declare_parameter('fist_threshold', 0.10)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('left_ws_x_min', -1.9)
        self.declare_parameter('left_ws_x_max', -1.1)
        self.declare_parameter('left_ws_z_min', 0.1)
        self.declare_parameter('left_ws_z_max', 0.7)
        self.declare_parameter('left_ws_y_fixed', 0.4)
        self.declare_parameter('right_ws_x_min', 1.1)
        self.declare_parameter('right_ws_x_max', 1.9)
        self.declare_parameter('right_ws_z_min', 0.1)
        self.declare_parameter('right_ws_z_max', 0.7)
        self.declare_parameter('right_ws_y_fixed', 0.4)

        self.camera_id = self.get_parameter('camera_id').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.pinch_thresh = self.get_parameter('pinch_threshold').value
        self.pinch_hyst = self.get_parameter('pinch_hysteresis').value
        self.fist_thresh = self.get_parameter('fist_threshold').value
        publish_rate = self.get_parameter('publish_rate').value

        self.ws = {}
        for side in ('left', 'right'):
            self.ws[side] = {
                'x_min': self.get_parameter(f'{side}_ws_x_min').value,
                'x_max': self.get_parameter(f'{side}_ws_x_max').value,
                'z_min': self.get_parameter(f'{side}_ws_z_min').value,
                'z_max': self.get_parameter(f'{side}_ws_z_max').value,
                'y_fixed': self.get_parameter(f'{side}_ws_y_fixed').value,
            }

        # Publishers per arm
        self.pubs = {}
        for side in ('left', 'right'):
            self.pubs[side] = {
                'pose': self.create_publisher(
                    Pose, f'/{side}/gesture/target_pose', 10),
                'gripper': self.create_publisher(
                    Bool, f'/{side}/gesture/gripper_command', 10),
                'estop': self.create_publisher(
                    Bool, f'/{side}/gesture/emergency_stop', 10),
                'rotation': self.create_publisher(
                    Float64, f'/{side}/gesture/hand_rotation', 10),
            }

        # RViz visualization
        self.hand_marker_pub = self.create_publisher(
            MarkerArray, '/gesture/hand_markers', 10)
        self.target_marker_pub = self.create_publisher(
            MarkerArray, '/gesture/target_markers', 10)
        self.camera_pub = self.create_publisher(
            Image, '/gesture/camera_feed', 10)
        self.cv_bridge = CvBridge()

        # MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

        # OpenCV
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open webcam (device {self.camera_id}).')

        self.gui_available = True
        try:
            cv2.namedWindow('__test__')
            cv2.destroyWindow('__test__')
        except cv2.error:
            self.gui_available = False
            self.get_logger().warn('No GUI display — headless mode.')

        # Per-hand state
        self.state = {}
        for side in ('left', 'right'):
            self.state[side] = {
                'smooth_x': 0.5,
                'smooth_y': 0.5,
                'smooth_rot': 0.0,
                'gripper_closed': False,
                'emergency': False,
                'last_robot_pos': None,
            }

        self._marker_id = 0
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            'DualGestureTracker started — tracking TWO hands with RViz viz.')

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        detected_sides = set()
        hand_markers = MarkerArray()
        self._marker_id = 0

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness_info in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness):
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                label = handedness_info.classification[0].label.lower()
                if label not in ('left', 'right'):
                    continue
                detected_sides.add(label)
                self._process_hand(label, hand_landmarks.landmark, frame)
                self._add_hand_markers(
                    hand_markers, label, hand_landmarks.landmark)

        if hand_markers.markers:
            self.hand_marker_pub.publish(hand_markers)
        else:
            clear = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            clear.markers.append(m)
            self.hand_marker_pub.publish(clear)

        self._publish_target_markers()

        for side in ('left', 'right'):
            if side not in detected_sides:
                estop_msg = Bool()
                estop_msg.data = False
                self.pubs[side]['estop'].publish(estop_msg)

        left_text = self._status_text('left')
        right_text = self._status_text('right')
        cv2.putText(frame, f'L: {left_text}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'R: {right_text}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)

        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.camera_pub.publish(img_msg)
        except Exception:
            pass

        if self.gui_available:
            cv2.imshow('Dual Gesture Tracker', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested — shutting down.')
                self.destroy_node()
                rclpy.shutdown()

    def _process_hand(self, side, lm, frame):
        st = self.state[side]
        ws = self.ws[side]

        raw_x = lm[self.WRIST].x
        raw_y = lm[self.WRIST].y

        st['smooth_x'] += self.smoothing * (raw_x - st['smooth_x'])
        st['smooth_y'] += self.smoothing * (raw_y - st['smooth_y'])

        robot_x = self._map(
            st['smooth_x'], 0.0, 1.0, ws['x_min'], ws['x_max'])
        robot_z = self._map(
            1.0 - st['smooth_y'], 0.0, 1.0, ws['z_min'], ws['z_max'])
        robot_y = ws['y_fixed']

        st['last_robot_pos'] = (robot_x, robot_y, robot_z)

        # Pinch detection with hysteresis
        pinch_dist = self._dist2d(lm[self.THUMB_TIP], lm[self.INDEX_TIP])
        if st['gripper_closed']:
            if pinch_dist > self.pinch_thresh + self.pinch_hyst:
                st['gripper_closed'] = False
        else:
            if pinch_dist < self.pinch_thresh:
                st['gripper_closed'] = True

        # Hand rotation (wrist -> middle MCP angle)
        dx = lm[self.MIDDLE_MCP].x - lm[self.WRIST].x
        dy = lm[self.MIDDLE_MCP].y - lm[self.WRIST].y
        raw_angle = math.atan2(dx, -dy)
        st['smooth_rot'] += self.smoothing * (raw_angle - st['smooth_rot'])

        # Fist = emergency stop
        st['emergency'] = self._is_fist(lm)

        # Publish Pose
        pose = Pose()
        pose.position.x = robot_x
        pose.position.y = robot_y
        pose.position.z = robot_z
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        self.pubs[side]['pose'].publish(pose)

        # Publish gripper
        grip_msg = Bool()
        grip_msg.data = st['gripper_closed']
        self.pubs[side]['gripper'].publish(grip_msg)

        # Publish hand rotation
        rot_msg = Float64()
        rot_msg.data = st['smooth_rot']
        self.pubs[side]['rotation'].publish(rot_msg)

        # Publish emergency stop
        estop_msg = Bool()
        estop_msg.data = st['emergency']
        self.pubs[side]['estop'].publish(estop_msg)

    def _add_hand_markers(self, marker_array, side, lm):
        ws = self.ws[side]
        now = self.get_clock().now().to_msg()
        if side == 'left':
            base_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            base_color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0)
        st = self.state[side]
        if st['emergency']:
            base_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif st['gripper_closed']:
            base_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        pts_3d = []
        for i in range(21):
            x_3d = self._map(lm[i].x, 0.0, 1.0, ws['x_min'], ws['x_max'])
            z_3d = self._map(1.0 - lm[i].y, 0.0, 1.0, ws['z_min'], ws['z_max'])
            y_3d = ws['y_fixed'] + (lm[i].z * -0.5)
            pts_3d.append((x_3d, y_3d, z_3d))

        for i, (px, py, pz) in enumerate(pts_3d):
            m = Marker()
            m.header = Header(stamp=now, frame_id='world')
            m.ns = f'{side}_hand_landmark'
            m.id = self._marker_id
            self._marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            m.pose.orientation.w = 1.0
            m.scale.x = 0.02
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color = base_color
            m.lifetime = Duration(sec=0, nanosec=200_000_000)
            marker_array.markers.append(m)

        bone_marker = Marker()
        bone_marker.header = Header(stamp=now, frame_id='world')
        bone_marker.ns = f'{side}_hand_bones'
        bone_marker.id = self._marker_id
        self._marker_id += 1
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.005
        bone_marker.color = base_color
        bone_marker.lifetime = Duration(sec=0, nanosec=200_000_000)
        bone_marker.pose.orientation.w = 1.0

        from geometry_msgs.msg import Point
        for (a, b) in _HAND_CONNECTIONS:
            p1 = Point(x=pts_3d[a][0], y=pts_3d[a][1], z=pts_3d[a][2])
            p2 = Point(x=pts_3d[b][0], y=pts_3d[b][1], z=pts_3d[b][2])
            bone_marker.points.append(p1)
            bone_marker.points.append(p2)
        marker_array.markers.append(bone_marker)

    def _publish_target_markers(self):
        target_markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, side in enumerate(('left', 'right')):
            pos = self.state[side].get('last_robot_pos')
            if pos is None:
                continue
            m = Marker()
            m.header = Header(stamp=now, frame_id='world')
            m.ns = 'gesture_target'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06
            m.scale.y = 0.06
            m.scale.z = 0.06
            if side == 'left':
                m.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.7)
            else:
                m.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.7)
            m.lifetime = Duration(sec=0, nanosec=500_000_000)
            target_markers.markers.append(m)
        if target_markers.markers:
            self.target_marker_pub.publish(target_markers)

    def _status_text(self, side):
        st = self.state[side]
        if st['emergency']:
            return 'FIST - E-STOP'
        elif st['gripper_closed']:
            return 'Pinch - GRIP'
        else:
            return 'Open'

    @staticmethod
    def _dist2d(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def _is_fist(self, lm):
        palm_x = (lm[self.WRIST].x + lm[self.INDEX_MCP].x +
                  lm[self.MIDDLE_MCP].x + lm[self.RING_MCP].x +
                  lm[self.PINKY_MCP].x) / 5.0
        palm_y = (lm[self.WRIST].y + lm[self.INDEX_MCP].y +
                  lm[self.MIDDLE_MCP].y + lm[self.RING_MCP].y +
                  lm[self.PINKY_MCP].y) / 5.0

        class _P:
            pass
        palm = _P()
        palm.x = palm_x
        palm.y = palm_y

        tips = [self.THUMB_TIP, self.INDEX_TIP, self.MIDDLE_TIP,
                self.RING_TIP, self.PINKY_TIP]
        for t in tips:
            if self._dist2d(lm[t], palm) > self.fist_thresh:
                return False
        return True

    @staticmethod
    def _map(value, in_min, in_max, out_min, out_max):
        value = max(in_min, min(in_max, value))
        return out_min + (value - in_min) / (in_max - in_min) * (out_max - out_min)

    def destroy_node(self):
        self.cap.release()
        if self.gui_available:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualGestureTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
