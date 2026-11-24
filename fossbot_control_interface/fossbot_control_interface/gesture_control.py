"""
Gesture control for FOSSBot: two hand gestures start a 360 degree rotation.

    VICTORY (index + middle finger)  -> turn 360 deg counter-clockwise (left)
    HORNS   (index + pinky)          -> turn 360 deg clockwise (right)
    PALM    (open hand)              -> abort the rotation

Hands are found with the MediaPipe hand landmarker in the user camera image
(/user_cam/image_raw by default). The rotation is closed-loop on the yaw from
/odom, not timed: the simulator runs slower than real time and the real robot
does not turn at exactly the commanded speed.

While turning, /cmd_vel is published continuously (the real FOSSBot stops after
0.5 s without commands). A zero Twist is always sent at the end of a rotation,
on abort and on shutdown (the Gazebo DiffDrive plugin keeps executing the last
command forever). When idle the node is silent, so it does not fight other
publishers such as click_teleop.

Keys in the window:  SPACE / s - abort,  q / ESC - quit.
Services (std_srvs/Trigger): ~/rotate_left, ~/rotate_right, ~/stop.
"""

import os
import signal
import threading
import time

import cv2
from fossbot_control_interface.dashboard_logic import quaternion_to_yaw
from fossbot_control_interface.gesture_logic import (
    classify, Gesture, GestureDebouncer, rotation_speed, RotationTracker)
from fossbot_control_interface.image_utils import image_to_bgr
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

WINDOW_NAME = 'FOSSBot gestures'
STOP_REPEAT_S = 0.5   # keep sending zero Twists this long after a motion ends

# Pairs of landmarks drawn as the hand skeleton
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17))

GESTURE_ACTION = {
    Gesture.VICTORY: +1,   # counter-clockwise
    Gesture.HORNS: -1,     # clockwise
}


class HandDetector:
    """Thin wrapper around the MediaPipe hand landmarker (VIDEO mode, one hand)."""

    def __init__(self, model_path: str):
        # Imported here so the rest of the package (and its tests) does not need MediaPipe.
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        self._mp = mp
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1)
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._last_ts = -1

    def detect(self, bgr: np.ndarray):
        """Return 21 (x, y) landmarks in pixels, or None if no hand is visible."""
        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        # VIDEO mode needs strictly increasing timestamps [ms]
        ts = max(int(time.monotonic() * 1000), self._last_ts + 1)
        self._last_ts = ts
        result = self._landmarker.detect_for_video(image, ts)
        if not result.hand_landmarks:
            return None
        h, w = bgr.shape[:2]
        return [(lm.x * w, lm.y * h) for lm in result.hand_landmarks[0]]

    def close(self):
        self._landmarker.close()


class GestureControl(Node):

    def __init__(self):
        super().__init__('gesture_control')

        self.image_topic = self.declare_parameter('image_topic', '/user_cam/image_raw').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        model_path = os.path.expanduser(
            self.declare_parameter('model_path', '~/hand_landmarker.task').value)
        self.max_speed = float(self.declare_parameter('max_angular_speed', 0.8).value)
        self.min_speed = float(self.declare_parameter('min_angular_speed', 0.2).value)
        self.slow_zone = float(self.declare_parameter('slow_zone', 0.6).value)
        self.tolerance = float(self.declare_parameter('angle_tolerance', 0.02).value)
        hold_frames = int(self.declare_parameter('hold_frames', 5).value)
        self.finger_ratio = float(self.declare_parameter('finger_ratio', 1.2).value)
        publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 0.5).value)
        self.rotation_timeout = float(self.declare_parameter('rotation_timeout', 30.0).value)
        self.show_window = bool(self.declare_parameter('show_window', True).value)
        self.mirror = bool(self.declare_parameter('mirror_preview', True).value)
        enable_motors = bool(self.declare_parameter('enable_motors_on_start', False).value)

        rotation_speed(1.0, self.max_speed, self.min_speed, self.slow_zone)  # validate early
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'Hand landmarker model not found: {model_path}. Download it with: '
                'wget -O ~/hand_landmarker.task https://storage.googleapis.com/'
                'mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task')
        self.detector = HandDetector(model_path)
        self.debouncer = GestureDebouncer(hold_frames)

        self.lock = threading.Lock()
        self.yaw = None
        self.last_odom_time = 0.0
        self.tracker = None              # RotationTracker while a rotation runs
        self.rotation_started = 0.0
        self.silent_after = 0.0          # monotonic time when zero Twists stop
        self.status = 'idle'
        self.preview = None              # annotated BGR frame for the window
        self.preview_time = 0.0
        self.quit_requested = False

        # Images are slow to process (~20 ms per frame); keep them away from the
        # control loop, which must run at publish_rate during a rotation.
        vision_group = MutuallyExclusiveCallbackGroup()
        control_group = MutuallyExclusiveCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10,
                                 callback_group=control_group)
        self.create_subscription(Image, self.image_topic, self._on_image,
                                 qos_profile_sensor_data, callback_group=vision_group)
        self.create_timer(1.0 / publish_rate, self._control_step, callback_group=control_group)
        self.create_service(Trigger, '~/rotate_left',
                            lambda req, res: self._service_start(res, +1),
                            callback_group=control_group)
        self.create_service(Trigger, '~/rotate_right',
                            lambda req, res: self._service_start(res, -1),
                            callback_group=control_group)
        self.create_service(Trigger, '~/stop', self._service_stop, callback_group=control_group)

        self.motors_enabled = not enable_motors
        self._enable_pending = False
        if enable_motors:
            self.enable_client = self.create_client(
                SetBool, '/fossbot/enable_motors', callback_group=control_group)
            self.enable_timer = self.create_timer(
                1.0, self._try_enable_motors, callback_group=control_group)

        self.get_logger().info(
            f'Gesture control ready: image={self.image_topic}, odom={odom_topic}, '
            f'cmd_vel={cmd_vel_topic}. VICTORY = 360 deg left, HORNS = 360 deg right, '
            'open PALM = abort')

    # ----------------------------------------------------------------- vision

    def _on_image(self, msg: Image):
        try:
            frame = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        points = self.detector.detect(frame)
        gesture = classify(points, self.finger_ratio)
        fired = self.debouncer.update(gesture)

        if gesture == Gesture.PALM and self.tracker is not None:
            self.abort('open palm')
        elif fired in GESTURE_ACTION:
            self.start_rotation(GESTURE_ACTION[fired], f'gesture {fired.value}')

        if self.show_window:
            self._annotate(frame, points, gesture)

    def _annotate(self, frame, points, gesture):
        if points is not None:
            pts = [(int(x), int(y)) for x, y in points]
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame, pts[a], pts[b], (0, 200, 0), 2, cv2.LINE_AA)
            for p in pts:
                cv2.circle(frame, p, 3, (0, 0, 255), -1, cv2.LINE_AA)
        if self.mirror:
            frame = cv2.flip(frame, 1)  # selfie view; landmarks were found on the raw image
        font = cv2.FONT_HERSHEY_SIMPLEX
        h, w = frame.shape[:2]
        progress = self.debouncer.progress
        cv2.putText(frame, f'gesture: {gesture.value}', (10, 25), font, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        if progress > 0.0:
            cv2.rectangle(frame, (10, 35), (10 + int(150 * progress), 42), (0, 255, 255), -1)
        with self.lock:
            status = self.status
        cv2.putText(frame, status, (10, 65), font, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'V: left 360   horns: right 360   palm/SPACE: abort   q: quit',
                    (10, h - 12), font, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        with self.lock:
            self.preview = frame
            self.preview_time = time.monotonic()

    # ---------------------------------------------------------------- control

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        with self.lock:
            self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.last_odom_time = time.monotonic()

    def start_rotation(self, direction: int, reason: str) -> str:
        """Start a 360 degree rotation; return an error message or ''."""
        now = time.monotonic()
        with self.lock:
            if self.tracker is not None:
                return 'already rotating'
            if not self.motors_enabled:
                return 'motors are not enabled yet'
            if self.yaw is None or now - self.last_odom_time > self.odom_timeout:
                return 'no recent odometry'
            self.tracker = RotationTracker(direction)
            self.tracker.update(self.yaw)
            self.rotation_started = now
            side = 'left (CCW)' if direction > 0 else 'right (CW)'
            self.status = f'turning 360 {side}'
        self.get_logger().info(f'Rotation 360 deg {side} started ({reason})')
        return ''

    def abort(self, reason: str):
        with self.lock:
            if self.tracker is None:
                return
            turned = self.tracker.turned
            self._finish_locked(f'aborted ({reason})')
        self.get_logger().warn(f'Rotation aborted after {turned * 57.2958:.0f} deg: {reason}')

    def _finish_locked(self, status: str):
        self.tracker = None
        self.status = status
        self.silent_after = time.monotonic() + STOP_REPEAT_S
        self.cmd_pub.publish(Twist())

    def _control_step(self):
        now = time.monotonic()
        msg = Twist()
        error = done = None
        with self.lock:
            tracker = self.tracker
            if tracker is not None:
                remaining = tracker.update(self.yaw)
                if now - self.last_odom_time > self.odom_timeout:
                    error = 'odometry lost'
                elif now - self.rotation_started > self.rotation_timeout:
                    error = 'timeout'
                elif remaining <= self.tolerance:
                    done = tracker.turned
                    self._finish_locked('done')
                else:
                    speed = rotation_speed(remaining, self.max_speed, self.min_speed,
                                           self.slow_zone)
                    msg.angular.z = tracker.direction * speed
                    self.cmd_pub.publish(msg)
                    self.status = (f'turning: {tracker.turned * 57.2958:.0f} / 360 deg')
            elif now < self.silent_after:
                self.cmd_pub.publish(msg)  # zero Twist
        if error:
            self.abort(error)
        elif done is not None:
            self.get_logger().info(f'Rotation done: {done * 57.2958:.1f} deg')

    def send_final_stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)  # let DDS deliver before the node is destroyed

    # --------------------------------------------------------------- services

    def _service_start(self, response, direction):
        error = self.start_rotation(direction, 'service')
        response.success = not error
        response.message = error or 'rotation started'
        return response

    def _service_stop(self, request, response):
        self.abort('service')
        response.success = True
        response.message = 'stopped'
        return response

    # ----------------------------------------------------------------- motors

    def _try_enable_motors(self):
        if self.motors_enabled or self._enable_pending:
            return
        if not self.enable_client.service_is_ready():
            self.get_logger().warn(
                'Waiting for /fossbot/enable_motors ...', throttle_duration_sec=5.0)
            return
        self._enable_pending = True
        future = self.enable_client.call_async(SetBool.Request(data=True))
        future.add_done_callback(self._on_motors_enabled)

    def _on_motors_enabled(self, future):
        result = future.result()
        self._enable_pending = False
        if result is not None and result.success:
            self.motors_enabled = True
            self.get_logger().info('Motors enabled')
            self.enable_timer.cancel()
        else:
            self.get_logger().error('Enabling motors failed, retrying')

    # -------------------------------------------------------------------- GUI

    def refresh_window(self):
        with self.lock:
            frame = self.preview
            fresh = frame is not None and time.monotonic() - self.preview_time < 1.0
        if not fresh:
            frame = np.full((480, 640, 3), 40, dtype=np.uint8)
            cv2.putText(frame, f'No image on {self.image_topic}', (10, 465),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord(' '), ord('s')):
            self.abort('key')
        elif key in (ord('q'), 27):
            self.quit_requested = True
        elif cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            self.quit_requested = True


def main(args=None):
    # Own SIGINT/SIGTERM handling: rclpy's handler would shut ROS down before the
    # final zero Twist could be published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = GestureControl()
    except (FileNotFoundError, ImportError, ValueError) as exc:
        print(f'[gesture_control] {exc}', flush=True)
        rclpy.try_shutdown()
        raise SystemExit(1)

    stop_event = threading.Event()

    def request_stop(*_):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    try:
        while not stop_event.is_set() and not node.quit_requested:
            if node.show_window:
                node.refresh_window()
                time.sleep(1.0 / 30.0)
            else:
                stop_event.wait(0.1)
    finally:
        # Stop the callbacks first, so nothing can publish a motion after the final stop.
        executor.shutdown()
        ros_thread.join(timeout=1.0)
        node.send_final_stop()
        node.detector.close()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
