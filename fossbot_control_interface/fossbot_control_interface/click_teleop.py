"""
Click-based teleoperation for FOSSBot.

Shows a window (camera image if available, a plain canvas otherwise). Clicking
above the horizontal centre line drives the robot forward, clicking below drives
it backward. The further from the centre, the faster (optional).

Keys in the window:  SPACE / s - stop,  q / ESC - quit.
"""

import signal
import time

import cv2
from fossbot_control_interface.click_logic import click_to_linear_velocity
from fossbot_control_interface.image_utils import image_to_bgr
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

WINDOW_NAME = 'FOSSBot click teleop'

# How long to keep sending zero Twists after a motion ends. After that the node
# goes silent on /cmd_vel so it does not fight other publishers (e.g. gestures).
STOP_REPEAT_S = 0.5


class ClickTeleop(Node):

    def __init__(self):
        super().__init__('click_teleop')

        self.image_topic = self.declare_parameter('image_topic', '/user_cam/image_raw').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.max_speed = float(self.declare_parameter('max_linear_speed', 0.2).value)
        self.min_speed = float(self.declare_parameter('min_linear_speed', 0.05).value)
        self.dead_zone = float(self.declare_parameter('dead_zone', 0.05).value)
        self.proportional = bool(self.declare_parameter('proportional_speed', True).value)
        self.move_duration = float(self.declare_parameter('move_duration', 1.0).value)
        publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)
        self.width = int(self.declare_parameter('window_width', 640).value)
        self.height = int(self.declare_parameter('window_height', 480).value)
        enable_motors = bool(self.declare_parameter('enable_motors_on_start', False).value)

        # validate once at start-up instead of failing on the first click
        click_to_linear_velocity(0, self.height, self.max_speed, self.min_speed, self.dead_zone)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self.frame = None
        self.last_image_time = 0.0
        self.image_error_logged = False

        self.speed = 0.0
        self.move_until = 0.0           # monotonic time when the current motion ends
        self.silent_after = 0.0         # monotonic time when we stop sending zero Twists
        self.last_click = None          # (x, y) of the last click, for drawing
        self.quit_requested = False

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)

        self.create_timer(1.0 / publish_rate, self._publish_cmd)
        self.create_timer(1.0 / 30.0, self._refresh_window)

        self.motors_enabled = not enable_motors
        if enable_motors:
            self.enable_client = self.create_client(SetBool, '/fossbot/enable_motors')
            self.enable_timer = self.create_timer(1.0, self._try_enable_motors)

        self.get_logger().info(
            f'Click teleop ready: image={self.image_topic}, cmd_vel={cmd_vel_topic}, '
            f'max_speed={self.max_speed} m/s, move_duration={self.move_duration} s')

    # ------------------------------------------------------------------ ROS I/O

    def _on_image(self, msg: Image):
        try:
            frame = image_to_bgr(msg)
        except ValueError as exc:
            if not self.image_error_logged:
                self.get_logger().error(str(exc))
                self.image_error_logged = True
            return
        self.frame = cv2.resize(frame, (self.width, self.height))
        self.last_image_time = time.monotonic()

    def _publish_cmd(self):
        now = time.monotonic()
        msg = Twist()
        if now < self.move_until:
            # Publish continuously: the real FOSSBot stops if /cmd_vel is silent for 0.5 s.
            msg.linear.x = self.speed
        elif now < self.silent_after:
            pass  # zero Twist
        else:
            return
        self.cmd_pub.publish(msg)

    def _try_enable_motors(self):
        if self.motors_enabled:
            return
        if not self.enable_client.service_is_ready():
            self.get_logger().warn(
                'Waiting for /fossbot/enable_motors ...', throttle_duration_sec=5.0)
            return
        future = self.enable_client.call_async(SetBool.Request(data=True))
        future.add_done_callback(self._on_motors_enabled)
        self.motors_enabled = True  # do not send a second request while waiting

    def _on_motors_enabled(self, future):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info('Motors enabled')
            self.enable_timer.cancel()
        else:
            self.get_logger().error('Enabling motors failed, retrying')
            self.motors_enabled = False

    # ---------------------------------------------------------------- commands

    def start_motion(self, speed: float):
        now = time.monotonic()
        self.speed = speed
        self.move_until = now + self.move_duration if speed != 0.0 else now
        self.silent_after = self.move_until + STOP_REPEAT_S
        direction = 'FORWARD' if speed > 0 else 'BACKWARD' if speed < 0 else 'STOP'
        self.get_logger().info(f'{direction}: {speed:+.2f} m/s')

    def stop(self):
        self.start_motion(0.0)

    def send_final_stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)  # let DDS deliver before the node is destroyed

    # ---------------------------------------------------------------------- GUI

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        self.last_click = (x, y)
        self.start_motion(click_to_linear_velocity(
            y, self.height, self.max_speed, self.min_speed, self.dead_zone, self.proportional))

    def _refresh_window(self):
        now = time.monotonic()
        has_image = self.frame is not None and now - self.last_image_time < 1.0
        if has_image:
            canvas = self.frame.copy()
        else:
            canvas = np.full((self.height, self.width, 3), 40, dtype=np.uint8)
            cv2.putText(canvas, f'No image on {self.image_topic}', (10, self.height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
        self._draw_overlay(canvas, now)
        cv2.imshow(WINDOW_NAME, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord(' '), ord('s')):
            self.stop()
        elif key in (ord('q'), 27):
            self.quit_requested = True
        elif cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            self.quit_requested = True  # window closed with the X button

    def _draw_overlay(self, canvas, now):
        h, w = self.height, self.width
        centre = h // 2
        band = int(self.dead_zone * centre)

        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, centre - band), (w, centre + band), (0, 200, 255), -1)
        cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0, dst=canvas)
        cv2.line(canvas, (0, centre), (w, centre), (0, 200, 255), 1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(canvas, 'FORWARD', (10, 25), font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(canvas, 'BACKWARD', (10, h - 40), font, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        moving = now < self.move_until
        if moving:
            status = f'v = {self.speed:+.2f} m/s  ({self.move_until - now:.1f} s)'
        else:
            status = 'stopped'
        cv2.putText(canvas, status, (w - 260, 25), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, 'SPACE: stop   q: quit', (w - 200, h - 15), font, 0.45,
                    (200, 200, 200), 1, cv2.LINE_AA)

        if self.last_click is not None:
            if self.speed > 0:
                colour = (0, 255, 0)
            elif self.speed < 0:
                colour = (0, 0, 255)
            else:
                colour = (0, 200, 255)
            cv2.circle(canvas, self.last_click, 8, colour, 2)


def main(args=None):
    # Own SIGINT/SIGTERM handling: rclpy's handler would shut ROS down first, and
    # the final zero Twist could not be published any more - the Gazebo DiffDrive
    # would then keep driving with the last command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ClickTeleop()

    def request_quit(*_):
        node.quit_requested = True

    signal.signal(signal.SIGINT, request_quit)
    signal.signal(signal.SIGTERM, request_quit)
    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.send_final_stop()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
