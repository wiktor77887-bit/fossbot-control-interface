"""
Live dashboard for FOSSBot: camera image, XY trajectory and velocity plots.

Read-only: it only subscribes (/odom, /cmd_vel, camera), so it can run next to
any controller - click teleop, gestures, keyboard teleop or Nav2.

The commanded velocity (/cmd_vel) is drawn next to the measured one (/odom),
which shows how the robot actually follows the commands (acceleration limits,
wheel slip, latency).

Keys in the window:  r - reset trajectory and plots,  q - quit.
"""

import math
import signal
import threading
import time

from fossbot_control_interface.dashboard_logic import (
    nice_ceil, quaternion_to_yaw, TimeSeries, Trajectory, update_square_limits)
from fossbot_control_interface.image_utils import image_to_bgr
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

# A command older than this is drawn as a gap ("nobody is commanding"), not as
# zero: the real FOSSBot stops after 0.5 s of silence, but the Gazebo DiffDrive
# plugin keeps executing the last command, and the plot must not hide that.
CMD_TIMEOUT_S = 0.5
CMD_SAMPLE_RATE = 20.0  # [Hz] the last command is sampled at this rate for the plot


class DashboardNode(Node):
    """Collects the data. Callbacks run in the ROS thread, the GUI reads under `lock`."""

    def __init__(self):
        super().__init__('dashboard')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.history = float(self.declare_parameter('history', 20.0).value)
        self.refresh_rate = float(self.declare_parameter('refresh_rate', 10.0).value)
        self.image_scale = float(self.declare_parameter('image_scale', 0.5).value)
        min_step = float(self.declare_parameter('trajectory_min_step', 0.01).value)
        max_points = int(self.declare_parameter('trajectory_max_points', 5000).value)

        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.cmd_series = TimeSeries(self.history, n_values=2)   # (v, w) commanded
        self.odom_series = TimeSeries(self.history, n_values=2)  # (v, w) measured
        self.trajectory = Trajectory(min_step, max_points)
        self.pose = None          # (x, y, yaw) of the last /odom message
        self.odom_twist = (0.0, 0.0)
        self.last_cmd = Twist()
        self.last_cmd_time = -math.inf
        self.image_msg = None
        self.image_time = -math.inf
        self.image_count = 0      # for the FPS counter

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / CMD_SAMPLE_RATE, self._sample_cmd)

        self.get_logger().info(
            f'Dashboard: odom={odom_topic}, cmd_vel={cmd_vel_topic}, image={self.image_topic}')

    def now(self) -> float:
        """Seconds since start (wall clock, so it works the same in sim and on the robot)."""
        return time.monotonic() - self.t0

    def _on_odom(self, msg: Odometry):
        p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        with self.lock:
            self.pose = (p.x, p.y, yaw)
            self.odom_twist = (tw.linear.x, tw.angular.z)
            self.trajectory.add(p.x, p.y)
            self.odom_series.append(self.now(), tw.linear.x, tw.angular.z)

    def _on_cmd(self, msg: Twist):
        with self.lock:
            self.last_cmd = msg
            self.last_cmd_time = self.now()

    def _sample_cmd(self):
        now = self.now()
        with self.lock:
            if now - self.last_cmd_time < CMD_TIMEOUT_S:
                v, w = self.last_cmd.linear.x, self.last_cmd.angular.z
            else:
                v, w = math.nan, math.nan  # matplotlib breaks the line on NaN
            self.cmd_series.append(now, v, w)

    def _on_image(self, msg: Image):
        # only keep the message; it is converted at the GUI refresh rate, not at 30 Hz
        with self.lock:
            self.image_msg = msg
            self.image_time = self.now()
            self.image_count += 1

    def reset(self):
        with self.lock:
            self.trajectory.clear()
            self.cmd_series.clear()
            self.odom_series.clear()


class DashboardWindow:
    """
    Matplotlib figure; all methods run in the main (GUI) thread.

    Rendering uses blitting: axes, ticks, labels and legends (text is the slow
    part of matplotlib) are drawn once and cached as a background bitmap; every
    frame only the "animated" artists (lines, image, status) are drawn on top.
    A full redraw happens only when axis limits change, and the limits change
    in coarse steps, so that is rare.
    """

    TRAJ_MARGIN = 0.5  # [m] free space around the path; the view moves when the robot nears it

    def __init__(self, node: DashboardNode, plt):
        self.node = node
        self.plt = plt
        self.image_error_logged = False
        self.fps = 0.0
        self.fps_count = 0
        self.fps_time = node.now()
        self.background = None
        self.traj_limits = None
        self.animated = []

        # 'constrained' layout is recomputed on every full redraw, so titles and
        # labels never overlap when the window is resized (or opens smaller).
        self.fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
        self.fig.canvas.manager.set_window_title('FOSSBot dashboard  (r: reset, q: quit)')
        (self.ax_cam, self.ax_traj), (self.ax_v, self.ax_w) = axes
        self._build_camera()
        self._build_trajectory()
        self.v_cmd, self.v_odom = self._build_velocity(self.ax_v, 'Linear velocity', 'v [m/s]')
        self.w_cmd, self.w_odom = self._build_velocity(self.ax_w, 'Angular velocity', 'ω [rad/s]')
        # supxlabel (not fig.text) so the constrained layout reserves space for it
        self.status = self._animated(self.fig.supxlabel(
            'waiting for odometry ...', family='monospace', fontsize=10))

        self.fig.canvas.mpl_connect('draw_event', self._on_draw)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.timer = self.fig.canvas.new_timer(interval=int(1000 / node.refresh_rate))
        self.timer.add_callback(self.update)
        self.timer.start()

    def _animated(self, artist):
        """Exclude the artist from the cached background; it is drawn on every frame."""
        artist.set_animated(True)
        self.animated.append(artist)
        return artist

    # ------------------------------------------------------------------ layout

    def _build_camera(self):
        import numpy as np
        self.ax_cam.set_title(f'Camera  {self.node.image_topic}')
        self.ax_cam.axis('off')
        # 'nearest' skips the costly antialiasing filter when the image is scaled
        self.cam_image = self._animated(self.ax_cam.imshow(
            np.zeros((240, 320, 3), dtype=np.uint8), interpolation='nearest'))
        self.cam_text = self._animated(self.ax_cam.text(
            0.5, 0.5, 'no image', color='0.7', ha='center', va='center',
            transform=self.ax_cam.transAxes))

    def _build_trajectory(self):
        ax = self.ax_traj
        ax.set_title('Trajectory (odom)')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
        self.path_line = self._animated(ax.plot([], [], '-', color='tab:blue', lw=1.5,
                                                label='path')[0])
        self.start_dot = self._animated(ax.plot([], [], 'o', color='tab:green', ms=6,
                                                label='start')[0])
        self.heading = self._animated(ax.plot([], [], '-', color='tab:red', lw=2.5)[0])
        self.robot_dot = self._animated(ax.plot([], [], 'o', color='tab:red', ms=8,
                                                label='robot')[0])
        ax.legend(loc='upper right', fontsize=8)
        self._set_traj_limits(update_square_limits(None, [], [], self.TRAJ_MARGIN))

    def _build_velocity(self, ax, title, ylabel):
        ax.set_title(title)
        ax.set_xlabel('time [s]')
        ax.set_ylabel(ylabel)
        ax.set_xlim(-self.node.history, 0.0)
        ax.set_ylim(-0.06, 0.06)
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color='0.5', lw=0.8)
        cmd = self._animated(ax.plot([], [], '--', color='tab:orange', lw=1.5,
                                     label='command (/cmd_vel, gap = silent)')[0])
        odom = self._animated(ax.plot([], [], '-', color='tab:blue', lw=1.5,
                                      label='measured (/odom)')[0])
        ax.legend(loc='upper left', fontsize=8)
        return cmd, odom

    # ----------------------------------------------------------------- refresh

    def _on_draw(self, event):
        """After a full redraw (start, resize, new limits): cache the static background."""
        canvas = self.fig.canvas
        self.background = canvas.copy_from_bbox(self.fig.bbox)
        self._draw_animated()

    def _draw_animated(self):
        for artist in self.animated:
            self.fig.draw_artist(artist)

    def update(self):
        node = self.node
        now = node.now()
        with node.lock:
            cmd = node.cmd_series.columns(now)
            odom = node.odom_series.columns(now)
            xs, ys = list(node.trajectory.xs), list(node.trajectory.ys)
            length = node.trajectory.length
            pose, twist = node.pose, node.odom_twist
            image_msg = node.image_msg if now - node.image_time < 1.0 else None
            image_count = node.image_count

        self._update_camera(image_msg, image_count, now)
        limits_changed = self._update_trajectory(xs, ys, pose)
        limits_changed |= self._update_velocity(
            self.ax_v, self.v_cmd, self.v_odom, cmd[0], cmd[1], odom[0], odom[1])
        limits_changed |= self._update_velocity(
            self.ax_w, self.w_cmd, self.w_odom, cmd[0], cmd[2], odom[0], odom[2])

        if pose is None:
            self.status.set_text('waiting for odometry ...')
        else:
            x, y, yaw = pose
            self.status.set_text(
                f'x={x:+.2f} m  y={y:+.2f} m  yaw={math.degrees(yaw):+6.1f}°  '
                f'v={twist[0]:+.2f} m/s  ω={twist[1]:+.2f} rad/s  '
                f'dist={length:.2f} m  cam={self.fps:.0f} fps')

        canvas = self.fig.canvas
        if limits_changed or self.background is None:
            canvas.draw_idle()  # full redraw; _on_draw then refreshes the background
        else:
            canvas.restore_region(self.background)
            self._draw_animated()
            canvas.blit(self.fig.bbox)  # Qt paints it in its event loop, no flush needed

    def _update_camera(self, msg, image_count, now):
        if now - self.fps_time >= 1.0:
            self.fps = (image_count - self.fps_count) / (now - self.fps_time)
            self.fps_count, self.fps_time = image_count, now
        if msg is None:
            self.cam_text.set_visible(True)
            self.fps = 0.0
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            if not self.image_error_logged:
                self.node.get_logger().error(str(exc))
                self.image_error_logged = True
            return
        step = max(1, int(round(1.0 / self.node.image_scale)))
        rgb = bgr[::step, ::step, ::-1]
        if self.cam_image.get_array().shape != rgb.shape:
            self.cam_image.set_extent((-0.5, rgb.shape[1] - 0.5, rgb.shape[0] - 0.5, -0.5))
        self.cam_image.set_data(rgb)
        self.cam_text.set_visible(False)

    def _set_traj_limits(self, limits):
        self.traj_limits = limits
        self.ax_traj.set_xlim(limits[0], limits[1])
        self.ax_traj.set_ylim(limits[2], limits[3])

    def _update_trajectory(self, xs, ys, pose) -> bool:
        """Update the artists; return True if the axis limits changed (full redraw needed)."""
        self.path_line.set_data(xs, ys)
        self.start_dot.set_data(xs[:1], ys[:1])
        if pose is None:
            return False
        x, y, yaw = pose
        self.robot_dot.set_data([x], [y])
        limits = update_square_limits(self.traj_limits, xs + [x], ys + [y], self.TRAJ_MARGIN)
        arrow = 0.08 * (limits[1] - limits[0])
        self.heading.set_data([x, x + arrow * math.cos(yaw)], [y, y + arrow * math.sin(yaw)])
        if limits is self.traj_limits:
            return False
        self._set_traj_limits(limits)
        return True

    @staticmethod
    def _update_velocity(ax, cmd_line, odom_line, t_cmd, v_cmd, t_odom, v_odom) -> bool:
        """Update the artists; return True if the axis limits changed (full redraw needed)."""
        cmd_line.set_data(t_cmd, v_cmd)
        odom_line.set_data(t_odom, v_odom)
        values = [abs(v) for v in list(v_cmd) + list(v_odom) if not math.isnan(v)]
        limit = 1.2 * nice_ceil(max(values, default=0.0))
        if ax.get_ylim()[1] == limit:
            return False
        ax.set_ylim(-limit, limit)
        return True

    def _on_key(self, event):
        if event.key == 'r':
            self.node.reset()
            self.traj_limits = None  # recentre the view on the next update
            self.node.get_logger().info('Trajectory and plots reset')
        # 'q' is matplotlib's default "close window" key


def _select_backend():
    """Use Qt if available (smoother), otherwise Tk; must run before importing pyplot."""
    import importlib.util

    import matplotlib
    if any(importlib.util.find_spec(m) for m in ('PyQt5', 'PyQt6', 'PySide6')):
        matplotlib.use('QtAgg')
    elif importlib.util.find_spec('tkinter'):
        matplotlib.use('TkAgg')
    else:
        raise RuntimeError('No GUI backend for matplotlib (install python3-pyqt5 or python3-tk)')
    import matplotlib.pyplot as plt
    return plt


def main(args=None):
    # rclpy's own SIGINT handler would shut ROS down while the GUI is still open;
    # instead Ctrl+C closes the window and the normal shutdown path runs.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    plt = _select_backend()
    signal.signal(signal.SIGINT, lambda *_: plt.close('all'))
    try:
        DashboardWindow(node, plt)
        plt.show()  # blocks until the window is closed
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
