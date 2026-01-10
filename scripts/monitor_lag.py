#!/usr/bin/env python3

import threading
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

try:
    from control_msgs.msg import JointTrajectoryControllerState  # type: ignore

    HAVE_JTC_STATE = True
except Exception:
    JointTrajectoryControllerState = None  # type: ignore
    HAVE_JTC_STATE = False


@dataclass
class GapStats:
    last_wall: Optional[float] = None
    last_ros: Optional[Time] = None
    max_wall_gap_s: float = 0.0
    max_ros_gap_s: float = 0.0

    # Rolling windows to characterize jitter, not only maxima
    wall_dt_window: "deque[float]" = None  # type: ignore
    ros_dt_window: "deque[float]" = None  # type: ignore

    def ensure_windows(self, window_size: int) -> None:
        if self.wall_dt_window is None:
            self.wall_dt_window = deque(maxlen=window_size)
        if self.ros_dt_window is None:
            self.ros_dt_window = deque(maxlen=window_size)


class LagMonitor(Node):
    def __init__(self) -> None:
        super().__init__("lag_monitor")

        # Make these relatively sensitive by default; Gazebo GUI "stutters" can be visible at ~50-150ms.
        self.declare_parameter("warn_wall_gap_s", 0.08)
        self.declare_parameter("warn_clock_wall_gap_s", 0.08)
        self.declare_parameter("warn_ctrl_state_wall_gap_s", 0.08)
        self.declare_parameter("warn_joint_jump_rad", 0.10)
        self.declare_parameter("motion_stall_vel_rad_s", 0.02)
        self.declare_parameter("motion_stall_min_s", 0.50)
        self.declare_parameter("motion_stall_repeat_s", 1.0)
        self.declare_parameter("tracking_err_threshold_rad", 0.05)
        self.declare_parameter("print_period_s", 5.0)
        self.declare_parameter("monitor_controller_state", True)
        self.declare_parameter("dt_window_size", 200)

        # IMPORTANT: if this node runs with use_sim_time:=true, timers are driven by /clock.
        # If /clock stalls (common during Gazebo/WSL stutters), periodic timers do not fire,
        # which can make the monitor go completely silent during the exact 0.5-2s hangs we
        # care about. This wall-clock heartbeat thread is independent of /clock.
        self.declare_parameter("enable_wall_heartbeat", True)
        self.declare_parameter("heartbeat_period_s", 0.2)
        self.declare_parameter("heartbeat_warn_no_msg_s", 0.8)
        self.declare_parameter("heartbeat_repeat_s", 1.0)

        window_size = int(self.get_parameter("dt_window_size").value)
        if window_size < 10:
            window_size = 10

        self._js_stats = GapStats()
        self._clock_stats = GapStats()
        self._ctrl_stats = GapStats()

        self._js_stats.ensure_windows(window_size)
        self._clock_stats.ensure_windows(window_size)
        self._ctrl_stats.ensure_windows(window_size)

        self._last_positions: Dict[str, float] = {}
        self._last_js_wall: Optional[float] = None
        self._stall_start_wall: Optional[float] = None
        self._last_stall_warn_wall: Optional[float] = None

        self._last_tracking_err: Optional[float] = None

        self._last_desired: Dict[str, float] = {}
        self._last_actual: Dict[str, float] = {}

        self._js_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self._clock_sub = self.create_subscription(Clock, "/clock", self._on_clock, 50)

        if bool(self.get_parameter("monitor_controller_state").value) and HAVE_JTC_STATE:
            self._ctrl_sub = self.create_subscription(
                JointTrajectoryControllerState,
                "/joint_trajectory_controller/state",
                self._on_controller_state,
                50,
            )
            self.get_logger().info("Also monitoring /joint_trajectory_controller/state (desired vs actual).")
        else:
            self._ctrl_sub = None
            if bool(self.get_parameter("monitor_controller_state").value) and not HAVE_JTC_STATE:
                self.get_logger().warn(
                    "control_msgs/JointTrajectoryControllerState not available; skipping controller state monitoring."
                )

        self._last_print_wall = time.monotonic()
        self._timer = self.create_timer(0.2, self._periodic)

        self._lock = threading.Lock()
        self._hb_stop = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None

        self._last_hb_warn_clock_wall: Optional[float] = None
        self._last_hb_warn_js_wall: Optional[float] = None
        self._last_hb_warn_ctrl_wall: Optional[float] = None

        if bool(self.get_parameter("enable_wall_heartbeat").value):
            self._hb_thread = threading.Thread(target=self._wall_heartbeat, daemon=True)
            self._hb_thread.start()

        self.get_logger().info(
            "LagMonitor running. Watching /joint_states and /clock for wall-time gaps and joint jumps."  # noqa: E501
        )

    def stop(self) -> None:
        self._hb_stop.set()
        if self._hb_thread is not None and self._hb_thread.is_alive():
            self._hb_thread.join(timeout=1.0)

    def _wall_heartbeat(self) -> None:
        # Runs on wall clock so it still reports when /clock freezes.
        while not self._hb_stop.is_set():
            period = float(self.get_parameter("heartbeat_period_s").value)
            warn_no_msg = float(self.get_parameter("heartbeat_warn_no_msg_s").value)
            repeat = float(self.get_parameter("heartbeat_repeat_s").value)

            if period <= 0.0:
                period = 0.2
            if warn_no_msg <= 0.0:
                warn_no_msg = 0.8

            time.sleep(period)
            now = time.monotonic()

            with self._lock:
                last_clock = self._clock_stats.last_wall
                last_js = self._js_stats.last_wall
                last_ctrl = self._ctrl_stats.last_wall

                def should_warn(last_warn: Optional[float]) -> bool:
                    return last_warn is None or repeat <= 0.0 or (now - last_warn) >= repeat

                if last_clock is not None and (now - last_clock) >= warn_no_msg and should_warn(
                    self._last_hb_warn_clock_wall
                ):
                    self.get_logger().warn(
                        "Heartbeat: no /clock message for %.3fs (wall). This usually means Gazebo/bridge is stalled."
                        % (now - last_clock)
                    )
                    self._last_hb_warn_clock_wall = now

                if last_js is not None and (now - last_js) >= warn_no_msg and should_warn(
                    self._last_hb_warn_js_wall
                ):
                    self.get_logger().warn(
                        "Heartbeat: no /joint_states message for %.3fs (wall). Controller/executor likely stalled."
                        % (now - last_js)
                    )
                    self._last_hb_warn_js_wall = now

                if self._ctrl_sub is not None and last_ctrl is not None and (now - last_ctrl) >= warn_no_msg and should_warn(
                    self._last_hb_warn_ctrl_wall
                ):
                    self.get_logger().warn(
                        "Heartbeat: no /joint_trajectory_controller/state message for %.3fs (wall)."
                        % (now - last_ctrl)
                    )
                    self._last_hb_warn_ctrl_wall = now

    def _periodic(self) -> None:
        now = time.monotonic()
        period = float(self.get_parameter("print_period_s").value)
        if period > 0.0 and (now - self._last_print_wall) >= period:
            self._last_print_wall = now

            def summarize(stats: GapStats) -> str:
                if not stats.wall_dt_window:
                    return "n/a"
                wall = list(stats.wall_dt_window)
                if len(wall) < 2:
                    return "n/a"
                wall_sorted = sorted(wall)
                p50 = wall_sorted[len(wall_sorted) // 2]
                p95 = wall_sorted[int(0.95 * (len(wall_sorted) - 1))]
                mx = max(wall)
                hz = (1.0 / p50) if p50 > 1e-6 else 0.0
                return "hz≈%.1f p50=%.3fs p95=%.3fs max=%.3fs" % (hz, p50, p95, mx)

            self.get_logger().info(
                "Max gaps so far: joint_states wall=%.3fs ros=%.3fs | clock wall=%.3fs ros=%.3fs | ctrl_state wall=%.3fs ros=%.3fs"
                % (
                    self._js_stats.max_wall_gap_s,
                    self._js_stats.max_ros_gap_s,
                    self._clock_stats.max_wall_gap_s,
                    self._clock_stats.max_ros_gap_s,
                    self._ctrl_stats.max_wall_gap_s,
                    self._ctrl_stats.max_ros_gap_s,
                )
            )

            self.get_logger().info(
                "Jitter: joint_states(%s) clock(%s) ctrl_state(%s)"
                % (summarize(self._js_stats), summarize(self._clock_stats), summarize(self._ctrl_stats))
            )

    def _update_gap(self, stats: GapStats, now_wall: float, now_ros: Time) -> None:
        stats.ensure_windows(int(self.get_parameter("dt_window_size").value))
        if stats.last_wall is not None:
            wall_gap = now_wall - stats.last_wall
            stats.max_wall_gap_s = max(stats.max_wall_gap_s, wall_gap)
            if stats.wall_dt_window is not None:
                stats.wall_dt_window.append(wall_gap)
        else:
            wall_gap = 0.0

        if stats.last_ros is not None:
            # ROS time delta in seconds
            ros_gap = (now_ros - stats.last_ros).nanoseconds / 1e9
            stats.max_ros_gap_s = max(stats.max_ros_gap_s, ros_gap)
            if stats.ros_dt_window is not None:
                stats.ros_dt_window.append(ros_gap)
        else:
            ros_gap = 0.0

        with self._lock:
            stats.last_wall = now_wall
            stats.last_ros = now_ros

        return wall_gap, ros_gap

    def _on_clock(self, msg: Clock) -> None:
        now_wall = time.monotonic()
        now_ros = Time.from_msg(msg.clock)

        wall_gap, ros_gap = self._update_gap(self._clock_stats, now_wall, now_ros)
        warn_wall = float(self.get_parameter("warn_clock_wall_gap_s").value)

        if warn_wall > 0.0 and wall_gap >= warn_wall:
            self.get_logger().warn(
                "Clock wall-gap %.3fs (ros dt %.3fs). If this spikes, /clock delivery is stalling."
                % (wall_gap, ros_gap)
            )

    def _on_joint_state(self, msg: JointState) -> None:
        now_wall = time.monotonic()
        now_ros = Time.from_msg(msg.header.stamp)

        wall_gap, ros_gap = self._update_gap(self._js_stats, now_wall, now_ros)

        warn_wall = float(self.get_parameter("warn_wall_gap_s").value)
        if warn_wall > 0.0 and wall_gap >= warn_wall:
            self.get_logger().warn(
                "joint_states wall-gap %.3fs (ros dt %.3fs). If this spikes, executor/transport is stalling."
                % (wall_gap, ros_gap)
            )

        # Keep a snapshot of previous positions before updating.
        prev_positions = dict(self._last_positions)

        # Detect sudden position jumps (can indicate a "pause then catch-up" effect).
        warn_jump = float(self.get_parameter("warn_joint_jump_rad").value)
        max_jump = 0.0
        max_joint = None
        if warn_jump > 0.0 and msg.name and msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                if not isinstance(pos, float) or not math.isfinite(pos):
                    continue
                if name in prev_positions:
                    jump = abs(pos - prev_positions[name])
                    if jump > max_jump:
                        max_jump = jump
                        max_joint = name

        if max_jump >= warn_jump and max_joint is not None:
            self.get_logger().warn(
                "Joint jump detected: %s jumped by %.3frad between joint_states messages." % (max_joint, max_jump)
            )

        # Detect motion stalls (near-zero motion) for a sustained time.
        # Prefer msg.velocity when available (joint_state_broadcaster publishes it).
        stall_vel = float(self.get_parameter("motion_stall_vel_rad_s").value)
        stall_min_s = float(self.get_parameter("motion_stall_min_s").value)
        stall_repeat_s = float(self.get_parameter("motion_stall_repeat_s").value)
        err_thresh = float(self.get_parameter("tracking_err_threshold_rad").value)

        if self._last_js_wall is not None:
            dt = now_wall - self._last_js_wall
        else:
            dt = 0.0

        max_abs_vel = 0.0
        if msg.velocity and msg.name and len(msg.velocity) == len(msg.name):
            for v in msg.velocity:
                try:
                    vf = float(v)
                except Exception:
                    continue
                if math.isfinite(vf):
                    max_abs_vel = max(max_abs_vel, abs(vf))
        elif dt > 1e-4 and msg.name and msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                if not isinstance(pos, float) or not math.isfinite(pos):
                    continue
                if name in prev_positions:
                    max_abs_vel = max(max_abs_vel, abs(pos - prev_positions[name]) / dt)

        should_consider_stall = True
        if HAVE_JTC_STATE and self._last_tracking_err is not None:
            # Only call it a "stall" if controller is actually trying to move (tracking error).
            should_consider_stall = self._last_tracking_err >= err_thresh

        if should_consider_stall and stall_vel > 0.0 and stall_min_s > 0.0 and dt > 1e-4:
            if max_abs_vel < stall_vel:
                if self._stall_start_wall is None:
                    self._stall_start_wall = now_wall
                    self._last_stall_warn_wall = None
                else:
                    stalled_for = now_wall - self._stall_start_wall
                    can_repeat = (
                        self._last_stall_warn_wall is None
                        or stall_repeat_s <= 0.0
                        or (now_wall - self._last_stall_warn_wall) >= stall_repeat_s
                    )
                    if stalled_for >= stall_min_s and can_repeat:
                        self.get_logger().warn(
                            "Motion stall: max|vel|=%.3frad/s for %.2fs (dt=%.3fs, max_jump=%.3frad, tracking_err=%.3frad)."
                            % (
                                max_abs_vel,
                                stalled_for,
                                dt,
                                max_jump,
                                float(self._last_tracking_err) if self._last_tracking_err is not None else -1.0,
                            )
                        )
                        self._last_stall_warn_wall = now_wall
            else:
                self._stall_start_wall = None
                self._last_stall_warn_wall = None

        # Finally update cached positions for next callback.
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                if isinstance(pos, float) and math.isfinite(pos):
                    self._last_positions[name] = pos

        self._last_js_wall = now_wall


    def _on_controller_state(self, msg) -> None:
        # Track controller state delivery jitter, because it is published by the controller.
        now_wall = time.monotonic()
        try:
            # header.stamp exists on this message in ROS 2
            now_ros = Time.from_msg(msg.header.stamp)
        except Exception:
            now_ros = Time(seconds=0, nanoseconds=0)

        wall_gap, ros_gap = self._update_gap(self._ctrl_stats, now_wall, now_ros)
        warn_ctrl = float(self.get_parameter("warn_ctrl_state_wall_gap_s").value)
        if warn_ctrl > 0.0 and wall_gap >= warn_ctrl:
            self.get_logger().warn(
                "ctrl_state wall-gap %.3fs (ros dt %.3fs). If this spikes, controller publishing/scheduling is stalling."
                % (wall_gap, ros_gap)
            )

        # Optional: track desired vs actual.
        try:
            desired = msg.desired
            actual = msg.actual
            if not desired.joint_names or not desired.positions or not actual.positions:
                return
            if len(desired.joint_names) != len(desired.positions) or len(desired.joint_names) != len(actual.positions):
                return

            max_err = 0.0
            max_joint = None
            for name, d, a in zip(desired.joint_names, desired.positions, actual.positions):
                if not (math.isfinite(float(d)) and math.isfinite(float(a))):
                    continue
                err = abs(float(d) - float(a))
                if err > max_err:
                    max_err = err
                    max_joint = name

            # This is just a diagnostic hint: large sustained error can look like stutter.
            if max_err >= 0.15:
                self.get_logger().warn(
                    "Tracking error high: %s |desired-actual|=%.3frad (controller state)." % (max_joint, max_err)
                )

            self._last_tracking_err = max_err
        except Exception:
            return


def main() -> None:
    rclpy.init()
    node = LagMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # rcl_shutdown can race with ExternalShutdownException depending on how Ctrl-C is delivered
            pass


if __name__ == "__main__":
    main()
