#!/usr/bin/env python3

"""Diagnose sporadic ~1s stalls in sim/control without relying on /clock timers.

Goal: classify stalls into:
  A) Sim/transport/executor freeze: /clock and/or /joint_states stop arriving
  B) Control/physics stall: /clock and /joint_states still arrive, but joints do not move

All logs include a monotonic (steady) timestamp (t=...s).

Run examples:
  python3 scripts/diagnose_stalls.py
  ros2 run arm_bringup ensure_controllers_active  # idempotent controller activation

This node intentionally avoids ROS timers so it still reports when /clock freezes.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

try:
    from controller_manager_msgs.srv import ListControllers  # type: ignore

    HAVE_LIST_CONTROLLERS = True
except Exception:
    ListControllers = None  # type: ignore
    HAVE_LIST_CONTROLLERS = False


def _is_wsl2() -> bool:
    try:
        with open("/proc/version", "r", encoding="utf-8") as f:
            txt = f.read()
        return "microsoft" in txt.lower()
    except Exception:
        return False


def _param_float(node: Node, name: str, default: float) -> float:
    try:
        v = node.get_parameter(name).value
        if v is None:
            return float(default)
        return float(v)
    except Exception:
        return float(default)


def _param_int(node: Node, name: str, default: int) -> int:
    try:
        v = node.get_parameter(name).value
        if v is None:
            return int(default)
        return int(v)
    except Exception:
        return int(default)


def _param_bool(node: Node, name: str, default: bool) -> bool:
    try:
        v = node.get_parameter(name).value
        if v is None:
            return bool(default)
        return bool(v)
    except Exception:
        return bool(default)


@dataclass
class StreamStats:
    last_wall: Optional[float] = None
    count: int = 0
    dt_window: Deque[float] = None  # type: ignore

    def ensure(self, window_size: int) -> None:
        if self.dt_window is None:
            self.dt_window = deque(maxlen=window_size)

    def update(self, now_wall: float, window_size: int) -> None:
        self.ensure(window_size)
        if self.last_wall is not None:
            dt = now_wall - self.last_wall
            if dt >= 0.0:
                self.dt_window.append(dt)
        self.last_wall = now_wall
        self.count += 1

    def hz_p50(self) -> float:
        if not self.dt_window:
            return 0.0
        dts = sorted(self.dt_window)
        p50 = dts[len(dts) // 2]
        if p50 <= 1e-9:
            return 0.0
        return 1.0 / p50


class StallDiagnoser(Node):
    def __init__(self) -> None:
        super().__init__("stall_diagnoser")

        # Wall-clock thresholds (seconds)
        self.declare_parameter("warn_no_clock_s", 0.8)
        self.declare_parameter("warn_no_joint_states_s", 0.8)
        self.declare_parameter("warn_no_motion_s", 0.8)
        self.declare_parameter("repeat_warn_s", 1.0)

        # Motion detection
        self.declare_parameter("motion_eps_rad", 1e-4)

        # Reporting
        self.declare_parameter("dt_window_size", 200)
        self.declare_parameter("status_period_s", 5.0)

        # ROS-time sanity checks (to explain negative min-dt in `ros2 topic hz`)
        # These thresholds are in seconds of ROS time.
        self.declare_parameter("warn_clock_backwards_s", 0.001)
        self.declare_parameter("warn_clock_jump_forward_s", 0.25)
        self.declare_parameter("warn_js_stamp_backwards_s", 0.001)
        self.declare_parameter("warn_js_stamp_jump_forward_s", 0.25)

        # Optional controller_manager introspection
        self.declare_parameter("enable_list_controllers", True)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("list_controllers_period_s", 5.0)
        self.declare_parameter("list_controllers_call_timeout_s", 1.0)

        self._t0 = time.monotonic()

        window_size = _param_int(self, "dt_window_size", 200)
        if window_size < 20:
            window_size = 20
        self._window_size = window_size

        self._clock = StreamStats()
        self._js = StreamStats()

        self._last_clock_ros_ns: Optional[int] = None
        self._last_js_stamp_ros_ns: Optional[int] = None
        self._clock_backwards_count: int = 0
        self._clock_forward_jump_count: int = 0
        self._js_backwards_count: int = 0
        self._js_forward_jump_count: int = 0

        self._last_positions: Dict[str, float] = {}
        self._last_motion_wall: Optional[float] = None

        self._last_warn_wall: Optional[float] = None
        self._last_status_wall: float = time.monotonic()
        self._last_list_wall: float = 0.0

        # Track stall intervals so we can log explicit "RESUMED" events with duration.
        self._stall_clock_since: Optional[float] = None
        self._stall_js_since: Optional[float] = None
        self._stall_both_since: Optional[float] = None

        self._sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 50)
        self._sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 50
        )

        self._list_cli = None
        if _param_bool(self, "enable_list_controllers", True) and HAVE_LIST_CONTROLLERS:
            cm = str(self.get_parameter("controller_manager").value)
            self._list_cli = self.create_client(ListControllers, f"{cm}/list_controllers")

        if _is_wsl2():
            self.get_logger().warn(
                self._ts() + "WSL2 detected (/proc/version contains Microsoft). "
                "This diagnoser avoids ROS timers to reduce /clock-related silence."
            )

        self.get_logger().info(
            self._ts()
            + "Running. Classifies stalls as A (stream freeze) vs B (motion stall with streams alive)."
        )

    def _ts(self, now_wall: Optional[float] = None) -> str:
        now = time.monotonic() if now_wall is None else now_wall
        return f"[t={now - self._t0:.3f}s] "

    def _on_clock(self, msg: Clock) -> None:
        now = time.monotonic()
        self._clock.update(now, self._window_size)

        # Track ROS time monotonicity on /clock.
        try:
            ros_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        except Exception:
            return

        if self._last_clock_ros_ns is not None:
            dt_ros_s = (ros_ns - self._last_clock_ros_ns) / 1e9

            back_thr = _param_float(self, "warn_clock_backwards_s", 0.001)
            fwd_thr = _param_float(self, "warn_clock_jump_forward_s", 0.25)
            if dt_ros_s < -abs(back_thr):
                self._clock_backwards_count += 1
                self.get_logger().warn(
                    self._ts(now)
                    + f"/clock ROS time jumped BACKWARDS by {(-dt_ros_s):.6f}s (count={self._clock_backwards_count})"
                )
            elif dt_ros_s > max(0.0, fwd_thr):
                self._clock_forward_jump_count += 1
                self.get_logger().warn(
                    self._ts(now)
                    + f"/clock ROS time jumped FORWARD by {dt_ros_s:.6f}s (count={self._clock_forward_jump_count})"
                )

        self._last_clock_ros_ns = ros_ns

    def _on_joint_states(self, msg: JointState) -> None:
        now = time.monotonic()
        self._js.update(now, self._window_size)

        # Track ROS time monotonicity on joint_states stamps.
        try:
            stamp = msg.header.stamp
            ros_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except Exception:
            ros_ns = None

        if ros_ns is not None and self._last_js_stamp_ros_ns is not None:
            dt_ros_s = (ros_ns - self._last_js_stamp_ros_ns) / 1e9

            back_thr = _param_float(self, "warn_js_stamp_backwards_s", 0.001)
            fwd_thr = _param_float(self, "warn_js_stamp_jump_forward_s", 0.25)
            if dt_ros_s < -abs(back_thr):
                self._js_backwards_count += 1
                self.get_logger().warn(
                    self._ts(now)
                    + f"/joint_states stamp jumped BACKWARDS by {(-dt_ros_s):.6f}s (count={self._js_backwards_count})"
                )
            elif dt_ros_s > max(0.0, fwd_thr):
                self._js_forward_jump_count += 1
                self.get_logger().warn(
                    self._ts(now)
                    + f"/joint_states stamp jumped FORWARD by {dt_ros_s:.6f}s (count={self._js_forward_jump_count})"
                )

        if ros_ns is not None:
            self._last_js_stamp_ros_ns = ros_ns

        # Motion detection: update last_motion_wall when any joint changes by > eps.
        eps = _param_float(self, "motion_eps_rad", 1e-4)
        if eps <= 0.0:
            eps = 1e-4

        prev = self._last_positions
        max_abs_dq = 0.0
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                try:
                    pf = float(pos)
                except Exception:
                    continue
                if not math.isfinite(pf):
                    continue
                if name in prev:
                    max_abs_dq = max(max_abs_dq, abs(pf - prev[name]))

        if max_abs_dq > eps:
            self._last_motion_wall = now

        if msg.name and msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                try:
                    pf = float(pos)
                except Exception:
                    continue
                if math.isfinite(pf):
                    self._last_positions[name] = pf

        # If first message arrives, initialize motion timestamp.
        if self._last_motion_wall is None:
            self._last_motion_wall = now

    def _age(self, last_wall: Optional[float], now: float) -> float:
        if last_wall is None:
            return float("inf")
        return max(0.0, now - last_wall)

    def _should_repeat(self, now: float, repeat_s: float) -> bool:
        if self._last_warn_wall is None:
            return True
        if repeat_s <= 0.0:
            return True
        return (now - self._last_warn_wall) >= repeat_s

    def _maybe_list_controllers(self, now: float) -> None:
        if self._list_cli is None:
            return

        period = _param_float(self, "list_controllers_period_s", 5.0)
        if period <= 0.0:
            return
        if (now - self._last_list_wall) < period:
            return

        self._last_list_wall = now

        if not self._list_cli.service_is_ready():
            # Don't block waiting; lack of readiness is also evidence.
            self.get_logger().warn(
                self._ts(now) + "controller_manager list_controllers service not ready"
            )
            return

        timeout = _param_float(self, "list_controllers_call_timeout_s", 1.0)
        if timeout <= 0.0:
            timeout = 1.0

        req = ListControllers.Request()  # type: ignore[attr-defined]
        fut = self._list_cli.call_async(req)

        # We do not use a ROS timer here. We will let the main loop spin the executor.
        deadline = now + timeout
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            # main loop will also spin; this local spin reduces latency for the service future
            rclpy.spin_once(self, timeout_sec=0.05)

        if not fut.done():
            self.get_logger().warn(
                self._ts() + "list_controllers call timed out (wall). Possible CM stall."
            )
            return

        resp = fut.result()
        if resp is None:
            self.get_logger().warn(self._ts() + "list_controllers returned None")
            return

        active = [c.name for c in resp.controller if getattr(c, "state", "") == "active"]
        inactive = [
            f"{c.name}:{getattr(c, 'state', '?')}"
            for c in resp.controller
            if getattr(c, "state", "") != "active"
        ]

        self.get_logger().info(
            self._ts()
            + f"Controllers: active={active} others={inactive[:8]}"
            + (" ..." if len(inactive) > 8 else "")
        )

    def step(self) -> None:
        now = time.monotonic()

        warn_no_clock_s = _param_float(self, "warn_no_clock_s", 0.8)
        warn_no_js_s = _param_float(self, "warn_no_joint_states_s", 0.8)
        warn_no_motion_s = _param_float(self, "warn_no_motion_s", 0.8)
        repeat_s = _param_float(self, "repeat_warn_s", 1.0)

        if warn_no_clock_s <= 0.0:
            warn_no_clock_s = 0.8
        if warn_no_js_s <= 0.0:
            warn_no_js_s = 0.8
        if warn_no_motion_s <= 0.0:
            warn_no_motion_s = 0.8

        age_clock = self._age(self._clock.last_wall, now)
        age_js = self._age(self._js.last_wall, now)
        age_motion = self._age(self._last_motion_wall, now)

        # Stall interval bookkeeping (no timers; pure wall/steady).
        # Start markers
        if age_clock >= warn_no_clock_s and self._stall_clock_since is None:
            self._stall_clock_since = now
        if age_js >= warn_no_js_s and self._stall_js_since is None:
            self._stall_js_since = now
        if (
            age_clock >= warn_no_clock_s
            and age_js >= warn_no_js_s
            and self._stall_both_since is None
        ):
            self._stall_both_since = now

        # Resume markers (use hysteresis to avoid flapping)
        clock_resumed = age_clock < (warn_no_clock_s * 0.5)
        js_resumed = age_js < (warn_no_js_s * 0.5)
        if clock_resumed and self._stall_clock_since is not None:
            dur = now - self._stall_clock_since
            self.get_logger().warn(self._ts(now) + f"/clock RESUMED after {dur:.3f}s wall")
            self._stall_clock_since = None
        if js_resumed and self._stall_js_since is not None:
            dur = now - self._stall_js_since
            self.get_logger().warn(self._ts(now) + f"/joint_states RESUMED after {dur:.3f}s wall")
            self._stall_js_since = None
        if clock_resumed and js_resumed and self._stall_both_since is not None:
            dur = now - self._stall_both_since
            self.get_logger().warn(
                self._ts(now) + f"/clock+ /joint_states RESUMED after {dur:.3f}s wall"
            )
            self._stall_both_since = None

        # Periodic status (proof: stream rates + ages).
        status_period = _param_float(self, "status_period_s", 5.0)
        if status_period > 0.0 and (now - self._last_status_wall) >= status_period:
            self._last_status_wall = now
            self.get_logger().info(
                self._ts(now)
                + "Status: "
                + f"/clock hz≈{self._clock.hz_p50():.1f} age={age_clock:.3f}s "
                + f"| /joint_states hz≈{self._js.hz_p50():.1f} age={age_js:.3f}s "
                + f"| motion_age={age_motion:.3f}s "
                + f"| clock_back={self._clock_backwards_count} clock_fwd={self._clock_forward_jump_count} "
                + f"| js_back={self._js_backwards_count} js_fwd={self._js_forward_jump_count}"
            )

        # Classification warnings (only when something crosses thresholds).
        classification: Optional[Tuple[str, str]] = None

        # A: /clock stalls (often Gazebo/bridge) AND joint_states stalls.
        if age_clock >= warn_no_clock_s and age_js >= warn_no_js_s:
            classification = (
                "A",
                f"/clock and /joint_states silent (ages {age_clock:.3f}s / {age_js:.3f}s). "
                "Likely sim/bridge/executor freeze.",
            )
        # A: /clock continues but joint_states stalls => controller / transport / executor issue.
        elif age_clock < warn_no_clock_s and age_js >= warn_no_js_s:
            classification = (
                "A",
                f"/clock alive (age {age_clock:.3f}s) but /joint_states silent (age {age_js:.3f}s). "
                "Likely controller/transport/executor stall.",
            )
        # B: streams alive but motion stops.
        elif (
            age_clock < warn_no_clock_s and age_js < warn_no_js_s and age_motion >= warn_no_motion_s
        ):
            classification = (
                "B",
                f"Streams alive (/clock age {age_clock:.3f}s, /joint_states age {age_js:.3f}s) "
                f"but motion stalled (no significant joint change for {age_motion:.3f}s).",
            )

        if classification is not None and self._should_repeat(now, repeat_s):
            kind, detail = classification
            self.get_logger().warn(self._ts(now) + f"STALL CLASS={kind}: {detail}")
            self._last_warn_wall = now

        # Optional controller listing (evidence for duplicate controllers / CM responsiveness)
        self._maybe_list_controllers(now)


def main() -> None:
    rclpy.init()
    node = StallDiagnoser()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            # Wall-clock driven spin so it keeps working if /clock stalls.
            ex.spin_once(timeout_sec=0.1)
            node.step()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.remove_node(node)
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
            pass


if __name__ == "__main__":
    main()
