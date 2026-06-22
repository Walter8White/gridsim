"""Keyboard teleop: moves robot on grid, publishes pose.

Keyboard reading runs in a blocking thread (avoids select/TTY issues).
Velocity persists until SPACE or a new key — press W to start moving forward,
SPACE to stop.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32

HELP = """
Teleop keys:
  W / S    forward / backward  (closer / farther from facade)
  A / D    translate left / right
  Q / E    rotate yaw left / right
  R / F    closer / farther (alias W/S)
  SPACE    stop
  Ctrl+C   quit
"""

_KEYS: dict[str, tuple[float, float, float]] = {
    "w": (0.0, -1.0, 0.0),
    "s": (0.0,  1.0, 0.0),
    "r": (0.0, -1.0, 0.0),
    "f": (0.0,  1.0, 0.0),
    "a": (-1.0, 0.0, 0.0),
    "d": ( 1.0, 0.0, 0.0),
    "q": (0.0, 0.0,  1.0),
    "e": (0.0, 0.0, -1.0),
}

_SPEED_M_S     = 0.3
_YAW_SPEED_RAD = 0.5
_DT            = 0.05   # 20 Hz publish rate

_INIT_X   =  0.0
_INIT_Y   =  1.25   # matches facade_standoff_m in grid.yaml
_INIT_YAW =  0.0
_MIN_Y    =  0.2
_MAX_Y    =  8.0


class TeleopRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_robot_node")

        self._x   = _INIT_X
        self._y   = _INIT_Y
        self._yaw = _INIT_YAW
        self._vx  = 0.0
        self._vy  = 0.0
        self._vyaw = 0.0

        self._pub_pose = self.create_publisher(PoseStamped, "/robot/pose", 10)
        self._pub_x    = self.create_publisher(Float32, "/robot/x_position", 10)
        self._pub_y    = self.create_publisher(Float32, "/robot/y_position", 10)
        self._pub_yaw  = self.create_publisher(Float32, "/robot/yaw", 10)

        self._key_q: queue.Queue[str] = queue.Queue()

        # Blocking stdin reader — works regardless of how stdin is connected
        self._stdin_thread = threading.Thread(
            target=self._stdin_reader, daemon=True
        )
        self._stdin_thread.start()

        self.create_timer(_DT, self._update)
        print(HELP, flush=True)
        print(f"Initial pose: x={_INIT_X:.2f}  y={_INIT_Y:.2f}  yaw=0.0", flush=True)

    # ── stdin reader thread ──────────────────────────────────────────────────

    def _stdin_reader(self) -> None:
        """Block on stdin; push each key into _key_q."""
        fd = sys.stdin.fileno()

        # Gracefully handle non-TTY stdin (e.g. piped / pytest)
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            print("[teleop] stdin is not a TTY — key input unavailable", flush=True)
            return

        try:
            tty.setraw(fd)
            while True:
                ch = os.read(fd, 1)
                if not ch:
                    break
                self._key_q.put(ch.decode("utf-8", errors="ignore"))
        except Exception as exc:
            print(f"[teleop] stdin reader stopped: {exc}", flush=True)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── ROS2 timer callback ──────────────────────────────────────────────────

    def _update(self) -> None:
        # Drain queue; last key wins this tick
        key = ""
        while not self._key_q.empty():
            try:
                key = self._key_q.get_nowait()
            except queue.Empty:
                break

        key = key.lower()

        if key == "\x03":           # Ctrl+C
            raise KeyboardInterrupt
        elif key == " ":
            self._vx = self._vy = self._vyaw = 0.0
            print("STOP", flush=True)
        elif key in _KEYS:
            dx, dy, dyaw = _KEYS[key]
            self._vx   = dx   * _SPEED_M_S
            self._vy   = dy   * _SPEED_M_S
            self._vyaw = dyaw * _YAW_SPEED_RAD
            print(f"key={key}  vx={self._vx:.2f}  vy={self._vy:.2f}  vyaw={self._vyaw:.2f}", flush=True)
        # no else: velocity persists until SPACE or a new key

        self._x   += self._vx   * _DT
        self._y    = max(_MIN_Y, min(_MAX_Y, self._y + self._vy * _DT))
        self._yaw += self._vyaw * _DT

        self._publish()

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp    = now
        pose.header.frame_id = "world"
        pose.pose.position.x = self._x
        pose.pose.position.y = self._y
        pose.pose.position.z = 0.0
        half = self._yaw * 0.5
        pose.pose.orientation.w = math.cos(half)
        pose.pose.orientation.z = math.sin(half)
        self._pub_pose.publish(pose)

        self._pub_x.publish(Float32(data=float(self._x)))
        self._pub_y.publish(Float32(data=float(self._y)))
        self._pub_yaw.publish(Float32(data=float(self._yaw)))

    def destroy_node(self) -> None:
        # Restore terminal on clean exit
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeleopRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
