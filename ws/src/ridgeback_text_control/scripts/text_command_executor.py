#!/usr/bin/env python3
"""Bounded text-command executor for a Ridgeback.

This intentionally accepts a tiny command language. An LLM can be added later
by making it emit the same command strings or a matching structured schema.
"""

import argparse
import json
import math
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass(frozen=True)
class SafetyConfig:
    max_distance_m: float = 1.0
    max_angle_deg: float = 90.0
    linear_speed_mps: float = 0.12
    angular_speed_radps: float = 0.35
    timeout_s: float = 12.0
    odom_timeout_s: float = 1.0


@dataclass(frozen=True)
class Command:
    kind: str
    value: float = 0.0
    direction: int = 1
    raw: str = ""


class CommandError(ValueError):
    pass


FORWARD_WORDS = {"forward", "forwards", "ahead"}
BACK_WORDS = {"back", "backward", "backwards", "reverse"}
LEFT_WORDS = {"left", "ccw", "counterclockwise"}
RIGHT_WORDS = {"right", "cw", "clockwise"}


DISTANCE_RE = re.compile(
    r"^(?:move\s+)?(?P<dir>forward|forwards|ahead|back|backward|backwards|reverse)"
    r"(?:\s+for)?\s+(?P<value>\d+(?:\.\d+)?)\s*(?P<unit>m|meter|meters|cm|centimeter|centimeters)?$"
)
DISTANCE_RE_REVERSED = re.compile(
    r"^(?:move\s+)?(?P<value>\d+(?:\.\d+)?)\s*(?P<unit>m|meter|meters|cm|centimeter|centimeters)?"
    r"\s+(?P<dir>forward|forwards|ahead|back|backward|backwards|reverse)$"
)
TURN_RE = re.compile(
    r"^(?:turn|rotate|spin)?\s*(?P<dir>left|right|ccw|cw|clockwise|counterclockwise)"
    r"\s+(?P<value>\d+(?:\.\d+)?)\s*(?P<unit>deg|degree|degrees|rad|radian|radians)?$"
)
TURN_RE_REVERSED = re.compile(
    r"^(?:turn|rotate|spin)\s+(?P<value>\d+(?:\.\d+)?)\s*"
    r"(?P<unit>deg|degree|degrees|rad|radian|radians)?\s+(?P<dir>left|right|ccw|cw|clockwise|counterclockwise)$"
)


def parse_command(text: str, safety: SafetyConfig) -> Command:
    raw = text.strip()
    normalized = " ".join(raw.lower().split())
    if not normalized:
        raise CommandError("Empty command.")

    if normalized in {"stop", "halt", "brake"}:
        return Command(kind="stop", raw=raw)
    if normalized in {"status", "state"}:
        return Command(kind="status", raw=raw)
    if normalized in {"help", "?"}:
        return Command(kind="help", raw=raw)
    if normalized in {"quit", "exit"}:
        return Command(kind="quit", raw=raw)

    if normalized.startswith("{"):
        return parse_json_command(normalized, safety)

    match = DISTANCE_RE.match(normalized) or DISTANCE_RE_REVERSED.match(normalized)
    if match:
        distance_m = float(match.group("value"))
        unit = match.group("unit") or "m"
        if unit.startswith("cm") or unit.startswith("centimeter"):
            distance_m /= 100.0
        if distance_m <= 0:
            raise CommandError("Distance must be positive.")
        if distance_m > safety.max_distance_m:
            raise CommandError(
                f"Distance {distance_m:.2f} m exceeds safety limit "
                f"{safety.max_distance_m:.2f} m."
            )
        direction_word = match.group("dir")
        direction = -1 if direction_word in BACK_WORDS else 1
        return Command(kind="move", value=distance_m, direction=direction, raw=raw)

    match = TURN_RE.match(normalized) or TURN_RE_REVERSED.match(normalized)
    if match:
        angle = float(match.group("value"))
        unit = match.group("unit") or "deg"
        angle_deg = math.degrees(angle) if unit.startswith("rad") else angle
        if angle_deg <= 0:
            raise CommandError("Angle must be positive.")
        if angle_deg > safety.max_angle_deg:
            raise CommandError(
                f"Angle {angle_deg:.1f} deg exceeds safety limit "
                f"{safety.max_angle_deg:.1f} deg."
            )
        direction_word = match.group("dir")
        direction = -1 if direction_word in RIGHT_WORDS else 1
        return Command(kind="turn", value=math.radians(angle_deg), direction=direction, raw=raw)

    raise CommandError(
        "I can only run: stop, status, forward 0.5m, back 20cm, "
        "turn left 45deg, turn right 90deg."
    )


def parse_json_command(payload: Any, safety: SafetyConfig) -> Command:
    if isinstance(payload, str):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise CommandError(f"Invalid JSON command: {exc}") from exc
    elif isinstance(payload, dict):
        data = payload
    else:
        raise CommandError("JSON command must be an object.")

    if not isinstance(data, dict):
        raise CommandError("JSON command must be an object.")

    allowed = {
        "action",
        "direction",
        "distance_m",
        "distance_cm",
        "angle_deg",
        "angle_rad",
        "reason",
    }
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise CommandError(f"JSON command contains unsupported fields: {', '.join(unknown)}")

    action = _required_string(data, "action").lower()
    raw = json.dumps(data, sort_keys=True)

    if action in {"stop", "halt", "brake"}:
        return Command(kind="stop", raw=raw)
    if action in {"status", "state"}:
        return Command(kind="status", raw=raw)

    if action in {"move", "drive"}:
        direction_word = _required_string(data, "direction").lower()
        if direction_word in FORWARD_WORDS:
            direction = 1
        elif direction_word in BACK_WORDS:
            direction = -1
        else:
            raise CommandError("Move direction must be forward or back.")

        has_m = "distance_m" in data
        has_cm = "distance_cm" in data
        if has_m == has_cm:
            raise CommandError("Move command needs exactly one of distance_m or distance_cm.")
        distance_m = _positive_number(data["distance_m"], "distance_m") if has_m else (
            _positive_number(data["distance_cm"], "distance_cm") / 100.0
        )
        if distance_m > safety.max_distance_m:
            raise CommandError(
                f"Distance {distance_m:.2f} m exceeds safety limit "
                f"{safety.max_distance_m:.2f} m."
            )
        return Command(kind="move", value=distance_m, direction=direction, raw=raw)

    if action in {"turn", "rotate", "spin"}:
        direction_word = _required_string(data, "direction").lower()
        if direction_word in LEFT_WORDS:
            direction = 1
        elif direction_word in RIGHT_WORDS:
            direction = -1
        else:
            raise CommandError("Turn direction must be left or right.")

        has_deg = "angle_deg" in data
        has_rad = "angle_rad" in data
        if has_deg == has_rad:
            raise CommandError("Turn command needs exactly one of angle_deg or angle_rad.")
        angle_rad = math.radians(_positive_number(data["angle_deg"], "angle_deg")) if has_deg else (
            _positive_number(data["angle_rad"], "angle_rad")
        )
        angle_deg = math.degrees(angle_rad)
        if angle_deg > safety.max_angle_deg:
            raise CommandError(
                f"Angle {angle_deg:.1f} deg exceeds safety limit "
                f"{safety.max_angle_deg:.1f} deg."
            )
        return Command(kind="turn", value=angle_rad, direction=direction, raw=raw)

    raise CommandError("JSON action must be one of: move, turn, stop, status.")


def _required_string(data: dict, key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CommandError(f"JSON field {key} must be a non-empty string.")
    return value.strip()


def _positive_number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandError(f"JSON field {key} must be a number.")
    number = float(value)
    if number <= 0:
        raise CommandError(f"JSON field {key} must be positive.")
    return number


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(previous: float, current: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class TextCommandExecutor:
    def __init__(self, args: argparse.Namespace, safety: SafetyConfig):
        self.args = args
        self.safety = safety
        self._lock = threading.Lock()
        self._odom: Optional[Odometry] = None
        self._odom_time: Optional[float] = None
        self._stop_requested = False

        self._pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self._on_odom, queue_size=1)

        signal.signal(signal.SIGINT, self._handle_sigint)
        signal.signal(signal.SIGTERM, self._handle_sigint)

        # Give the publisher a moment to connect, then make startup explicitly still.
        rospy.sleep(0.3)
        self.stop_robot()

    def _handle_sigint(self, signum, frame):
        self._stop_requested = True
        self.stop_robot()
        rospy.signal_shutdown(f"signal {signum}")

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom = msg
            self._odom_time = time.monotonic()

    def latest_odom(self) -> Optional[Odometry]:
        with self._lock:
            return self._odom

    def odom_age(self) -> Optional[float]:
        with self._lock:
            if self._odom_time is None:
                return None
            return time.monotonic() - self._odom_time

    def wait_for_fresh_odom(self) -> Odometry:
        deadline = time.monotonic() + self.safety.odom_timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                return self.require_fresh_odom()
            except CommandError:
                pass
            rospy.sleep(0.05)
        raise CommandError(
            f"No fresh odometry on {self.args.odom_topic}. Refusing motion."
        )

    def require_fresh_odom(self) -> Odometry:
        odom = self.latest_odom()
        age = self.odom_age()
        if odom is None or age is None or age > self.safety.odom_timeout_s:
            raise CommandError(
                f"No fresh odometry on {self.args.odom_topic}. Stopping."
            )
        return odom

    def stop_robot(self):
        self._pub.publish(Twist())

    def run_command(self, command: Command):
        if command.kind == "stop":
            self._stop_requested = True
            self.stop_robot()
            print("Stopped.")
            return
        if command.kind == "status":
            self.print_status()
            return
        if command.kind == "help":
            print_help()
            return
        if command.kind == "quit":
            raise EOFError

        self._stop_requested = False
        if command.kind == "move":
            self.move_distance(command.value, command.direction)
            return
        if command.kind == "turn":
            self.turn_angle(command.value, command.direction)
            return
        raise CommandError(f"Unsupported command kind: {command.kind}")

    def move_distance(self, target_m: float, direction: int):
        start = self.wait_for_fresh_odom()
        start_pos = start.pose.pose.position
        start_x = start_pos.x
        start_y = start_pos.y
        twist = Twist()
        twist.linear.x = direction * self.safety.linear_speed_mps

        rate = rospy.Rate(self.args.rate_hz)
        deadline = time.monotonic() + self.safety.timeout_s
        travelled = 0.0

        print(
            f"Moving {'forward' if direction > 0 else 'back'} {target_m:.2f} m "
            f"at {self.safety.linear_speed_mps:.2f} m/s."
        )
        try:
            while not rospy.is_shutdown() and not self._stop_requested:
                odom = self.require_fresh_odom()
                pos = odom.pose.pose.position
                travelled = math.hypot(pos.x - start_x, pos.y - start_y)
                if travelled >= target_m:
                    break
                if time.monotonic() >= deadline:
                    raise CommandError(
                        f"Timed out after travelling about {travelled:.2f} m."
                    )
                self._pub.publish(twist)
                rate.sleep()
        finally:
            self.stop_robot()

        print(f"Done. Odometry distance: {travelled:.2f} m.")

    def turn_angle(self, target_rad: float, direction: int):
        start = self.wait_for_fresh_odom()
        previous_yaw = yaw_from_odom(start)
        accumulated = 0.0

        twist = Twist()
        twist.angular.z = direction * self.safety.angular_speed_radps

        rate = rospy.Rate(self.args.rate_hz)
        deadline = time.monotonic() + self.safety.timeout_s

        print(
            f"Turning {'left' if direction > 0 else 'right'} "
            f"{math.degrees(target_rad):.1f} deg at "
            f"{self.safety.angular_speed_radps:.2f} rad/s."
        )
        try:
            while not rospy.is_shutdown() and not self._stop_requested:
                odom = self.require_fresh_odom()
                current_yaw = yaw_from_odom(odom)
                accumulated += shortest_angle_delta(previous_yaw, current_yaw)
                previous_yaw = current_yaw
                if abs(accumulated) >= target_rad:
                    break
                if time.monotonic() >= deadline:
                    raise CommandError(
                        f"Timed out after turning about {math.degrees(abs(accumulated)):.1f} deg."
                    )
                self._pub.publish(twist)
                rate.sleep()
        finally:
            self.stop_robot()

        print(f"Done. Odometry angle: {math.degrees(abs(accumulated)):.1f} deg.")

    def print_status(self):
        odom = self.latest_odom()
        age = self.odom_age()
        if odom is None or age is None:
            print(f"No odometry received yet on {self.args.odom_topic}.")
            return
        pos = odom.pose.pose.position
        yaw_deg = math.degrees(yaw_from_odom(odom))
        print(
            f"Odom age: {age:.2f}s | x: {pos.x:.2f} m | y: {pos.y:.2f} m | "
            f"yaw: {yaw_deg:.1f} deg"
        )


def print_help():
    print(
        "Commands:\n"
        "  status\n"
        "  stop\n"
        "  forward 0.5m\n"
        "  back 20cm\n"
        "  turn left 45deg\n"
        "  turn right 90deg\n"
        "  quit\n"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Bounded text-command safety executor for Ridgeback /cmd_vel."
    )
    parser.add_argument("command", nargs="*", help="Optional one-shot command.")
    parser.add_argument("--dry-run", action="store_true", help="Parse only; do not start ROS or publish.")
    parser.add_argument("--json", action="store_true", help="Parse the one-shot command as JSON.")
    parser.add_argument("--cmd-topic", default="/cmd_vel", help="Velocity command topic.")
    parser.add_argument("--odom-topic", default="/odometry/filtered", help="Odometry topic used for bounds.")
    parser.add_argument("--max-distance", type=float, default=1.0, help="Max distance per command, meters.")
    parser.add_argument("--max-angle", type=float, default=90.0, help="Max angle per command, degrees.")
    parser.add_argument("--linear-speed", type=float, default=0.12, help="Linear speed, m/s.")
    parser.add_argument("--angular-speed", type=float, default=0.35, help="Angular speed, rad/s.")
    parser.add_argument("--timeout", type=float, default=12.0, help="Max seconds per motion command.")
    parser.add_argument("--odom-timeout", type=float, default=1.0, help="Fresh odom timeout, seconds.")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish/control loop rate.")
    return parser


def safety_from_args(args: argparse.Namespace) -> SafetyConfig:
    return SafetyConfig(
        max_distance_m=args.max_distance,
        max_angle_deg=args.max_angle,
        linear_speed_mps=args.linear_speed,
        angular_speed_radps=args.angular_speed,
        timeout_s=args.timeout,
        odom_timeout_s=args.odom_timeout,
    )


def describe_command(command: Command) -> str:
    if command.kind == "move":
        direction = "forward" if command.direction > 0 else "back"
        return f"move {direction} {command.value:.2f} m"
    if command.kind == "turn":
        direction = "left" if command.direction > 0 else "right"
        return f"turn {direction} {math.degrees(command.value):.1f} deg"
    return command.kind


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    safety = safety_from_args(args)

    one_shot = " ".join(args.command).strip()
    if args.dry_run:
        if not one_shot:
            print("Dry run needs a command, for example: --dry-run 'forward 0.5m'")
            return 2
        try:
            command = parse_json_command(one_shot, safety) if args.json else parse_command(one_shot, safety)
        except CommandError as exc:
            print(f"Rejected: {exc}")
            return 1
        print(f"Accepted: {describe_command(command)}")
        return 0

    rospy.init_node("ridgeback_text_command_executor", anonymous=False)
    executor = TextCommandExecutor(args, safety)

    try:
        if one_shot:
            try:
                command = parse_json_command(one_shot, safety) if args.json else parse_command(one_shot, safety)
                executor.run_command(command)
                return 0
            except CommandError as exc:
                executor.stop_robot()
                print(f"Rejected: {exc}")
                return 1

        print("Ridgeback text safety executor is ready.")
        print_help()
        while not rospy.is_shutdown():
            try:
                line = input("ridgeback> ")
                command = parse_command(line, safety)
                executor.run_command(command)
            except CommandError as exc:
                executor.stop_robot()
                print(f"Rejected: {exc}")
            except EOFError:
                break
    finally:
        executor.stop_robot()

    print("Exiting. Robot stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
