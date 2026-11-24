#!/usr/bin/env python3
"""EWELLIX SCU RS232 driver with conservative ROS command glue.

This implements the public SCU RS232 transport: ASCII two-byte commands,
little-endian parameters, and CRC-16/CCITT with initial value 0.
Motion commands are disabled unless explicitly enabled.
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - useful before the Docker image is rebuilt.
    serial = None


ACK_CODES = {
    0x06: "ACK",
    0x80: "checksum error",
    0x81: "parameter data error",
    0x82: "parameter count error",
    0x83: "invalid command",
    0x84: "permission error",
}


class EwellixError(RuntimeError):
    pass


@dataclass
class Reply:
    command: str
    ack: int
    data: bytes = b""

    @property
    def ok(self) -> bool:
        return self.ack == 0x06

    @property
    def ack_name(self) -> str:
        return ACK_CODES.get(self.ack, f"unknown 0x{self.ack:02x}")


def crc16_ccitt(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def with_crc(payload: bytes) -> bytes:
    crc = crc16_ccitt(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def i16(value: int) -> bytes:
    return int(value).to_bytes(2, "little", signed=True)


def u16(value: int) -> bytes:
    return int(value).to_bytes(2, "little", signed=False)


class EwellixScu:
    def __init__(
        self,
        port: str,
        baud: int = 38400,
        dry_run: bool = True,
        timeout_s: float = 2.0,
    ):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.timeout_s = timeout_s
        self._serial = None
        self._lock = threading.Lock()

    def open(self) -> None:
        if self.dry_run:
            return
        if serial is None:
            raise EwellixError("pyserial is not installed. Rebuild the Docker image.")
        self._serial = serial.Serial(
            self.port,
            self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
        # The SCU manual says DTR and RTS must be permanently active because
        # they supply the RS232 converter in the control unit.
        self._serial.dtr = True
        self._serial.rts = True
        time.sleep(0.05)

    def close(self) -> None:
        if self._serial:
            self._serial.close()
            self._serial = None

    def command(self, name: str, params: bytes = b"", expect_data: bool = False) -> Reply:
        if len(name) != 2:
            raise ValueError("SCU command names are two ASCII characters.")
        request = with_crc(name.encode("ascii") + params)
        if self.dry_run:
            return Reply(command=name, ack=0x06, data=request)

        if self._serial is None:
            raise EwellixError("Serial port is not open.")

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(request)
            self._serial.flush()
            header = self._read_exact(3)
            if header[:2] != name.encode("ascii"):
                raise EwellixError(f"Unexpected reply command bytes: {header[:2]!r}")

            ack = header[2]
            if not expect_data or ack != 0x06:
                crc_bytes = self._read_exact(2)
                self._check_crc(header, crc_bytes)
                return Reply(command=name, ack=ack)

            ctp_bytes = self._read_exact(2)
            ctp = int.from_bytes(ctp_bytes, "little", signed=False)
            data = self._read_exact(ctp)
            crc_bytes = self._read_exact(2)
            self._check_crc(header + ctp_bytes + data, crc_bytes)
            return Reply(command=name, ack=ack, data=data)

    def _read_exact(self, size: int) -> bytes:
        data = self._serial.read(size)
        if len(data) != size:
            raise EwellixError(f"Timed out reading {size} byte(s); got {len(data)}.")
        return data

    @staticmethod
    def _check_crc(payload: bytes, crc_bytes: bytes) -> None:
        expected = crc16_ccitt(payload)
        actual = int.from_bytes(crc_bytes, "little", signed=False)
        if actual != expected:
            raise EwellixError(f"Bad CRC: got 0x{actual:04x}, expected 0x{expected:04x}.")

    def remote_open(self, safety_id: int = 1) -> Reply:
        return self.command("RO", bytes((safety_id & 0xFF,)))

    def remote_close(self) -> Reply:
        return self.command("RA")

    def keepalive(self) -> Reply:
        # RC with ctp=1 and cyclic object index -1: watchdog only, no data.
        return self.command("RC", u16(1) + bytes((0xFF,)))

    def read_data(self, data_index: int) -> Reply:
        return self.command("RG", u16(data_index), expect_data=True)

    def write_u16(self, data_index: int, value: int) -> Reply:
        # RT payload: ctp=4, data_id, uint16 value.
        return self.command("RT", u16(4) + u16(data_index) + u16(value))

    def execute_motion(self, function_id: int, motion_direction: int) -> Reply:
        # para_ID[1] is unused for RE and set to -1.
        return self.command("RE", bytes((function_id & 0xFF, motion_direction & 0xFF, 0xFF)))

    def stop_motion(self, function_id: int, soft_stop: bool = True) -> Reply:
        return self.command("RS", bytes((function_id & 0xFF, 1 if soft_stop else 0)))


class KeepaliveThread:
    def __init__(self, scu: EwellixScu, period_s: float = 0.4):
        self.scu = scu
        self.period_s = period_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.scu.keepalive()
            except Exception:
                pass
            self._stop.wait(self.period_s)


def format_reply(reply: Reply) -> str:
    if reply.ok:
        if reply.data:
            return f"{reply.command}: ACK data={reply.data.hex(' ')}"
        return f"{reply.command}: ACK"
    return f"{reply.command}: {reply.ack_name}"


def parse_function_id(text: str) -> int:
    value = int(text)
    if value < 0 or value > 9:
        raise argparse.ArgumentTypeError("function id must be 0..9 for F1..F10")
    return value


def parse_speed(text: str) -> int:
    value = int(text)
    if value < 0 or value > 100:
        raise argparse.ArgumentTypeError("speed must be 0..100 percent")
    return value


def run_cli(args: argparse.Namespace) -> int:
    scu = EwellixScu(args.port, args.baud, dry_run=args.dry_run, timeout_s=args.timeout)
    scu.open()
    keepalive: Optional[KeepaliveThread] = None
    try:
        if args.action == "crc-self-test":
            payload = bytes.fromhex(args.hex)
            print(f"crc=0x{crc16_ccitt(payload):04x}")
            print(f"frame={with_crc(payload).hex(' ')}")
            return 0

        if args.action == "open":
            print(format_reply(scu.remote_open(args.safety_id)))
            return 0
        if args.action == "close":
            print(format_reply(scu.remote_close()))
            return 0

        print(format_reply(scu.remote_open(args.safety_id)))
        keepalive = KeepaliveThread(scu)
        keepalive.start()
        time.sleep(0.05)

        if args.action == "keepalive":
            print(format_reply(scu.keepalive()))
        elif args.action == "position":
            print(format_reply(scu.read_data(0x0011 + args.actuator - 1)))
        elif args.action == "status":
            print(format_reply(scu.read_data(0x0171 + args.actuator - 1)))
        elif args.action == "stop":
            print(format_reply(scu.stop_motion(args.function_id, soft_stop=True)))
        elif args.action == "move":
            if not args.enable_motion:
                raise EwellixError("Motion is disabled. Add --enable-motion only when the area is clear.")
            direction = {"in": 1, "out": 2}[args.direction]
            speed_index = 0x3011 + args.function_id
            print(format_reply(scu.write_u16(speed_index, args.speed)))
            print(format_reply(scu.execute_motion(args.function_id, direction)))
            time.sleep(args.max_s)
            print(format_reply(scu.stop_motion(args.function_id, soft_stop=True)))
        return 0
    finally:
        if keepalive:
            keepalive.stop()
        if not args.dry_run:
            try:
                scu.remote_close()
            except Exception:
                pass
        scu.close()


def run_ros_node(args: argparse.Namespace) -> int:
    import rospy
    from std_msgs.msg import String

    rospy.init_node("ewellix_scu_rs232")
    port = rospy.get_param("~port", args.port)
    baud = int(rospy.get_param("~baud", args.baud))
    dry_run = bool(rospy.get_param("~dry_run", args.dry_run))
    enable_motion = bool(rospy.get_param("~enable_motion", False))
    safety_id = int(rospy.get_param("~safety_id", 1))
    max_s = float(rospy.get_param("~max_motion_s", 1.0))
    default_speed = int(rospy.get_param("~speed_pct", 20))
    default_function = int(rospy.get_param("~function_id", 0))

    scu = EwellixScu(port, baud, dry_run=dry_run, timeout_s=args.timeout)
    pub = rospy.Publisher("lift/status", String, queue_size=10, latch=True)

    def publish(text: str) -> None:
        rospy.loginfo(text)
        pub.publish(String(data=text))

    def handle(msg: String) -> None:
        parts = msg.data.strip().lower().split()
        if not parts:
            return
        try:
            if parts[0] == "position":
                actuator = int(parts[1]) if len(parts) > 1 else 1
                publish(format_reply(scu.read_data(0x0011 + actuator - 1)))
            elif parts[0] == "status":
                actuator = int(parts[1]) if len(parts) > 1 else 1
                publish(format_reply(scu.read_data(0x0171 + actuator - 1)))
            elif parts[0] == "stop":
                function_id = int(parts[1]) if len(parts) > 1 else default_function
                publish(format_reply(scu.stop_motion(function_id, soft_stop=True)))
            elif parts[0] == "move":
                if not enable_motion:
                    publish("motion disabled; set _enable_motion:=true only after dry checks")
                    return
                direction = parts[1] if len(parts) > 1 else ""
                if direction not in {"in", "out"}:
                    publish("move command must be: move in|out [function_id] [speed_pct] [seconds]")
                    return
                function_id = int(parts[2]) if len(parts) > 2 else default_function
                speed = int(parts[3]) if len(parts) > 3 else default_speed
                duration = float(parts[4]) if len(parts) > 4 else max_s
                duration = min(duration, max_s)
                motion_direction = {"in": 1, "out": 2}[direction]
                publish(format_reply(scu.write_u16(0x3011 + function_id, speed)))
                publish(format_reply(scu.execute_motion(function_id, motion_direction)))
                threading.Timer(duration, lambda: publish(format_reply(scu.stop_motion(function_id)))).start()
            else:
                publish("commands: status [actuator], position [actuator], stop [function_id], move in|out ...")
        except Exception as exc:
            publish(f"lift command failed: {exc}")

    scu.open()
    publish(format_reply(scu.remote_open(safety_id)))
    keepalive = KeepaliveThread(scu)
    keepalive.start()
    rospy.Subscriber("lift/command", String, handle, queue_size=10)
    try:
        rospy.spin()
    finally:
        keepalive.stop()
        if not dry_run:
            try:
                scu.remote_close()
            except Exception:
                pass
        scu.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=38400)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--safety-id", type=int, default=1)
    parser.add_argument("--ros-node", action="store_true")
    sub = parser.add_subparsers(dest="action", required=False)

    sub.add_parser("open")
    sub.add_parser("close")
    sub.add_parser("keepalive")

    position = sub.add_parser("position")
    position.add_argument("actuator", type=int, nargs="?", default=1)

    status = sub.add_parser("status")
    status.add_argument("actuator", type=int, nargs="?", default=1)

    stop = sub.add_parser("stop")
    stop.add_argument("function_id", type=parse_function_id, nargs="?", default=0)

    move = sub.add_parser("move")
    move.add_argument("direction", choices=("in", "out"))
    move.add_argument("function_id", type=parse_function_id, nargs="?", default=0)
    move.add_argument("speed", type=parse_speed, nargs="?", default=20)
    move.add_argument("max_s", type=float, nargs="?", default=1.0)

    crc = sub.add_parser("crc-self-test")
    crc.add_argument("hex", help="payload bytes without CRC, e.g. '52 4f 00'")
    parser.set_defaults(action="status")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.ros_node:
            return run_ros_node(args)
        return run_cli(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
