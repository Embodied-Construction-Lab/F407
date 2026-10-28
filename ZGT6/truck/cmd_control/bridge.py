#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jetson UDP-to-STM32 bridge for Logitech wheel data.

Receive raw wheel samples over UDP, normalize them to the truck_control
contract, forward each valid command to STM32, and log every STM32 reply.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO


@dataclass(frozen=True)
class WheelSample:
    timestamp: float
    steering: float
    throttle: float
    brake: float
    up: int
    down: int
    source: str = ""


@dataclass(frozen=True)
class TruckCommand:
    timestamp: float
    source: str
    raw_steering: float
    raw_throttle: float
    raw_reverse: float
    steering: float
    throttle: float
    brake: float
    up: int
    down: int


class LatestSample:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample: Optional[TruckCommand] = None

    def set(self, sample: TruckCommand) -> None:
        with self._lock:
            self._sample = sample

    def get(self) -> Optional[TruckCommand]:
        with self._lock:
            return self._sample


def finite_float(payload: dict, key: str) -> float:
    try:
        value = float(payload[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"字段 {key!r} 缺失或不是数字") from exc
    if not math.isfinite(value):
        raise ValueError(f"字段 {key!r} 不是有限数")
    return value


def button_value(payload: dict, key: str) -> int:
    try:
        value = int(payload[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"字段 {key!r} 缺失或不是按钮值") from exc
    if value not in (0, 1):
        raise ValueError(f"字段 {key!r} 必须为 0 或 1")
    return value


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def raw_pedal_to_unit(raw_axis: float, dead_zone: float) -> float:
    """Convert a pygame pedal axis (+1 released, -1 pressed) to 0..1."""
    value = (1.0 - clamp(raw_axis, -1.0, 1.0)) * 0.5
    return 0.0 if value <= dead_zone else value


def parse_udp_sample(payload: dict, addr: tuple[str, int]) -> WheelSample:
    """Accept logi2.py packets and the equivalent legacy logi_send2 fields."""
    steering = finite_float(payload, "steering")
    throttle = finite_float(payload, "throttle")
    # Omitted reverse pedal and lift buttons mean released, not an invalid frame.
    brake = finite_float(payload, "brake") if "brake" in payload else 1.0

    # logi2.py sends pygame raw values in [-1, 1]. Values are not converted.
    for key, value in (("steering", steering), ("throttle", throttle), ("brake", brake)):
        if not -1.0001 <= value <= 1.0001:
            raise ValueError(f"原始轴 {key!r}={value} 超出 [-1, 1]")

    timestamp = finite_float(payload, "timestamp") if "timestamp" in payload else time.time()
    return WheelSample(
        timestamp=timestamp,
        steering=steering,
        throttle=throttle,
        brake=brake,
        up=button_value(payload, "up") if "up" in payload else 0,
        down=button_value(payload, "down") if "down" in payload else 0,
        source=f"{addr[0]}:{addr[1]}",
    )


def preprocess_wheel_sample(
    sample: WheelSample,
    steering_max_deg: float,
    steering_direction: float,
    pedal_dead_zone: float,
) -> TruckCommand:
    """Convert Logitech raw axes to the truck_control command contract.

    The physical throttle pedal requests forward drive. The physical brake
    pedal keeps the legacy remote_control meaning and requests reverse drive.
    Pressing both pedals is treated as a brake request and therefore neutral.
    """
    forward = raw_pedal_to_unit(sample.throttle, pedal_dead_zone)
    reverse = raw_pedal_to_unit(sample.brake, pedal_dead_zone)
    if forward > 0.0 and reverse > 0.0:
        throttle = 0.0
        brake = 1.0
    else:
        throttle = forward - reverse
        brake = 0.0

    return TruckCommand(
        timestamp=sample.timestamp,
        source=sample.source,
        raw_steering=sample.steering,
        raw_throttle=sample.throttle,
        raw_reverse=sample.brake,
        steering=(
            clamp(sample.steering * steering_direction, -1.0, 1.0)
            * steering_max_deg
        ),
        throttle=clamp(throttle, -1.0, 1.0),
        brake=brake,
        up=sample.up,
        down=sample.down,
    )


def encode_stm32_command(sample: TruckCommand) -> bytes:
    values = {
        "type": "truck_control",
        "timestamp": sample.timestamp,
        "steering": sample.steering,
        "throttle": sample.throttle,
        "brake": sample.brake,
        "up": sample.up,
        "down": sample.down,
    }
    json_bytes = json.dumps(
        values,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")

    # Append a real LF byte (0x0A), not the two visible characters "\n".
    return json_bytes + b"\x0A"


def encode_stm32_button_update(payload: dict) -> bytes:
    """Encode a button-only UDP command for the STM32 incremental parser."""
    values = {"type": "button"}
    if "up" in payload:
        values["up"] = button_value(payload, "up")
    if "down" in payload:
        values["down"] = button_value(payload, "down")
    if len(values) == 1:
        raise ValueError("button update requires 'up' or 'down'")
    return (
        json.dumps(
            values,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
        + b"\x0A"
    )


class Stm32RawLogger:
    HEADER = (
        "host_timestamp",
        "host_datetime",
        "source",
        "wheel_timestamp",
        "raw_steering",
        "raw_throttle",
        "raw_reverse",
        "command_steering_deg",
        "command_throttle",
        "command_brake",
        "up",
        "down",
    )

    def __init__(self, fp: TextIO, flush_interval: float) -> None:
        self.fp = fp
        self.writer = csv.writer(fp)
        self.writer.writerow(self.HEADER)
        self.flush_interval = max(0.0, flush_interval)
        self.last_flush = time.monotonic()
        self.fp.flush()

    @staticmethod
    def display_text(data: bytes) -> str:
        decoded = data.decode("utf-8", errors="backslashreplace").rstrip("\r\n")
        return "".join(
            char if char.isprintable() or char == "\t" else f"\\x{ord(char):02x}"
            for char in decoded
        )

    def write(self, _data: bytes, sample: Optional[TruckCommand]) -> None:
        now = time.time()
        wheel = ("", "", "", "", "", "", "", "", "", "") if sample is None else (
            sample.source,
            sample.timestamp,
            sample.raw_steering,
            sample.raw_throttle,
            sample.raw_reverse,
            sample.steering,
            sample.throttle,
            sample.brake,
            sample.up,
            sample.down,
        )
        self.writer.writerow(
            (
                f"{now:.6f}",
                datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                *wheel,
            )
        )
        if self.flush_interval == 0 or time.monotonic() - self.last_flush >= self.flush_interval:
            self.fp.flush()
            self.last_flush = time.monotonic()


class SerialLineFramer:
    """把任意串口读取块重组为以 LF 结尾的完整消息。"""

    def __init__(self, max_buffer: int = 65536) -> None:
        self.buffer = bytearray()
        self.max_buffer = max(1024, max_buffer)

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []

        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                break
            frames.append(bytes(self.buffer[: newline + 1]))
            del self.buffer[: newline + 1]

        # 防止异常设备长期不发换行导致内存无限增长。该块仍会原样记录。
        if len(self.buffer) >= self.max_buffer:
            frames.append(bytes(self.buffer))
            self.buffer.clear()

        return frames

    def flush(self) -> Optional[bytes]:
        if not self.buffer:
            return None
        remaining = bytes(self.buffer)
        self.buffer.clear()
        return remaining


def serial_reader_loop(
    ser: object,
    logger: Stm32RawLogger,
    latest: LatestSample,
    stop_event: threading.Event,
    poll_interval: float,
) -> None:
    framer = SerialLineFramer()
    try:
        while not stop_event.is_set():
            try:
                waiting = int(getattr(ser, "in_waiting", 0))
                data = ser.read(waiting or 1)
            except (OSError, ValueError) as exc:
                print(f"\nSTM32 串口读取失败：{exc}", file=sys.stderr)
                stop_event.set()
                return

            if not data:
                stop_event.wait(poll_interval)
                continue

            for frame in framer.feed(data):
                logger.write(frame, latest.get())
                shown = Stm32RawLogger.display_text(frame)
                print(f"\nSTM32 RX: {shown if shown else frame.hex(' ')}", flush=True)
    finally:
        remaining = framer.flush()
        if remaining:
            logger.write(remaining, latest.get())
            shown = Stm32RawLogger.display_text(remaining)
            print(
                f"\nSTM32 RX (未完整结束): {shown if shown else remaining.hex(' ')}",
                flush=True,
            )


def make_log_path(output_dir: Path, prefix: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.csv"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "接收UDP原始方向盘数据，预处理为truck_control协议后发送给STM32，"
            "并记录STM32返回数据"
        )
    )
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8000)
    parser.add_argument("--socket-timeout", type=float, default=0.05)
    parser.add_argument("--serial-port", default="/dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial-timeout", type=float, default=0.02)
    parser.add_argument("--serial-poll-interval", type=float, default=0.002)
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--output-prefix", default="bridge_stm32")
    parser.add_argument("--flush-interval", type=float, default=1.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--steering-max-deg", type=float, default=30.0)
    parser.add_argument(
        "--steering-direction", type=float, choices=(-1.0, 1.0), default=1.0
    )
    parser.add_argument("--pedal-dead-zone", type=float, default=0.02)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请执行：pip3 install pyserial") from exc

    stop_event = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_ip, args.listen_port))
    sock.settimeout(max(0.001, args.socket_timeout))

    try:
        ser = serial.Serial(
            port=args.serial_port,
            baudrate=args.baud,
            timeout=max(0.0, args.serial_timeout),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        ser.setDTR(False)
        ser.setRTS(False)
        time.sleep(0.05)
        ser.reset_input_buffer()
    except Exception:
        sock.close()
        raise

    log_path = make_log_path(Path(args.output_dir), args.output_prefix)
    try:
        log_fp = log_path.open("w", newline="", encoding="utf-8")
    except Exception:
        ser.close()
        sock.close()
        raise

    latest = LatestSample()
    logger = Stm32RawLogger(log_fp, args.flush_interval)
    reader = threading.Thread(
        target=serial_reader_loop,
        args=(ser, logger, latest, stop_event, args.serial_poll_interval),
        daemon=True,
    )
    reader.start()

    print(f"UDP：{args.listen_ip}:{args.listen_port}")
    print(
        f"STM32：{args.serial_port} @ {args.baud}，"
        "发送格式：truck_control JSON+LF"
    )
    print(
        f"预处理：方向盘±{max(0.0, args.steering_max_deg):g}°，"
        "油门踏板=前进，制动踏板=倒车，同时踩下=制动中性"
    )
    print(f"STM32 日志：{log_path}")
    print("等待 logi2.py，按 Ctrl+C 停止")

    print_period = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
    last_print = 0.0
    try:
        while not stop_event.is_set():
            try:
                packet, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"UDP 接收失败：{exc}", file=sys.stderr)
                break

            try:
                payload = json.loads(packet.decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("UDP JSON 顶层必须是对象")
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                print(f"丢弃来自 {addr} 的无效 UDP 数据：{exc}", file=sys.stderr)
                continue

            if payload.get("type") == "handshake":
                response = {
                    "status": "ready",
                    "protocol": "truck_control_preprocessed_v1",
                }
                sock.sendto(json.dumps(response).encode("utf-8"), addr)
                print(f"握手完成：{addr}")
                continue

            is_button_update = (
                ("up" in payload or "down" in payload)
                and not any(
                    key in payload for key in ("steering", "throttle", "brake")
                )
            )
            if is_button_update:
                try:
                    frame = encode_stm32_button_update(payload)
                    written = ser.write(frame)
                    if written != len(frame):
                        raise OSError(
                            f"按钮串口发送不完整：期望 {len(frame)} 字节，"
                            f"实际 {written} 字节"
                        )
                except (OSError, ValueError) as exc:
                    print(f"STM32 按钮发送失败：{exc}", file=sys.stderr)
                    continue
                print(
                    f"\nBUTTON {addr[0]}:{addr[1]} "
                    f"up={payload.get('up', '-')} "
                    f"down={payload.get('down', '-')} "
                    f"STM32_TX={written}B",
                    flush=True,
                )
                continue

            try:
                raw_sample = parse_udp_sample(payload, addr)
                sample = preprocess_wheel_sample(
                    raw_sample,
                    steering_max_deg=max(0.0, args.steering_max_deg),
                    steering_direction=args.steering_direction,
                    pedal_dead_zone=clamp(args.pedal_dead_zone, 0.0, 0.25),
                )
            except ValueError as exc:
                print(f"丢弃来自 {addr} 的方向盘数据：{exc}", file=sys.stderr)
                continue

            latest.set(sample)
            try:
                frame = encode_stm32_command(sample)
                written = ser.write(frame)
                if written != len(frame):
                    raise OSError(
                        f"串口发送不完整：期望 {len(frame)} 字节，"
                        f"实际 {written} 字节"
                    )
            except (OSError, ValueError) as exc:
                print(f"STM32 串口发送失败：{exc}", file=sys.stderr)
                stop_event.set()
                break

            now = time.monotonic()
            if print_period and now - last_print >= print_period:
                print(
                    f"\rRX {sample.source} "
                    f"raw=({sample.raw_steering:+.3f},"
                    f"{sample.raw_throttle:+.3f},{sample.raw_reverse:+.3f}) "
                    f"cmd=({sample.steering:+.1f}deg,"
                    f"{sample.throttle:+.3f},B{sample.brake:.0f}) "
                    f"up={sample.up} down={sample.down} "
                    f"STM32_TX={written}B tail={frame[-2:].hex(' ')}   ",
                    end="",
                    flush=True,
                )
                last_print = now
    finally:
        stop_event.set()
        reader.join(timeout=2.0)
        try:
            ser.close()
        finally:
            sock.close()
            log_fp.flush()
            log_fp.close()
        print("\nbridge 已停止，串口、UDP 和日志文件已关闭。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
