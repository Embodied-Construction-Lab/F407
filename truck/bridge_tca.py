#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jetson UDP-to-STM32 bridge for Logitech wheel data.

Receive raw wheel samples over UDP, forward each valid sample to STM32 through
the configured serial port, and save every STM32 reply to a timestamped CSV.
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


PROTOCOL_TYPE = "logi_raw"
PROTOCOL_VERSION = "logi_raw_v1"


@dataclass(frozen=True)
class WheelSample:
    timestamp: float
    steering: float
    throttle: float
    brake: float
    up: int
    down: int
    source: str = ""


class LatestSample:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample: Optional[WheelSample] = None

    def set(self, sample: WheelSample) -> None:
        with self._lock:
            self._sample = sample

    def get(self) -> Optional[WheelSample]:
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


def parse_udp_sample(payload: dict, addr: tuple[str, int]) -> WheelSample:
    """Validate one logi_tca.py packet and normalize its control fields."""
    packet_type = payload.get("type")
    if packet_type != PROTOCOL_TYPE:
        raise ValueError(
            f"字段 'type' 必须为 {PROTOCOL_TYPE!r}，实际为 {packet_type!r}"
        )

    steering = finite_float(payload, "steering")
    throttle = finite_float(payload, "throttle")
    brake = finite_float(payload, "brake")

    # logi_tca.py keeps all three control values in the pygame range [-1, 1].
    for key, value in (("steering", steering), ("throttle", throttle), ("brake", brake)):
        if not -1.0001 <= value <= 1.0001:
            raise ValueError(f"原始轴 {key!r}={value} 超出 [-1, 1]")

    timestamp = finite_float(payload, "timestamp") if "timestamp" in payload else time.time()
    return WheelSample(
        timestamp=timestamp,
        steering=steering,
        throttle=throttle,
        brake=brake,
        up=button_value(payload, "up"),
        down=button_value(payload, "down"),
        source=f"{addr[0]}:{addr[1]}",
    )


def control_payload(sample: WheelSample) -> dict:
    """Return the one canonical payload used by UDP and STM32 serial."""
    return {
        "type": PROTOCOL_TYPE,
        "timestamp": sample.timestamp,
        "steering": sample.steering,
        "throttle": sample.throttle,
        "brake": sample.brake,
        "up": sample.up,
        "down": sample.down,
    }


def encode_control_payload(sample: WheelSample, append_lf: bool) -> bytes:
    """Encode the canonical control payload.

    logi_tca.py sends the JSON bytes over UDP without a terminator. STM32
    receives the same JSON bytes followed by one LF byte for serial framing.
    """
    json_bytes = json.dumps(
        control_payload(sample),
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")
    return json_bytes + (b"\x0A" if append_lf else b"")


def encode_stm32_command(sample: WheelSample) -> bytes:
    # Append a real LF byte (0x0A), not the two visible characters "\n".
    return encode_control_payload(sample, append_lf=True)


class Stm32RawLogger:
    HEADER = (
        "host_timestamp",
        "host_datetime",
        "source",
        "wheel_timestamp",
        "steering",
        "throttle",
        "brake",
        "up",
        "down",
        "stm32_text",
        "stm32_hex",
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

    def write(self, data: bytes, sample: Optional[WheelSample]) -> None:
        now = time.time()
        wheel = ("", "", "", "", "", "", "") if sample is None else (
            sample.source,
            sample.timestamp,
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
                self.display_text(data),
                data.hex(" "),
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
            "接收 logi_tca.py 的统一控制 JSON，经指定串口原样发送给 STM32，"
            "并记录 STM32 返回数据"
        )
    )
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8888)
    parser.add_argument("--socket-timeout", type=float, default=0.05)
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial-timeout", type=float, default=0.02)
    parser.add_argument("--serial-poll-interval", type=float, default=0.002)
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--output-prefix", default="bridge_tca_stm32")
    parser.add_argument("--flush-interval", type=float, default=1.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
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
    print(f"STM32：{args.serial_port} @ {args.baud}，发送格式：JSON+LF")
    print(f"STM32 日志：{log_path}")
    print(f"统一协议：{PROTOCOL_VERSION}")
    print("等待 logi_tca.py，按 Ctrl+C 停止")

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
                response = {"status": "ready", "protocol": PROTOCOL_VERSION}
                sock.sendto(json.dumps(response).encode("utf-8"), addr)
                print(f"握手完成：{addr}")
                continue

            try:
                sample = parse_udp_sample(payload, addr)
            except ValueError as exc:
                print(f"丢弃来自 {addr} 的方向盘数据：{exc}", file=sys.stderr)
                continue

            latest.set(sample)
            try:
                frame = encode_stm32_command(sample)
                written = ser.write(frame)
                ser.flush()
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
                tx_json = frame[:-1].decode("ascii")
                print(
                    f"\rUDP→STM32 {sample.source} {tx_json} "
                    f"[{written}B, LF=0a]   ",
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
