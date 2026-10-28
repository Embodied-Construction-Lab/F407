#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read raw Logitech wheel data and send it to bridge.py over UDP.

This lightweight sender does not import evdev and does not enable force
feedback. Use logi2.py when force feedback is required.
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class WheelSample:
    """pygame 原始归一化轴值和按钮状态。"""

    timestamp: float
    steering: float
    throttle: float
    brake: float
    up: int
    down: int


def encode_udp_sample(sample: WheelSample) -> bytes:
    return json.dumps(
        {"type": "logi_raw", **asdict(sample)}, separators=(",", ":")
    ).encode("utf-8")


def verify_bridge(sock: socket.socket, address: tuple[str, int], timeout: float) -> bool:
    """最多发送三次握手，确认 bridge.py 已准备好。"""
    handshake = json.dumps(
        {"type": "handshake", "timestamp": time.time()}, separators=(",", ":")
    ).encode("utf-8")
    old_timeout = sock.gettimeout()
    sock.settimeout(timeout)
    try:
        for _ in range(3):
            sock.sendto(handshake, address)
            try:
                packet, source = sock.recvfrom(1024)
            except socket.timeout:
                continue
            if source != address:
                continue
            response = json.loads(packet.decode("utf-8"))
            if isinstance(response, dict) and response.get("status") == "ready":
                return True
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    finally:
        sock.settimeout(old_timeout)
    return False


def validate_index(name: str, index: int, count: int) -> None:
    if not 0 <= index < count:
        raise RuntimeError(f"{name}序号 {index} 越界，设备只有 {count} 个{name}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="读取 Logitech 原始方向盘数据并通过 UDP 发送给 bridge.py（无力反馈）"
    )
    parser.add_argument(
        "bridge_ip",
        nargs="?",
        default="192.168.2.62",
        help="运行 bridge.py 的 Jetson IP（默认：192.168.2.62）",
    )
    parser.add_argument("--bridge-port", type=int, default=8000)
    parser.add_argument("--handshake-timeout", type=float, default=1.0)
    parser.add_argument("--joystick", type=int, default=0)
    parser.add_argument("--steering-axis", type=int, default=0)
    parser.add_argument("--throttle-axis", type=int, default=1)
    parser.add_argument("--brake-axis", type=int, default=2)
    parser.add_argument("--up-button", type=int, default=3)
    parser.add_argument("--down-button", type=int, default=0)
    parser.add_argument("--rate", type=float, default=100.0, help="方向盘采样频率 Hz")
    parser.add_argument("--send-hz", type=float, default=50.0, help="UDP 发送频率 Hz")
    parser.add_argument("--print-hz", type=float, default=5.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        address = (socket.gethostbyname(args.bridge_ip), args.bridge_port)
    except OSError as exc:
        raise SystemExit(f"无法解析 bridge 地址 {args.bridge_ip!r}：{exc}") from exc

    try:
        import pygame
    except ImportError as exc:
        raise SystemExit("缺少 pygame，请执行：pip3 install pygame") from exc

    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if args.joystick < 0 or args.joystick >= joystick_count:
        pygame.quit()
        raise SystemExit(
            f"找不到 joystick {args.joystick}，当前检测到 {joystick_count} 个设备"
        )

    joystick = pygame.joystick.Joystick(args.joystick)
    joystick.init()
    try:
        for name, index, count in (
            ("轴", args.steering_axis, joystick.get_numaxes()),
            ("轴", args.throttle_axis, joystick.get_numaxes()),
            ("轴", args.brake_axis, joystick.get_numaxes()),
            ("按钮", args.up_button, joystick.get_numbuttons()),
            ("按钮", args.down_button, joystick.get_numbuttons()),
        ):
            validate_index(name, index, count)
    except RuntimeError as exc:
        joystick.quit()
        pygame.quit()
        raise SystemExit(str(exc)) from exc

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if not verify_bridge(sock, address, max(0.1, args.handshake_timeout)):
        sock.close()
        joystick.quit()
        pygame.quit()
        raise SystemExit(f"bridge 握手失败：{address[0]}:{address[1]}")

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print(f"方向盘：{joystick.get_name()}")
    print(f"bridge：{address[0]}:{address[1]}，握手成功")
    print("力反馈：禁用")
    print("按 Ctrl+C 停止")

    sample_period = 1.0 / max(1.0, args.rate)
    send_period = 1.0 / max(1.0, args.send_hz)
    print_period = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
    last_send = 0.0
    last_print = 0.0

    try:
        while running:
            loop_start = time.monotonic()
            pygame.event.pump()
            sample = WheelSample(
                timestamp=time.time(),
                steering=float(joystick.get_axis(args.steering_axis)),
                throttle=float(joystick.get_axis(args.throttle_axis)),
                brake=float(joystick.get_axis(args.brake_axis)),
                up=int(bool(joystick.get_button(args.up_button))),
                down=int(bool(joystick.get_button(args.down_button))),
            )

            now = time.monotonic()
            if now - last_send >= send_period:
                sock.sendto(encode_udp_sample(sample), address)
                last_send = now

            if print_period and now - last_print >= print_period:
                print(
                    f"\rsteering={sample.steering:+.4f} "
                    f"throttle={sample.throttle:+.4f} "
                    f"brake={sample.brake:+.4f} "
                    f"up={sample.up} down={sample.down}   ",
                    end="",
                    flush=True,
                )
                last_print = now

            time.sleep(max(0.0, sample_period - (time.monotonic() - loop_start)))
    except OSError as exc:
        print(f"\nUDP 发送失败：{exc}", file=sys.stderr)
        return 1
    finally:
        sock.close()
        joystick.quit()
        pygame.quit()
        print("\nlogi 已停止。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
