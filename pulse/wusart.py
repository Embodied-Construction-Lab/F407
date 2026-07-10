#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read STM32/lower-controller telemetry from Jetson NX THS0 and save it as CSV.

Expected serial text format:
    t,s_boom,s_stick,s_bucket,v_boom,v_stick,v_bucket,a_boom,a_stick,a_bucket,yaw,yaw_rate
    288,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000
"""

from __future__ import annotations

import argparse
import csv
import re
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, TextIO


SERIAL_PREFIX = "STM32 RX:"


class SerialCsvLogger:
    def __init__(
        self,
        fp: TextIO,
        flush_interval: float = 0.0,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._fp = fp
        self._writer = csv.writer(fp)
        self._flush_interval = flush_interval
        self._clock = clock
        self._last_flush = time.monotonic()
        self._header: Optional[list[str]] = None
        self._raw_header_written = False

    def feed_line(self, line: str) -> bool:
        line = clean_serial_line(line)
        if not line:
            return False

        fields = parse_csv_fields(line)
        if fields is None:
            self._write_raw_line(line)
            return True

        if is_csv_header(fields):
            if self._header != fields:
                self._header = fields
                self._writer.writerow(fields)
                self._flush()
            return False

        if self._header is None:
            self._header = [f"field_{idx + 1}" for idx in range(len(fields))]
            self._writer.writerow(self._header)

        self._writer.writerow(pad_fields(fields, len(self._header)))
        self._flush_if_due()
        return True

    def close(self) -> None:
        self._fp.flush()

    def _write_raw_line(self, line: str) -> None:
        if not self._raw_header_written:
            self._writer.writerow(["host_time", "raw_line"])
            self._raw_header_written = True
        self._writer.writerow([f"{self._clock():.6f}", line])
        self._flush_if_due()

    def _flush_if_due(self) -> None:
        if self._flush_interval <= 0:
            self._flush()
            return

        now = time.monotonic()
        if now - self._last_flush >= self._flush_interval:
            self._flush()
            self._last_flush = now

    def _flush(self) -> None:
        self._fp.flush()


def clean_serial_line(line: str) -> str:
    line = line.strip()
    if line.startswith(SERIAL_PREFIX):
        line = line[len(SERIAL_PREFIX) :].strip()
    return line


def parse_csv_fields(line: str) -> Optional[list[str]]:
    if "," not in line:
        return None

    try:
        fields = [field.strip() for field in next(csv.reader([line]))]
    except csv.Error:
        return None

    return fields if fields else None


def is_csv_header(fields: list[str]) -> bool:
    return any(re.search(r"[A-Za-z_]", field) for field in fields)


def pad_fields(fields: list[str], length: int) -> list[str]:
    if len(fields) >= length:
        return fields[:length]
    return [*fields, *([""] * (length - len(fields)))]


def make_csv_path(output_dir: Path, prefix: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"{prefix}_{stamp}.csv"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read Jetson NX /dev/ttyTHS0 serial data from a lower controller and save it to CSV."
    )
    parser.add_argument("--serial-port", default="/dev/ttyTHS0", help="Serial port. Default: /dev/ttyTHS0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate. Default: 115200")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial readline timeout in seconds. Default: 1.0")
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent),
        help="CSV output directory. Default: script directory.",
    )
    parser.add_argument("--output-prefix", default="wusart_log", help="CSV filename prefix. Default: wusart_log")
    parser.add_argument("--flush-interval", type=float, default=0.0, help="CSV flush interval. Default: every row.")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    stop = False

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install pyserial") from exc

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    csv_path = make_csv_path(Path(args.output_dir), args.output_prefix)
    print(f"Opening serial: {args.serial_port} @ {args.baud}")
    print(f"CSV logging to: {csv_path}")

    ser = serial.Serial(
        port=args.serial_port,
        baudrate=args.baud,
        timeout=args.timeout,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(0.05)
    ser.reset_input_buffer()

    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        logger = SerialCsvLogger(fp, args.flush_interval)
        try:
            while not stop:
                raw_line = ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                print(f"STM32 RX: {line}", flush=True)
                logger.feed_line(line)
        finally:
            logger.close()
            ser.close()
            print("Stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
