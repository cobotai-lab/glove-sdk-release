#!/usr/bin/env python3
"""Glovity integrated magnetometer calibration helper.

The product glove firmware exposes magnetometer calibration over the glove USB
CDC text port. This script can either use a user-provided port, or scan all
serial ports and identify left/right gloves from the firmware identity/status
text. The receiver outputs binary frames and is intentionally ignored.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Iterable

import serial
from serial.tools import list_ports


DEFAULT_BAUD = 921600
DEFAULT_DURATION_S = 35.0
COMMAND_DELAY_S = 0.15


@dataclass
class DeviceInfo:
    port: str
    kind: str = "unknown"
    hand_name: str | None = None
    hand_id: int | None = None
    status: str | None = None
    active_imus: int | None = None
    total_imus: int | None = None
    source_line: str | None = None
    error: str | None = None

    @property
    def is_glove(self) -> bool:
        return self.kind == "glove" and self.hand_name in {"left", "right"}

    @property
    def label(self) -> str:
        if self.is_glove:
            return f"{self.hand_name}({self.hand_id})"
        if self.error:
            return f"{self.kind}: {self.error}"
        return self.kind


def normalize_side(side: str | None) -> str | None:
    if side is None:
        return None
    side = side.lower()
    if side in {"l", "left", "0"}:
        return "left"
    if side in {"r", "right", "1"}:
        return "right"
    raise ValueError(f"unknown side: {side}")


def parse_identity_line(line: str, info: DeviceInfo) -> None:
    text = line.strip()
    if not text:
        return

    # Preferred machine-readable identity:
    # $GLOVITY,DEVICE,GLOVE,left,0
    parts = [p.strip() for p in text.split(",")]
    if len(parts) >= 5 and parts[:3] == ["$GLOVITY", "DEVICE", "GLOVE"]:
        side = normalize_side(parts[3])
        try:
            hand_id = int(parts[4])
        except ValueError:
            hand_id = 0 if side == "left" else 1
        info.kind = "glove"
        info.hand_name = side
        info.hand_id = hand_id
        info.source_line = text
        return

    # Extended status:
    # $GLOVITY,CAL,STATUS,IDLE,11,11,left,0
    if len(parts) >= 8 and parts[:3] == ["$GLOVITY", "CAL", "STATUS"]:
        info.kind = "glove"
        info.status = parts[3]
        try:
            info.active_imus = int(parts[4])
            info.total_imus = int(parts[5])
        except ValueError:
            pass
        side = normalize_side(parts[6])
        try:
            hand_id = int(parts[7])
        except ValueError:
            hand_id = 0 if side == "left" else 1
        info.hand_name = side
        info.hand_id = hand_id
        info.source_line = text
        return

    # Older status without hand identity still tells us it is Glovity firmware,
    # but not which hand. Keep scanning for banner/text-frame hand data.
    if len(parts) >= 6 and parts[:3] == ["$GLOVITY", "CAL", "STATUS"]:
        info.kind = "glove"
        info.status = parts[3]
        try:
            info.active_imus = int(parts[4])
            info.total_imus = int(parts[5])
        except ValueError:
            pass
        info.source_line = text
        return

    # Human-readable banner/text frames:
    # Mode: ... Hand: left(0) ...
    # Frame: ... Hand: right(1) ...
    match = re.search(r"\bHand:\s*(left|right)\s*\((\d+)\)", text, re.I)
    if match:
        info.kind = "glove"
        info.hand_name = match.group(1).lower()
        info.hand_id = int(match.group(2))
        info.source_line = text


def open_serial(port: str, baud: int, timeout: float = 0.1) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=1,
        rtscts=False,
        dsrdtr=False,
    )
    # Avoid reset-prone control-line combinations on ESP32-S3 boards.
    ser.dtr = False
    ser.rts = False
    return ser


def read_text_lines(ser: serial.Serial, seconds: float) -> list[str]:
    deadline = time.monotonic() + seconds
    lines: list[str] = []
    pending = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        for b in chunk:
            if b in (10, 13):
                if pending:
                    lines.append(pending.decode("utf-8", errors="replace").strip())
                    pending.clear()
            elif 32 <= b <= 126 or b >= 128:
                pending.append(b)
            else:
                # Receiver binary frames often land here. Drop non-text bytes.
                if len(pending) > 240:
                    pending.clear()
    if pending:
        lines.append(pending.decode("utf-8", errors="replace").strip())
    return lines


def identify_port(port: str, baud: int, probe_seconds: float) -> DeviceInfo:
    info = DeviceInfo(port=port)
    try:
        with open_serial(port, baud) as ser:
            time.sleep(0.25)
            ser.reset_input_buffer()
            for _ in range(3):
                ser.write(b"$GLOVITY,CAL,STATUS\r\n")
                ser.flush()
                time.sleep(COMMAND_DELAY_S)
            for line in read_text_lines(ser, probe_seconds):
                parse_identity_line(line, info)
                if info.is_glove:
                    break
    except Exception as exc:  # noqa: BLE001 - show port-level failures to operator.
        info.kind = "unavailable"
        info.error = str(exc)
    return info


def list_candidate_ports(
    explicit_ports: Iterable[str] | None = None,
    include_bluetooth: bool = False,
) -> list[str]:
    if explicit_ports:
        return list(dict.fromkeys(explicit_ports))
    ports = []
    for item in list_ports.comports():
        text = f"{item.description} {item.hwid}".lower()
        if not include_bluetooth and ("bthenum" in text or "bluetooth" in text):
            continue
        ports.append(item.device)
    return sorted(dict.fromkeys(ports))


def scan_gloves(ports: list[str], baud: int, probe_seconds: float) -> list[DeviceInfo]:
    results = []
    for port in ports:
        info = identify_port(port, baud, probe_seconds)
        results.append(info)
        print(f"[scan] {port}: {info.label}")
    return results


def choose_gloves(
    scanned: list[DeviceInfo],
    side: str | None,
    all_gloves: bool,
) -> list[DeviceInfo]:
    gloves = [item for item in scanned if item.is_glove]
    if all_gloves:
        by_side: dict[str, DeviceInfo] = {}
        for item in gloves:
            by_side.setdefault(item.hand_name or "", item)
        ordered = [by_side[s] for s in ("left", "right") if s in by_side]
        if not ordered:
            raise RuntimeError("no Glovity glove ports were identified")
        return ordered

    if side:
        matches = [item for item in gloves if item.hand_name == side]
        if not matches:
            raise RuntimeError(f"no {side} glove port was identified")
        if len(matches) > 1:
            raise RuntimeError(f"multiple {side} glove ports identified: {[m.port for m in matches]}")
        return matches

    if len(gloves) != 1:
        raise RuntimeError(
            "please specify --side left/right, --all, or --port when multiple/no gloves are identified"
        )
    return gloves


def wait_for_token(
    ser: serial.Serial,
    tokens: tuple[str, ...],
    timeout_s: float,
    echo: bool = True,
) -> tuple[bool, list[str]]:
    deadline = time.monotonic() + timeout_s
    lines_seen: list[str] = []
    pending = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        for b in chunk:
            if b in (10, 13):
                if not pending:
                    continue
                line = pending.decode("utf-8", errors="replace").strip()
                pending.clear()
                if not line:
                    continue
                lines_seen.append(line)
                if echo:
                    print(line)
                if any(token in line for token in tokens):
                    return True, lines_seen
            elif 32 <= b <= 126 or b >= 128:
                pending.append(b)
            elif len(pending) > 240:
                pending.clear()
    return False, lines_seen


def calibrate_one(info: DeviceInfo, baud: int, duration_s: float, yes: bool) -> None:
    label = info.label
    print(f"\n=== Calibrating {label} on {info.port} ===")
    if not yes:
        input("Press Enter to start magnetometer calibration...")

    with open_serial(info.port, baud, timeout=0.1) as ser:
        time.sleep(0.25)
        ser.reset_input_buffer()

        ser.write(b"$GLOVITY,CAL,STATUS\r\n")
        ser.flush()
        ok, _ = wait_for_token(ser, ("$GLOVITY,CAL,STATUS",), 2.0, echo=True)
        if not ok:
            raise RuntimeError(f"{info.port} did not respond to CAL STATUS")

        ser.write(b"$GLOVITY,CAL,START\r\n")
        ser.flush()
        ok, lines = wait_for_token(
            ser,
            ("$GLOVITY,CAL,STARTED", "$GLOVITY,CAL,ERROR"),
            8.0,
            echo=True,
        )
        if not ok or any("$GLOVITY,CAL,ERROR" in line for line in lines):
            raise RuntimeError(f"{info.port} failed to enter calibration mode")

        print(f"Rotate {label} slowly in all directions for {duration_s:.0f}s...")
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            wait_for_token(ser, ("__never_match__",), min(1.0, deadline - time.monotonic()), echo=True)

        ser.write(b"$GLOVITY,CAL,STOP\r\n")
        ser.flush()
        ok, lines = wait_for_token(
            ser,
            ("$GLOVITY,CAL,COMPLETE", "$GLOVITY,CAL,ERROR"),
            12.0,
            echo=True,
        )
        if not ok or any("$GLOVITY,CAL,ERROR" in line for line in lines):
            raise RuntimeError(f"{info.port} did not complete calibration cleanly")

        # Give firmware a moment to print MODE,NORMAL after COMPLETE.
        wait_for_token(ser, ("$GLOVITY,MODE,NORMAL",), 2.0, echo=True)
    print(f"=== {label} calibration done ===")


def main() -> int:
    parser = argparse.ArgumentParser(description="Glovity USB-triggered magnetometer calibration")
    parser.add_argument("--port", action="append", help="Serial port to use. Can be repeated.")
    parser.add_argument("--auto", action="store_true", help="Scan serial ports and auto-detect glove side.")
    parser.add_argument("--side", choices=["left", "right"], help="Calibrate the detected left or right glove.")
    parser.add_argument("--all", action="store_true", help="Calibrate all detected gloves, left then right.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Glove USB CDC baud, default {DEFAULT_BAUD}.")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="Rotation duration in seconds.")
    parser.add_argument("--probe-seconds", type=float, default=2.0, help="Per-port identity probe time.")
    parser.add_argument("--scan-only", action="store_true", help="Only print detected devices; do not calibrate.")
    parser.add_argument("--include-bluetooth", action="store_true", help="Also probe Bluetooth virtual COM ports.")
    parser.add_argument("-y", "--yes", action="store_true", help="Do not wait for Enter before each calibration.")
    args = parser.parse_args()

    if args.all and args.side:
        parser.error("--all and --side cannot be used together")
    if not args.port and not args.auto:
        args.auto = True

    try:
        if args.auto:
            ports = list_candidate_ports(args.port, args.include_bluetooth)
            if not ports:
                raise RuntimeError("no serial ports found")
            scanned = scan_gloves(ports, args.baud, args.probe_seconds)
            if args.scan_only:
                return 0
            targets = choose_gloves(scanned, args.side, args.all)
        else:
            scanned = scan_gloves(list_candidate_ports(args.port, args.include_bluetooth), args.baud, args.probe_seconds)
            if args.scan_only:
                return 0
            targets = [item if item.is_glove else DeviceInfo(port=item.port, kind="glove") for item in scanned]

        for target in targets:
            calibrate_one(target, args.baud, args.duration, args.yes)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:  # noqa: BLE001 - command-line tool should print concise error.
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
