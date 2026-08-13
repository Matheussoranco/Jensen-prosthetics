#!/usr/bin/env python3
"""Live telemetry viewer and logger for the Jensen Hand.

The firmware emits one line per sample when telemetry is enabled:

    T,t=12345,mode=active,grip=power,ap=0.4210,vel=+1.2000,ef=0.55,...

This tool enables that stream, renders it as a terminal dashboard, and can log
it to CSV for later analysis. It is the practical way to tune the drive law and
the grip-force loop: watching `ef`/`ee` while contracting tells you whether the
electrodes are placed well, and watching `pk` against `tgt` while gripping
tells you whether the force loop is regulating or fighting.

Usage
-----
    python jensen_monitor.py COM7
    python jensen_monitor.py /dev/ttyUSB0 --hz 20 --csv session.csv
    python jensen_monitor.py COM7 --raw          # print lines, no dashboard

Requires pyserial:  pip install pyserial
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from typing import Any

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - dependency guidance
    sys.exit("pyserial is required:  pip install pyserial")


DIGITS = ["thumb", "thumb_rot", "index", "middle", "ring", "pinky", "wrist"]
GRASP_DIGITS = DIGITS[:6]

FAULT_BITS = [
    (0x0001, "battery_low"),
    (0x0002, "battery_critical"),
    (0x0004, "servo_bus"),
    (0x0008, "emg_uncalibrated"),
    (0x0010, "digits_uncalibrated"),
    (0x0020, "over_force"),
    (0x0040, "sustained_load"),
    (0x0080, "loop_overrun"),
    (0x0100, "sensor_range"),
    (0x0200, "motor_stall"),
]


def parse_line(line: str) -> dict[str, Any] | None:
    """Parses one `T,key=value,...` telemetry record."""
    if not line.startswith("T,"):
        return None
    record: dict[str, Any] = {}
    for field in line[2:].split(","):
        key, separator, value = field.partition("=")
        if not separator:
            continue
        key = key.strip()
        value = value.strip()
        if key in ("mode", "grip"):
            record[key] = value
        elif key == "flt":
            try:
                record[key] = int(value, 16)
            except ValueError:
                record[key] = 0
        else:
            try:
                record[key] = float(value)
            except ValueError:
                record[key] = value
    return record or None


def bar(value: float, width: int = 24, lo: float = 0.0, hi: float = 1.0) -> str:
    span = hi - lo
    fraction = 0.0 if span <= 0 else (value - lo) / span
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * width))
    return "#" * filled + "." * (width - filled)


def decode_faults(mask: int) -> str:
    if not mask:
        return "none"
    names = [name for bit, name in FAULT_BITS if mask & bit]
    return " ".join(names) if names else f"0x{mask:04X}"


def render(record: dict[str, Any]) -> str:
    get = record.get
    lines = [
        "Jensen Hand — live telemetry    (ctrl-c to stop)",
        "",
        f"  mode     {str(get('mode', '?')):<16} grip  {str(get('grip', '?')):<10}"
        f" loop {float(get('loop', 0.0)) * 1000:5.2f} ms",
        "",
        f"  aperture [{bar(float(get('ap', 0.0)))}] {float(get('ap', 0.0)):5.3f}"
        f"   velocity {float(get('vel', 0.0)):+6.2f}/s",
        f"  limit    [{bar(float(get('lim', 1.0)))}] {float(get('lim', 1.0)):5.3f}",
        "",
        f"  emg flex [{bar(float(get('ef', 0.0)))}] {float(get('ef', 0.0)):5.3f}",
        f"  emg ext  [{bar(float(get('ee', 0.0)))}] {float(get('ee', 0.0)):5.3f}",
        "  sites    " + "  ".join(f"{i}:{float(get(f's{i}', 0.0)):4.2f}" for i in range(4)),
        "",
    ]

    target = float(get("tgt", 0.0)) or 1.0
    lines.append("  fingertip force (N)")
    for i, name in enumerate(GRASP_DIGITS):
        force = float(get(f"f{i}", 0.0))
        if force == 0.0 and i in (1, 4, 5):
            continue  # no sensor fitted on these by default
        lines.append(f"    {name:<10} [{bar(force, 20, 0.0, max(target * 1.5, 1.0))}] {force:5.1f}")
    lines.append(
        f"    peak {float(get('pk', 0.0)):5.1f} / target {float(get('tgt', 0.0)):5.1f}"
        f"   contact {'yes' if float(get('ct', 0.0)) else 'no ':<3}"
        f"   slips {int(float(get('slip', 0.0)))}"
    )
    lines.append("")

    lines.append("  servo pulses (us)")
    pulses = "    " + "  ".join(
        f"{name}:{int(float(get(f'p{i}', 0.0))):4d}" for i, name in enumerate(DIGITS)
    )
    lines.append(pulses)
    currents = "    " + "  ".join(
        f"{name}:{float(get(f'i{i}', 0.0)):4.2f}A" for i, name in enumerate(DIGITS)
        if float(get(f'i{i}', 0.0)) > 0.0 or (int(float(get('stall', 0.0))) & (1 << i))
    )
    if currents.strip():
        lines.append("  motor current")
        lines.append(currents)
    lines.append("")

    charge = float(get("soc", 0.0))
    lines.append(
        f"  battery  [{bar(charge, 20)}] {charge * 100:3.0f}%  {float(get('vb', 0.0)):5.2f} V"
    )
    lines.append(f"  faults   {decode_faults(int(get('flt', 0)))}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=10.0, help="telemetry rate to request")
    parser.add_argument("--csv", help="also log every record to this CSV file")
    parser.add_argument("--raw", action="store_true", help="print raw lines instead of a dashboard")
    args = parser.parse_args()

    try:
        link = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as error:
        return f"could not open {args.port}: {error}"  # type: ignore[return-value]

    # Give the ESP32 time to finish resetting if opening the port asserted DTR.
    time.sleep(0.4)
    link.reset_input_buffer()
    link.write(f"telem on {args.hz:g}\n".encode("ascii"))
    link.flush()

    writer = None
    csv_file = None
    columns: list[str] = []

    print("\x1b[2J", end="")  # clear screen once
    try:
        while True:
            raw = link.readline().decode("ascii", errors="replace").strip()
            if not raw:
                continue

            if args.raw:
                print(raw)
                continue

            record = parse_line(raw)
            if record is None:
                continue

            # Home the cursor rather than clearing, so the dashboard does not
            # flicker at 10 Hz.
            print("\x1b[H" + render(record) + "\x1b[J")

            if args.csv:
                if writer is None:
                    csv_file = open(args.csv, "w", newline="", encoding="utf-8")
                    columns = list(record.keys())
                    writer = csv.DictWriter(csv_file, fieldnames=columns, extrasaction="ignore")
                    writer.writeheader()
                writer.writerow(record)

    except KeyboardInterrupt:
        print()
    finally:
        try:
            link.write(b"telem off\n")
            link.flush()
        except serial.SerialException:
            pass
        link.close()
        if csv_file is not None:
            csv_file.close()
            print(f"logged to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
