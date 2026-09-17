#!/usr/bin/env python3
"""Validate a Jensen Hand bench/performance campaign and render a report.

Empty or undersampled sections are PENDING, failed gates are FAIL, and only a
fully populated passing campaign exits zero. This intentionally cannot invent
measurements; it turns physical evidence into a reproducible release decision.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Gate:
    name: str
    status: str
    value: str
    criterion: str
    detail: str = ""


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"expected finite number, got {value!r}")
    return float(value)


def threshold(data: dict[str, Any], key: str) -> float:
    return number(data["thresholds"][key])


def pending(name: str, criterion: str, detail: str) -> Gate:
    return Gate(name, "PENDING", "no decision", criterion, detail)


def evaluate(data: dict[str, Any]) -> list[Gate]:
    gates: list[Gate] = []
    meta = data.get("metadata", {})
    required_meta = ("device_id", "firmware_commit", "operator", "date_utc")
    complete = all(str(meta.get(key, "")).strip() for key in required_meta) and bool(meta.get("instruments"))
    gates.append(Gate("Traceability", "PASS" if complete else "PENDING",
                      "complete" if complete else "incomplete", "device, commit, operator, UTC date, instruments"))

    battery = data.get("battery", [])
    if len(battery) < 3:
        gates.append(pending("Battery divider", "at least 3 points and max error within threshold",
                             f"received {len(battery)} point(s)"))
    elif any(number(row["reference_volts"]) == 0 for row in battery):
        bad = sum(1 for row in battery if number(row["reference_volts"]) == 0)
        gates.append(Gate("Battery divider", "FAIL", f"{bad} point(s) with 0 V reference",
                          "reference_volts must be non-zero (relative error is undefined at 0 V)"))
    else:
        errors = [abs(number(row["telemetry_volts"]) - number(row["reference_volts"])) /
                  number(row["reference_volts"]) * 100.0 for row in battery]
        limit = threshold(data, "battery_max_error_percent")
        gates.append(Gate("Battery divider", "PASS" if max(errors) <= limit else "FAIL",
                          f"max error {max(errors):.2f}%", f"<= {limit:.2f}%"))

    required_digits = {"thumb", "index", "middle", "ring", "pinky"}
    endpoints = {str(row.get("digit")): row for row in data.get("servo_endpoints", [])}
    missing = sorted(required_digits - endpoints.keys())
    endpoint_ok = not missing and all(bool(endpoints[d].get("clearance_pass")) and
                                      abs(number(endpoints[d]["closed_us"]) - number(endpoints[d]["open_us"])) >= 200
                                      for d in required_digits)
    status = "PENDING" if missing else ("PASS" if endpoint_ok else "FAIL")
    gates.append(Gate("Servo endpoints", status, f"{len(required_digits) - len(missing)}/5 required digits",
                      "clearance pass and >=200 us usable travel", f"missing: {', '.join(missing)}" if missing else ""))

    fsr_by_sensor: dict[str, list[float]] = defaultdict(list)
    for row in data.get("fsr", []):
        ref = number(row["reference_newtons"])
        if ref > 0:
            fsr_by_sensor[str(row["sensor"])].append(abs(number(row["telemetry_newtons"]) - ref) / ref * 100.0)
    fsr_required = {"thumb", "index", "middle"}
    fsr_missing = sorted(sensor for sensor in fsr_required if len(fsr_by_sensor[sensor]) < 3)
    if fsr_missing:
        gates.append(pending("FSR force fit", "3+ loads per fitted sensor within mean-error threshold",
                             f"undersampled: {', '.join(fsr_missing)}"))
    else:
        mean_error = statistics.mean(error for sensor in fsr_required for error in fsr_by_sensor[sensor])
        limit = threshold(data, "fsr_mean_error_percent")
        gates.append(Gate("FSR force fit", "PASS" if mean_error <= limit else "FAIL",
                          f"mean absolute error {mean_error:.2f}%", f"<= {limit:.2f}%"))

    stalls = data.get("stall_trials", [])
    if not stalls:
        gates.append(pending("Direct stall detection", "latency/isolation pass for every fitted current channel",
                             "no stall trials"))
    else:
        limit = threshold(data, "stall_max_latency_ms")
        ok = all(number(row["detection_latency_ms"]) <= limit and bool(row.get("isolated_channel")) and
                 bool(row.get("other_channels_operational")) for row in stalls)
        worst = max(number(row["detection_latency_ms"]) for row in stalls)
        gates.append(Gate("Direct stall detection", "PASS" if ok else "FAIL", f"worst latency {worst:.0f} ms",
                          f"<= {limit:.0f} ms; stalled channel isolated; other channels operate"))

    pattern_rows = data.get("pattern_trials", [])
    pattern_by_class: dict[str, list[bool]] = defaultdict(list)
    for row in pattern_rows:
        pattern_by_class[str(row["expected_grip"])].append(row.get("predicted_grip") == row.get("expected_grip"))
    if len(pattern_by_class) < 3 or any(len(rows) < 10 for rows in pattern_by_class.values()):
        gates.append(pending("Pattern recognition", "3+ grips, 10+ blinded trials/grip, accuracy gates",
                             f"classes/trials: {', '.join(f'{k}:{len(v)}' for k, v in pattern_by_class.items()) or 'none'}"))
    else:
        all_trials = [result for rows in pattern_by_class.values() for result in rows]
        overall = sum(all_trials) / len(all_trials)
        worst_class = min(sum(rows) / len(rows) for rows in pattern_by_class.values())
        overall_min = threshold(data, "pattern_min_overall_accuracy")
        class_min = threshold(data, "pattern_min_class_accuracy")
        ok = overall >= overall_min and worst_class >= class_min
        gates.append(Gate("Pattern recognition", "PASS" if ok else "FAIL",
                          f"overall {overall:.1%}; worst class {worst_class:.1%}",
                          f">= {overall_min:.0%} overall and >= {class_min:.0%} each class"))

    forces = data.get("grasp_force", [])
    if not forces:
        gates.append(pending("Grasp force", "power grasp reaches practical minimum; pinch stays below maximum",
                             "no load-cell trials"))
    else:
        power = [number(row["measured_newtons"]) for row in forces if row.get("grip") == "power"]
        pinch = [number(row["measured_newtons"]) for row in forces if row.get("grip") == "pinch"]
        if len(power) < 5 or len(pinch) < 5:
            gates.append(pending("Grasp force", "5+ power and pinch trials", f"power={len(power)}, pinch={len(pinch)}"))
        else:
            practical = threshold(data, "minimum_practical_grip_newtons")
            pinch_max = threshold(data, "maximum_pinch_newtons")
            ok = statistics.mean(power) >= practical and max(pinch) <= pinch_max
            gates.append(Gate("Grasp force", "PASS" if ok else "FAIL",
                              f"power mean {statistics.mean(power):.1f} N; pinch max {max(pinch):.1f} N",
                              f"power >= {practical:.0f} N; pinch <= {pinch_max:.0f} N"))

    life = data.get("cycle_life", [])
    if not life:
        gates.append(pending("Cycle life", "survive benchmark without functional loss", "no cycle campaign"))
    else:
        best = max(life, key=lambda row: number(row["cycles"]))
        cycles = int(number(best["cycles"]))
        minimum = int(threshold(data, "minimum_grasp_cycles"))
        service = int(threshold(data, "service_goal_grasp_cycles"))
        intact = not bool(best.get("failure")) and bool(best.get("inspection_pass"))
        status = "PASS" if cycles >= minimum and intact else "FAIL"
        gates.append(Gate("Cycle life", status, f"{cycles:,} cycles; {'intact' if intact else 'functional loss'}",
                          f">= {minimum:,} baseline ({service:,} service goal)"))
    return gates


def render(data: dict[str, Any], gates: list[Gate]) -> str:
    overall = "PASS" if all(g.status == "PASS" for g in gates) else (
        "FAIL" if any(g.status == "FAIL" for g in gates) else "PENDING")
    meta = data.get("metadata", {})
    lines = ["# Jensen Hand verification report", "", f"Overall: **{overall}**", "",
             f"Device: `{meta.get('device_id', '')}`  ", f"Firmware: `{meta.get('firmware_commit', '')}`  ",
             f"Test date: `{meta.get('date_utc', '')}`", "", "| Gate | Status | Result | Criterion |", "|---|---|---|---|"]
    for gate in gates:
        result = gate.value + (f" ({gate.detail})" if gate.detail else "")
        lines.append(f"| {gate.name} | **{gate.status}** | {result} | {gate.criterion} |")
    lines += ["", "A PENDING report is not bench validation. Keep raw measurements, instrument IDs, and this report together.", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--report", type=Path, help="write Markdown report")
    args = parser.parse_args()
    try:
        data = json.loads(args.campaign.read_text(encoding="utf-8"))
        gates = evaluate(data)
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        print(f"invalid campaign: {exc}", file=sys.stderr)
        return 3
    report = render(data, gates)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report, encoding="utf-8")
    else:
        print(report)
    if any(g.status == "FAIL" for g in gates):
        return 1
    if any(g.status == "PENDING" for g in gates):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
