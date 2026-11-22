#!/usr/bin/env python3
"""Validate a Jensen Hand bench/performance campaign and render a report.

Empty or undersampled sections are PENDING, failed gates are FAIL, and only a
fully populated passing campaign on an identified physical assembly exits zero.
This intentionally cannot invent measurements; it rejects synthetic/example
records and unfitted-channel claims, and turns physical evidence into a
reproducible release decision.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

BASE_DIGITS: tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")
DEFAULT_FSR_SENSORS: tuple[str, ...] = ("thumb", "index", "middle")

#: Known hardware profiles reflecting ``hardware/profile-trace.md``.
#: Design-stage profiles can be evaluated for diagnostic completeness, but
#: cannot pass release eligibility until an identified physical assembly is recorded.
KNOWN_PROFILES: dict[str, dict[str, Any]] = {
    "unidentified_design_stage": {
        "identified_assembly": False,
        "has_thumb_opposition": False,
        "has_wrist": False,
        "fitted_fsr_channels": list(DEFAULT_FSR_SENSORS),
        "fitted_current_channels": [],
    },
    "cad_design_stage": {
        "identified_assembly": False,
        "has_thumb_opposition": False,
        "has_wrist": True,
        "fitted_fsr_channels": list(DEFAULT_FSR_SENSORS),
        "fitted_current_channels": [],
    },
    "firmware_default": {
        "identified_assembly": False,
        "has_thumb_opposition": True,
        "has_wrist": False,
        "fitted_fsr_channels": list(DEFAULT_FSR_SENSORS),
        "fitted_current_channels": [],
    },
    "identified_assembly": {
        "identified_assembly": True,
        "has_thumb_opposition": False,
        "has_wrist": False,
        "fitted_fsr_channels": list(DEFAULT_FSR_SENSORS),
        "fitted_current_channels": [],
    },
}

_PLACEHOLDER_DEVICE_RE = re.compile(
    r"(?:^example\b|not-a-real-test|synthetic|dummy|\btbd\b|\bplaceholder\b)",
    re.IGNORECASE,
)
_ALL_ZERO_COMMIT_RE = re.compile(r"^0+$")
_PLACEHOLDER_OPERATOR_RE = re.compile(r"^(?:example|placeholder|tbd|synthetic|none)$", re.IGNORECASE)


@dataclass
class Gate:
    name: str
    status: str
    value: str
    criterion: str
    detail: str = ""


@dataclass(frozen=True)
class ResolvedProfile:
    name: str
    identified_assembly: bool
    assembly_id: str
    actuators: tuple[str, ...]
    has_thumb_opposition: bool
    has_wrist: bool
    fitted_fsr_channels: tuple[str, ...]
    fitted_current_channels: tuple[str, ...]
    emg_front_end_isolated: bool | None


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"expected finite number, got {value!r}")
    return float(value)


def threshold(data: dict[str, Any], key: str, default: float | None = None) -> float:
    thresholds = data.get("thresholds", {})
    if key not in thresholds:
        if default is not None:
            return float(default)
        raise KeyError(key)
    return number(thresholds[key])


def pending(name: str, criterion: str, detail: str) -> Gate:
    return Gate(name, "PENDING", "no decision", criterion, detail)


def _string_tuple(raw: Any, default: tuple[str, ...]) -> tuple[str, ...]:
    if raw is None:
        return default
    if not isinstance(raw, list):
        raise ValueError(f"expected list of channel names, got {raw!r}")
    return tuple(str(item).strip() for item in raw if str(item).strip())


def resolve_profile(data: dict[str, Any]) -> ResolvedProfile:
    """Resolve the declared hardware profile or default to unidentified design stage."""
    raw = data.get("hardware_profile")
    if raw is None:
        raw = {}
    elif isinstance(raw, str):
        raw = {"name": raw}
    elif not isinstance(raw, dict):
        raise ValueError(f"hardware_profile must be a string or object, got {raw!r}")

    name = str(raw.get("name") or "unidentified_design_stage").strip()
    if name not in KNOWN_PROFILES:
        raise ValueError(
            f"unknown hardware_profile {name!r}; expected one of {sorted(KNOWN_PROFILES)}"
        )
    spec = KNOWN_PROFILES[name]

    has_thumb_opp = bool(raw.get("has_thumb_opposition", spec["has_thumb_opposition"]))
    has_wrist = bool(raw.get("has_wrist", spec["has_wrist"]))
    fitted_fsr = _string_tuple(raw.get("fitted_fsr_channels"), tuple(spec["fitted_fsr_channels"]))
    fitted_curr = _string_tuple(
        raw.get("fitted_current_channels"), tuple(spec["fitted_current_channels"])
    )
    assembly_id = str(raw.get("assembly_id") or "").strip()
    actuators = _string_tuple(raw.get("actuators"), ())
    emg_iso_raw = raw.get("emg_front_end_isolated")
    emg_isolated = None if emg_iso_raw is None else bool(emg_iso_raw)

    return ResolvedProfile(
        name=name,
        identified_assembly=bool(spec["identified_assembly"]),
        assembly_id=assembly_id,
        actuators=actuators,
        has_thumb_opposition=has_thumb_opp,
        has_wrist=has_wrist,
        fitted_fsr_channels=fitted_fsr,
        fitted_current_channels=fitted_curr,
        emg_front_end_isolated=emg_isolated,
    )


def _check_provenance(meta: dict[str, Any]) -> Gate:
    reasons: list[str] = []
    if bool(meta.get("synthetic", False)):
        reasons.append("metadata.synthetic=true")
    device_id = str(meta.get("device_id", "")).strip()
    commit = str(meta.get("firmware_commit", "")).strip()
    operator = str(meta.get("operator", "")).strip()

    if device_id and _PLACEHOLDER_DEVICE_RE.search(device_id):
        reasons.append(f"placeholder device_id {device_id!r}")
    if commit and _ALL_ZERO_COMMIT_RE.match(commit):
        reasons.append(f"placeholder firmware_commit {commit!r}")
    if operator and _PLACEHOLDER_OPERATOR_RE.match(operator):
        reasons.append(f"placeholder operator {operator!r}")

    criterion = "non-synthetic, non-example physical bench record"
    if reasons:
        return Gate("Evidence provenance", "FAIL", "synthetic/example rejected", criterion, "; ".join(reasons))
    if not (device_id and commit and operator):
        return pending("Evidence provenance", criterion, "incomplete provenance metadata")
    return Gate("Evidence provenance", "PASS", "physical record declared", criterion)


def _check_profile_gate(profile: ResolvedProfile) -> Gate:
    criterion = "identified physical assembly with declared actuators and isolated EMG front end"
    if not profile.identified_assembly:
        return pending(
            "Hardware profile",
            criterion,
            f"design-stage profile '{profile.name}' cannot release without an identified assembly",
        )
    if profile.emg_front_end_isolated is False:
        return Gate(
            "Hardware profile",
            "FAIL",
            f"{profile.name} ({profile.assembly_id or 'unnamed'})",
            criterion,
            "EMG front-end isolation explicitly false",
        )
    missing: list[str] = []
    if not profile.assembly_id:
        missing.append("assembly_id")
    if not profile.actuators:
        missing.append("actuators")
    if profile.emg_front_end_isolated is None:
        missing.append("emg_front_end_isolated")
    if missing:
        return pending(
            "Hardware profile",
            criterion,
            f"missing profile fields: {', '.join(missing)}",
        )
    return Gate(
        "Hardware profile",
        "PASS",
        f"{profile.assembly_id} ({len(profile.actuators)} actuator spec(s))",
        criterion,
    )


def evaluate(data: dict[str, Any]) -> list[Gate]:
    gates: list[Gate] = []
    meta = data.get("metadata", {})
    if not isinstance(meta, dict):
        raise ValueError("metadata must be an object")

    profile = resolve_profile(data)
    gates.append(_check_provenance(meta))
    gates.append(_check_profile_gate(profile))

    required_meta = ("device_id", "firmware_commit", "operator", "date_utc")
    complete = all(str(meta.get(key, "")).strip() for key in required_meta) and bool(
        meta.get("instruments")
    )
    gates.append(
        Gate(
            "Traceability",
            "PASS" if complete else "PENDING",
            "complete" if complete else "incomplete",
            "device, commit, operator, UTC date, instruments",
        )
    )

    battery = data.get("battery", [])
    if len(battery) < 3:
        gates.append(
            pending(
                "Battery divider",
                "at least 3 points and max error within threshold",
                f"received {len(battery)} point(s)",
            )
        )
    elif any(number(row["reference_volts"]) == 0 for row in battery):
        bad = sum(1 for row in battery if number(row["reference_volts"]) == 0)
        gates.append(
            Gate(
                "Battery divider",
                "FAIL",
                f"{bad} point(s) with 0 V reference",
                "reference_volts must be non-zero (relative error is undefined at 0 V)",
            )
        )
    else:
        errors = [
            abs(number(row["telemetry_volts"]) - number(row["reference_volts"]))
            / number(row["reference_volts"])
            * 100.0
            for row in battery
        ]
        limit = threshold(data, "battery_max_error_percent")
        gates.append(
            Gate(
                "Battery divider",
                "PASS" if max(errors) <= limit else "FAIL",
                f"max error {max(errors):.2f}%",
                f"<= {limit:.2f}%",
            )
        )

    required_digits = set(BASE_DIGITS)
    if profile.has_thumb_opposition:
        required_digits.add("thumb_rot")
    if profile.has_wrist:
        required_digits.add("wrist")

    endpoints = {str(row.get("digit")): row for row in data.get("servo_endpoints", [])}
    missing = sorted(required_digits - endpoints.keys())
    endpoint_ok = not missing and all(
        bool(endpoints[d].get("clearance_pass"))
        and abs(number(endpoints[d]["closed_us"]) - number(endpoints[d]["open_us"])) >= 200
        for d in required_digits
    )
    status = "PENDING" if missing else ("PASS" if endpoint_ok else "FAIL")
    gates.append(
        Gate(
            "Servo endpoints",
            status,
            f"{len(required_digits) - len(missing)}/{len(required_digits)} required channels",
            "clearance pass and >=200 us usable travel for all profile channels",
            f"missing: {', '.join(missing)}" if missing else "",
        )
    )

    fsr_by_sensor: dict[str, list[float]] = defaultdict(list)
    for row in data.get("fsr", []):
        ref = number(row["reference_newtons"])
        if ref > 0:
            fsr_by_sensor[str(row["sensor"])].append(
                abs(number(row["telemetry_newtons"]) - ref) / ref * 100.0
            )
    fsr_required = set(profile.fitted_fsr_channels)
    unfitted_fsr = sorted(set(fsr_by_sensor.keys()) - fsr_required)
    if unfitted_fsr:
        gates.append(
            Gate(
                "FSR force fit",
                "FAIL",
                f"unfitted sensor(s): {', '.join(unfitted_fsr)}",
                "measurements must match fitted FSR channels in hardware profile",
            )
        )
    elif not fsr_required:
        gates.append(
            pending(
                "FSR force fit",
                "3+ loads per fitted sensor within mean-error threshold",
                "no fitted FSR channels declared in profile",
            )
        )
    else:
        fsr_missing = sorted(sensor for sensor in fsr_required if len(fsr_by_sensor[sensor]) < 3)
        if fsr_missing:
            gates.append(
                pending(
                    "FSR force fit",
                    "3+ loads per fitted sensor within mean-error threshold",
                    f"undersampled: {', '.join(fsr_missing)}",
                )
            )
        else:
            mean_error = statistics.mean(
                error for sensor in fsr_required for error in fsr_by_sensor[sensor]
            )
            limit = threshold(data, "fsr_mean_error_percent")
            gates.append(
                Gate(
                    "FSR force fit",
                    "PASS" if mean_error <= limit else "FAIL",
                    f"mean absolute error {mean_error:.2f}%",
                    f"<= {limit:.2f}%",
                )
            )

    stalls = data.get("stall_trials", [])
    fitted_current = set(profile.fitted_current_channels)
    if not fitted_current:
        if stalls:
            gates.append(
                Gate(
                    "Direct stall detection",
                    "FAIL",
                    f"{len(stalls)} trial(s) reported without fitted current channels",
                    "an unavailable current channel cannot substantiate a stall measurement",
                    f"profile '{profile.name}' has 0 fitted current channels",
                )
            )
        else:
            gates.append(
                pending(
                    "Direct stall detection",
                    "latency/isolation pass for every fitted current channel",
                    f"no fitted current channels in profile '{profile.name}'",
                )
            )
    elif not stalls:
        gates.append(
            pending(
                "Direct stall detection",
                "latency/isolation pass for every fitted current channel",
                f"no stall trials for fitted channels: {', '.join(sorted(fitted_current))}",
            )
        )
    else:
        trial_channels = {str(row.get("digit", "")).strip() for row in stalls}
        unfitted_stall = sorted(trial_channels - fitted_current)
        missing_stall = sorted(fitted_current - trial_channels)
        limit = threshold(data, "stall_max_latency_ms")
        worst = max(number(row["detection_latency_ms"]) for row in stalls)
        trials_ok = all(
            number(row["detection_latency_ms"]) <= limit
            and bool(row.get("isolated_channel"))
            and bool(row.get("other_channels_operational"))
            for row in stalls
        )
        if unfitted_stall:
            gates.append(
                Gate(
                    "Direct stall detection",
                    "FAIL",
                    f"unfitted channel(s): {', '.join(unfitted_stall)}",
                    "stall trials must correspond to fitted current channels",
                )
            )
        elif missing_stall:
            gates.append(
                pending(
                    "Direct stall detection",
                    "latency/isolation pass for every fitted current channel",
                    f"missing fitted channel(s): {', '.join(missing_stall)}",
                )
            )
        else:
            gates.append(
                Gate(
                    "Direct stall detection",
                    "PASS" if trials_ok else "FAIL",
                    f"worst latency {worst:.0f} ms across {len(fitted_current)} channel(s)",
                    f"<= {limit:.0f} ms; stalled channel isolated; other channels operate",
                )
            )

    pattern_rows = data.get("pattern_trials", [])
    pattern_by_class: dict[str, list[bool]] = defaultdict(list)
    for row in pattern_rows:
        pattern_by_class[str(row["expected_grip"])].append(
            row.get("predicted_grip") == row.get("expected_grip")
        )
    if len(pattern_by_class) < 3 or any(len(rows) < 10 for rows in pattern_by_class.values()):
        gates.append(
            pending(
                "Pattern recognition",
                "3+ grips, 10+ blinded trials/grip, accuracy gates",
                f"classes/trials: {', '.join(f'{k}:{len(v)}' for k, v in pattern_by_class.items()) or 'none'}",
            )
        )
    else:
        all_trials = [result for rows in pattern_by_class.values() for result in rows]
        overall = sum(all_trials) / len(all_trials)
        worst_class = min(sum(rows) / len(rows) for rows in pattern_by_class.values())
        overall_min = threshold(data, "pattern_min_overall_accuracy")
        class_min = threshold(data, "pattern_min_class_accuracy")
        ok = overall >= overall_min and worst_class >= class_min
        gates.append(
            Gate(
                "Pattern recognition",
                "PASS" if ok else "FAIL",
                f"overall {overall:.1%}; worst class {worst_class:.1%}",
                f">= {overall_min:.0%} overall and >= {class_min:.0%} each class",
            )
        )

    forces = data.get("grasp_force", [])
    if not forces:
        gates.append(
            pending(
                "Grasp force",
                "power grasp reaches practical minimum; pinch stays below maximum",
                "no load-cell trials",
            )
        )
    else:
        power = [number(row["measured_newtons"]) for row in forces if row.get("grip") == "power"]
        pinch = [number(row["measured_newtons"]) for row in forces if row.get("grip") == "pinch"]
        if len(power) < 5 or len(pinch) < 5:
            gates.append(
                pending(
                    "Grasp force",
                    "5+ power and pinch trials",
                    f"power={len(power)}, pinch={len(pinch)}",
                )
            )
        else:
            practical = threshold(data, "minimum_practical_grip_newtons")
            pinch_max = threshold(data, "maximum_pinch_newtons")
            ok = statistics.mean(power) >= practical and max(pinch) <= pinch_max
            gates.append(
                Gate(
                    "Grasp force",
                    "PASS" if ok else "FAIL",
                    f"power mean {statistics.mean(power):.1f} N; pinch max {max(pinch):.1f} N",
                    f"power >= {practical:.0f} N; pinch <= {pinch_max:.0f} N",
                )
            )

    life = data.get("cycle_life", [])
    if not life:
        gates.append(
            pending("Cycle life", "survive benchmark without functional loss", "no cycle campaign")
        )
    else:
        best = max(life, key=lambda row: number(row["cycles"]))
        cycles = int(number(best["cycles"]))
        minimum = int(threshold(data, "minimum_grasp_cycles"))
        service = int(threshold(data, "service_goal_grasp_cycles"))
        intact = not bool(best.get("failure")) and bool(best.get("inspection_pass"))
        status = "PASS" if cycles >= minimum and intact else "FAIL"
        gates.append(
            Gate(
                "Cycle life",
                status,
                f"{cycles:,} cycles; {'intact' if intact else 'functional loss'}",
                f">= {minimum:,} baseline ({service:,} service goal)",
            )
        )

    timing = data.get("timing_trials", [])
    timing_limit = threshold(data, "timing_max_tick_ms", default=10.0)
    if not timing:
        gates.append(
            pending(
                "Control timing",
                f"measured worst tick <= {timing_limit:.1f} ms and watchdog reset verified",
                "no bench timing measurements",
            )
        )
    else:
        worst_tick = max(number(row["worst_tick_ms"]) for row in timing)
        wdg_ok = all(bool(row.get("watchdog_reset_verified")) for row in timing)
        ok = worst_tick <= timing_limit and wdg_ok
        gates.append(
            Gate(
                "Control timing",
                "PASS" if ok else "FAIL",
                f"worst tick {worst_tick:.2f} ms; watchdog {'verified' if wdg_ok else 'unverified'}",
                f"<= {timing_limit:.1f} ms and watchdog reset verified",
            )
        )

    disconnect = data.get("safety_disconnect")
    disconnect_fields = (
        "reset_loss_of_torque_verified",
        "oe_disable_verified",
        "i2c_bus_fault_loss_of_torque_verified",
        "independent_power_cut_verified",
    )
    if not isinstance(disconnect, dict) or not disconnect:
        gates.append(
            pending(
                "Hardware disconnect / STO",
                "physical torque-loss verified for reset, OE, I2C fault, and power cut",
                "no bench disconnect evidence",
            )
        )
    else:
        missing_checks = [k for k in disconnect_fields if k not in disconnect]
        if missing_checks:
            gates.append(
                pending(
                    "Hardware disconnect / STO",
                    "physical torque-loss verified for reset, OE, I2C fault, and power cut",
                    f"missing: {', '.join(missing_checks)}",
                )
            )
        else:
            failed_checks = [k for k in disconnect_fields if not bool(disconnect[k])]
            gates.append(
                Gate(
                    "Hardware disconnect / STO",
                    "PASS" if not failed_checks else "FAIL",
                    "all 4 physical checks verified"
                    if not failed_checks
                    else f"failed: {', '.join(failed_checks)}",
                    "physical torque-loss verified for reset, OE, I2C fault, and power cut",
                )
            )

    return gates


def render(data: dict[str, Any], gates: list[Gate]) -> str:
    overall = (
        "PASS"
        if all(g.status == "PASS" for g in gates)
        else ("FAIL" if any(g.status == "FAIL" for g in gates) else "PENDING")
    )
    meta = data.get("metadata", {})
    profile = resolve_profile(data)
    lines = [
        "# Jensen Hand verification report",
        "",
        f"Overall: **{overall}**",
        "",
        f"Device: `{meta.get('device_id', '')}`  ",
        f"Hardware profile: `{profile.name}`  ",
        f"Firmware: `{meta.get('firmware_commit', '')}`  ",
        f"Test date: `{meta.get('date_utc', '')}`",
        "",
        "| Gate | Status | Result | Criterion |",
        "|---|---|---|---|",
    ]
    for gate in gates:
        result = gate.value + (f" ({gate.detail})" if gate.detail else "")
        lines.append(f"| {gate.name} | **{gate.status}** | {result} | {gate.criterion} |")
    lines += [
        "",
        "A PENDING or FAIL report is not bench validation. Keep raw measurements, instrument IDs, and this report together.",
        "",
    ]
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
