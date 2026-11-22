"""Host unit tests for profile-aware campaign eligibility (Jensen-prosthetics #4).

Exercises missing channels, design-stage vs identified-assembly profiles,
synthetic/example evidence rejection, unfitted current/FSR claims, timing
discontinuities, and incomplete bench evidence while preserving all physical
release gates.
"""

from __future__ import annotations

import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

VERIFICATION_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(VERIFICATION_DIR))

import verify_campaign as vc  # noqa: E402


def _gate_map(gates: list[vc.Gate]) -> dict[str, vc.Gate]:
    return {g.name: g for g in gates}


def _complete_identified_fixture() -> dict:
    """In-memory test fixture satisfying every gate on an identified assembly.

    Exists solely to test positive-path gate logic; not physical bench evidence.
    """
    return {
        "metadata": {
            "device_id": "JH-BENCH-UNIT-01",
            "firmware_commit": "7791097abcdef1234567890",
            "operator": "bench-engineer",
            "date_utc": "2026-11-21T12:00:00Z",
            "instruments": [
                {"id": "DMM-101", "calibration_due": "2027-06-01"},
                {"id": "LOADCELL-202", "calibration_due": "2027-06-01"},
            ],
        },
        "hardware_profile": {
            "name": "identified_assembly",
            "assembly_id": "JH-REV1-SN01",
            "actuators": ["5x-MG996R-digits", "1x-MG996R-thumb-rot"],
            "has_thumb_opposition": True,
            "has_wrist": False,
            "fitted_fsr_channels": ["thumb", "index", "middle"],
            "fitted_current_channels": ["index", "middle"],
            "emg_front_end_isolated": True,
        },
        "thresholds": {
            "battery_max_error_percent": 2.0,
            "fsr_mean_error_percent": 20.0,
            "stall_max_latency_ms": 300.0,
            "pattern_min_overall_accuracy": 0.90,
            "pattern_min_class_accuracy": 0.80,
            "minimum_practical_grip_newtons": 45.0,
            "maximum_pinch_newtons": 65.0,
            "minimum_grasp_cycles": 300000,
            "service_goal_grasp_cycles": 500000,
            "timing_max_tick_ms": 10.0,
        },
        "battery": [
            {"reference_volts": 8.40, "telemetry_volts": 8.39},
            {"reference_volts": 7.40, "telemetry_volts": 7.41},
            {"reference_volts": 6.60, "telemetry_volts": 6.59},
        ],
        "servo_endpoints": [
            {"digit": "thumb", "open_us": 1000, "closed_us": 2000, "clearance_pass": True},
            {"digit": "thumb_rot", "open_us": 1100, "closed_us": 1900, "clearance_pass": True},
            {"digit": "index", "open_us": 1000, "closed_us": 2000, "clearance_pass": True},
            {"digit": "middle", "open_us": 1000, "closed_us": 2000, "clearance_pass": True},
            {"digit": "ring", "open_us": 1000, "closed_us": 2000, "clearance_pass": True},
            {"digit": "pinky", "open_us": 1000, "closed_us": 2000, "clearance_pass": True},
        ],
        "fsr": [
            {"sensor": s, "reference_newtons": load, "telemetry_newtons": load + 0.1}
            for s in ("thumb", "index", "middle")
            for load in (5.0, 10.0, 15.0)
        ],
        "stall_trials": [
            {
                "digit": "index",
                "detection_latency_ms": 220,
                "isolated_channel": True,
                "other_channels_operational": True,
            },
            {
                "digit": "middle",
                "detection_latency_ms": 240,
                "isolated_channel": True,
                "other_channels_operational": True,
            },
        ],
        "pattern_trials": [
            {"expected_grip": grip, "predicted_grip": grip}
            for grip in ("power", "pinch", "tripod")
            for _ in range(10)
        ],
        "grasp_force": [
            *[{"grip": "power", "measured_newtons": 50.0} for _ in range(5)],
            *[{"grip": "pinch", "measured_newtons": 35.0} for _ in range(5)],
        ],
        "cycle_life": [{"cycles": 310000, "failure": False, "inspection_pass": True}],
        "timing_trials": [{"worst_tick_ms": 9.2, "watchdog_reset_verified": True}],
        "safety_disconnect": {
            "reset_loss_of_torque_verified": True,
            "oe_disable_verified": True,
            "i2c_bus_fault_loss_of_torque_verified": True,
            "independent_power_cut_verified": True,
        },
    }


class TestVerifyCampaignProfiles(unittest.TestCase):
    def test_template_campaign_is_pending_exit_2(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(VERIFICATION_DIR / "campaign_template.json")],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 2, proc.stdout + proc.stderr)
        self.assertIn("Overall: **PENDING**", proc.stdout)

    def test_example_campaign_fails_provenance_and_unfitted_stall_claim(self) -> None:
        data = json.loads((VERIFICATION_DIR / "example_campaign.json").read_text(encoding="utf-8"))
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Evidence provenance"].status, "FAIL")
        self.assertEqual(gates["Hardware profile"].status, "PENDING")
        # Example campaign has no fitted current channels declared, yet claims an index stall trial.
        self.assertEqual(gates["Direct stall detection"].status, "FAIL")

        proc = subprocess.run(
            [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(VERIFICATION_DIR / "example_campaign.json")],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 1)

    def test_synthetic_flag_blocks_release_even_when_all_numbers_pass(self) -> None:
        data = _complete_identified_fixture()
        data["metadata"]["synthetic"] = True
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Evidence provenance"].status, "FAIL")

    def test_cad_design_stage_profile_retains_physical_and_stall_gates_pending(self) -> None:
        data = _complete_identified_fixture()
        data["hardware_profile"] = {"name": "cad_design_stage"}
        data["stall_trials"] = []
        # CAD BOM includes a wrist MG996R -> missing wrist endpoint keeps Servo endpoints PENDING.
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Hardware profile"].status, "PENDING")
        self.assertEqual(gates["Servo endpoints"].status, "PENDING")
        self.assertIn("wrist", gates["Servo endpoints"].detail)
        self.assertEqual(gates["Direct stall detection"].status, "PENDING")

    def test_cad_profile_rejects_fabricated_stall_trial_without_current_channels(self) -> None:
        data = _complete_identified_fixture()
        data["hardware_profile"] = {"name": "cad_design_stage"}
        # Keeping stall_trials when cad_design_stage has 0 fitted current channels must FAIL.
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Direct stall detection"].status, "FAIL")

    def test_firmware_default_profile_requires_thumb_rot_and_remains_pending(self) -> None:
        data = _complete_identified_fixture()
        data["hardware_profile"] = {"name": "firmware_default"}
        data["stall_trials"] = []
        data["servo_endpoints"] = [r for r in data["servo_endpoints"] if r["digit"] != "thumb_rot"]
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Hardware profile"].status, "PENDING")
        self.assertEqual(gates["Servo endpoints"].status, "PENDING")
        self.assertIn("thumb_rot", gates["Servo endpoints"].detail)

    def test_identified_assembly_missing_one_fitted_current_channel_is_pending(self) -> None:
        data = _complete_identified_fixture()
        data["stall_trials"] = [r for r in data["stall_trials"] if r["digit"] == "index"]
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Direct stall detection"].status, "PENDING")
        self.assertIn("middle", gates["Direct stall detection"].detail)

    def test_identified_assembly_rejects_unisolated_emg_front_end(self) -> None:
        data = _complete_identified_fixture()
        data["hardware_profile"]["emg_front_end_isolated"] = False
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Hardware profile"].status, "FAIL")

    def test_timing_discontinuity_over_budget_fails_control_timing_gate(self) -> None:
        data = _complete_identified_fixture()
        data["timing_trials"] = [{"worst_tick_ms": 14.5, "watchdog_reset_verified": True}]
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Control timing"].status, "FAIL")

    def test_incomplete_hardware_disconnect_evidence_is_pending(self) -> None:
        data = _complete_identified_fixture()
        del data["safety_disconnect"]["independent_power_cut_verified"]
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["Hardware disconnect / STO"].status, "PENDING")
        self.assertIn("independent_power_cut_verified", gates["Hardware disconnect / STO"].detail)

    def test_unfitted_fsr_sensor_fails_fsr_gate(self) -> None:
        data = _complete_identified_fixture()
        data["fsr"].append({"sensor": "pinky", "reference_newtons": 10.0, "telemetry_newtons": 10.0})
        gates = _gate_map(vc.evaluate(data))
        self.assertEqual(gates["FSR force fit"].status, "FAIL")

    def test_unknown_profile_exits_3_as_invalid_campaign(self) -> None:
        data = _complete_identified_fixture()
        data["hardware_profile"] = {"name": "nonexistent_profile"}
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "bad.json"
            p.write_text(json.dumps(data), encoding="utf-8")
            proc = subprocess.run(
                [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(p)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 3)

    def test_complete_identified_assembly_fixture_passes_all_gates_exit_0(self) -> None:
        data = copy.deepcopy(_complete_identified_fixture())
        gates = vc.evaluate(data)
        self.assertTrue(all(g.status == "PASS" for g in gates), gates)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "pass.json"
            p.write_text(json.dumps(data), encoding="utf-8")
            proc = subprocess.run(
                [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(p)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            self.assertIn("Overall: **PASS**", proc.stdout)

    def test_all_boolean_evidence_fields_reject_non_boolean_values_exit_3(self) -> None:
        """Every boolean evidence/profile field must reject truthy strings, ints, and None."""
        mutations = [
            ("metadata.synthetic", lambda d, v: d["metadata"].__setitem__("synthetic", v)),
            (
                "hardware_profile.has_thumb_opposition",
                lambda d, v: d["hardware_profile"].__setitem__("has_thumb_opposition", v),
            ),
            (
                "hardware_profile.has_wrist",
                lambda d, v: d["hardware_profile"].__setitem__("has_wrist", v),
            ),
            (
                "hardware_profile.emg_front_end_isolated",
                lambda d, v: d["hardware_profile"].__setitem__("emg_front_end_isolated", v),
            ),
            (
                "servo_endpoints[0].clearance_pass",
                lambda d, v: d["servo_endpoints"][0].__setitem__("clearance_pass", v),
            ),
            (
                "stall_trials[0].isolated_channel",
                lambda d, v: d["stall_trials"][0].__setitem__("isolated_channel", v),
            ),
            (
                "stall_trials[0].other_channels_operational",
                lambda d, v: d["stall_trials"][0].__setitem__("other_channels_operational", v),
            ),
            (
                "cycle_life[0].failure",
                lambda d, v: d["cycle_life"][0].__setitem__("failure", v),
            ),
            (
                "cycle_life[0].inspection_pass",
                lambda d, v: d["cycle_life"][0].__setitem__("inspection_pass", v),
            ),
            (
                "timing_trials[0].watchdog_reset_verified",
                lambda d, v: d["timing_trials"][0].__setitem__("watchdog_reset_verified", v),
            ),
            (
                "safety_disconnect.reset_loss_of_torque_verified",
                lambda d, v: d["safety_disconnect"].__setitem__("reset_loss_of_torque_verified", v),
            ),
            (
                "safety_disconnect.oe_disable_verified",
                lambda d, v: d["safety_disconnect"].__setitem__("oe_disable_verified", v),
            ),
            (
                "safety_disconnect.i2c_bus_fault_loss_of_torque_verified",
                lambda d, v: d["safety_disconnect"].__setitem__(
                    "i2c_bus_fault_loss_of_torque_verified", v
                ),
            ),
            (
                "safety_disconnect.independent_power_cut_verified",
                lambda d, v: d["safety_disconnect"].__setitem__(
                    "independent_power_cut_verified", v
                ),
            ),
        ]
        bad_values = ["false", "true", 0, 1, None]
        with tempfile.TemporaryDirectory() as tmp:
            for field_name, mutate in mutations:
                for bad in bad_values:
                    # emg_front_end_isolated=None explicitly represents PENDING (missing field).
                    if field_name == "hardware_profile.emg_front_end_isolated" and bad is None:
                        continue
                    with self.subTest(field=field_name, value=bad):
                        data = copy.deepcopy(_complete_identified_fixture())
                        mutate(data, bad)
                        with self.assertRaises(ValueError):
                            vc.evaluate(data)
                        if bad == "false":
                            p = Path(tmp) / "bad_bool.json"
                            p.write_text(json.dumps(data), encoding="utf-8")
                            proc = subprocess.run(
                                [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(p)],
                                capture_output=True,
                                text=True,
                                check=False,
                            )
                            self.assertEqual(proc.returncode, 3, proc.stdout + proc.stderr)

    def test_false_safety_confirmation_fields_fail_gates(self) -> None:
        safety_fields = [
            (
                "Hardware profile",
                lambda d: d["hardware_profile"].__setitem__("emg_front_end_isolated", False),
            ),
            (
                "Servo endpoints",
                lambda d: d["servo_endpoints"][0].__setitem__("clearance_pass", False),
            ),
            (
                "Direct stall detection",
                lambda d: d["stall_trials"][0].__setitem__("isolated_channel", False),
            ),
            (
                "Direct stall detection",
                lambda d: d["stall_trials"][0].__setitem__("other_channels_operational", False),
            ),
            (
                "Cycle life",
                lambda d: d["cycle_life"][0].__setitem__("inspection_pass", False),
            ),
            (
                "Cycle life",
                lambda d: d["cycle_life"][0].__setitem__("failure", True),
            ),
            (
                "Control timing",
                lambda d: d["timing_trials"][0].__setitem__("watchdog_reset_verified", False),
            ),
            (
                "Hardware disconnect / STO",
                lambda d: d["safety_disconnect"].__setitem__("reset_loss_of_torque_verified", False),
            ),
            (
                "Hardware disconnect / STO",
                lambda d: d["safety_disconnect"].__setitem__("oe_disable_verified", False),
            ),
            (
                "Hardware disconnect / STO",
                lambda d: d["safety_disconnect"].__setitem__(
                    "i2c_bus_fault_loss_of_torque_verified", False
                ),
            ),
            (
                "Hardware disconnect / STO",
                lambda d: d["safety_disconnect"].__setitem__(
                    "independent_power_cut_verified", False
                ),
            ),
        ]
        for gate_name, mutate in safety_fields:
            with self.subTest(gate=gate_name):
                data = copy.deepcopy(_complete_identified_fixture())
                mutate(data)
                gates = _gate_map(vc.evaluate(data))
                self.assertEqual(gates[gate_name].status, "FAIL")

    def test_negative_numeric_evidence_fields_rejected_exit_3(self) -> None:
        numeric_mutations = [
            ("timing_trials.worst_tick_ms", lambda d: d["timing_trials"][0].__setitem__("worst_tick_ms", -1)),
            (
                "stall_trials.detection_latency_ms",
                lambda d: d["stall_trials"][0].__setitem__("detection_latency_ms", -5),
            ),
            (
                "grasp_force.measured_newtons",
                lambda d: d["grasp_force"][0].__setitem__("measured_newtons", -10.0),
            ),
            ("cycle_life.cycles", lambda d: d["cycle_life"][0].__setitem__("cycles", -100)),
            ("servo_endpoints.open_us", lambda d: d["servo_endpoints"][0].__setitem__("open_us", -1)),
            ("battery.reference_volts", lambda d: d["battery"][0].__setitem__("reference_volts", -1.0)),
            ("fsr.reference_newtons", lambda d: d["fsr"][0].__setitem__("reference_newtons", -2.0)),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            for field_name, mutate in numeric_mutations:
                with self.subTest(field=field_name):
                    data = copy.deepcopy(_complete_identified_fixture())
                    mutate(data)
                    with self.assertRaises(ValueError):
                        vc.evaluate(data)
                    p = Path(tmp) / "neg_num.json"
                    p.write_text(json.dumps(data), encoding="utf-8")
                    proc = subprocess.run(
                        [sys.executable, str(VERIFICATION_DIR / "verify_campaign.py"), str(p)],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    self.assertEqual(proc.returncode, 3, proc.stdout + proc.stderr)


if __name__ == "__main__":
    unittest.main()
