"""Verify surface tracking and recovery from out-of-range measurements."""

import math
import statistics

from pymavlink import mavutil


RANGE_CRITERIA = {
    "baseline_measure_s": 8, "response_settle_s": 12, "response_measure_s": 6,
    "synthetic_position_step_m": 1.0, "step_altitude_change_m": [0.6, 1.4],
    "return_altitude_error_max_m": 0.5, "out_of_range_position_z_m": -50.0,
    "fallback_measure_s": 10, "fallback_altitude_change_max_m": 0.75,
    "range_truth_error_max_m": 0.5, "range_max_age_s": 1.0,
    "measurement_min_samples": 30, "horizontal_error_max_m": 2.0,
    "log_min_samples_per_stream": 5,
}


def measurements(flight, name, require_in_range, duration=None):
    if duration is not None:
        flight.step(name)
        flight.duration(duration)
    rows = [r for r in flight.rows if r["phase"] == name]
    if len(rows) < RANGE_CRITERIA["measurement_min_samples"]:
        raise RuntimeError(f"{name}: insufficient measurement samples")
    for row in rows:
        needed = ("relative_alt_m", "true_altitude_relative_m", "truth_horizontal_error_m",
                  "distance_m", "distance_min_m", "distance_max_m", "distance_age_s", "tilt_deg")
        if any(row[k] is None or not math.isfinite(row[k]) for k in needed):
            raise RuntimeError(f"{name}: missing/nonfinite range or navigation data")
        if row["distance_age_s"] > 1 or row["true_sample_age_s"] is None or row["true_sample_age_s"] > 1:
            raise RuntimeError(f"{name}: stale sensor/truth data")
        if row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > 2 or not row["armed"] or row["mode"] != flight.link.mode_mapping()["LOITER"]:
            raise RuntimeError(f"{name}: needs fresh armed LOITER heartbeat")
        if row["truth_horizontal_error_m"] > RANGE_CRITERIA["horizontal_error_max_m"]:
            raise RuntimeError(f"{name}: horizontal position error > 2 m")
        in_range = row["distance_min_m"] <= row["distance_m"] <= row["distance_max_m"]
        if require_in_range != in_range:
            raise RuntimeError(f"{name}: unexpected range validity")
    result = {
        "samples": len(rows), "start_s": rows[0]["time_boot_ms"] / 1000,
        "end_s": rows[-1]["time_boot_ms"] / 1000,
        "altitude_mean_m": statistics.fmean(r["relative_alt_m"] for r in rows),
        "altitude_min_m": min(r["relative_alt_m"] for r in rows),
        "altitude_max_m": max(r["relative_alt_m"] for r in rows),
        "true_altitude_mean_m": statistics.fmean(r["true_altitude_relative_m"] for r in rows),
        "true_altitude_min_m": min(r["true_altitude_relative_m"] for r in rows),
        "true_altitude_max_m": max(r["true_altitude_relative_m"] for r in rows),
        "distance_mean_m": statistics.fmean(r["distance_m"] for r in rows),
        "distance_min_m": min(r["distance_m"] for r in rows),
        "distance_max_m": max(r["distance_m"] for r in rows),
        "truth_horizontal_error_max_m": max(r["truth_horizontal_error_m"] for r in rows),
        "laser_health_bit_values": sorted(set(r["laser_health_bit"] for r in rows)),
        "sensor_position_z_m": flight.sonar_position_z,
    }
    flight.summary["metrics"].setdefault("rangefinder", {})[name] = result
    return result


def run(flight):
    baseline = measurements(flight, "range_baseline", True, 8)
    rows = [r for r in flight.rows if r["phase"] == "range_baseline"]
    errors = [abs(r["distance_m"] * math.cos(math.radians(r["tilt_deg"])) - r["true_altitude_relative_m"]) for r in rows]
    baseline["range_truth_error_max_m"] = max(errors)
    if max(errors) > RANGE_CRITERIA["range_truth_error_max_m"] or not 9 <= baseline["altitude_mean_m"] <= 11:
        raise RuntimeError("Baseline range/true height disagreement")
    flight.step("range_bias_injection")
    flight.set_parameter("SIM_SONAR_POS_Z", 1)
    flight.duration(12)
    shifted = measurements(flight, "range_bias_response", True, 6)
    delta = shifted["altitude_mean_m"] - baseline["altitude_mean_m"]
    shifted["altitude_change_m"] = delta
    truth_delta = shifted["true_altitude_mean_m"] - baseline["true_altitude_mean_m"]
    shifted["true_altitude_change_m"] = truth_delta
    if not 0.6 <= delta <= 1.4 or not 0.6 <= truth_delta <= 1.4:
        raise RuntimeError(f"Surface tracking did not respond to 1 m synthetic range step: estimated={delta}, truth={truth_delta}")
    flight.step("range_bias_restore")
    flight.set_parameter("SIM_SONAR_POS_Z", 0)
    flight.duration(12)
    restored = measurements(flight, "range_bias_restored", True, 6)
    restored["altitude_change_m"] = restored["altitude_mean_m"] - baseline["altitude_mean_m"]
    restored["true_altitude_change_m"] = restored["true_altitude_mean_m"] - baseline["true_altitude_mean_m"]
    if abs(restored["altitude_change_m"]) > 0.5 or abs(restored["true_altitude_change_m"]) > 0.5:
        raise RuntimeError("Surface tracking did not return after restoring range offset")
    flight.step("range_out_of_range_injection")
    flight.set_parameter("SIM_SONAR_POS_Z", -50)
    flight.wait("distance beyond configured maximum", lambda m: m.get_type() == "DISTANCE_SENSOR" and
        m.current_distance > m.max_distance, sim_timeout=10)
    flight.duration(5)
    fallback = measurements(flight, "range_out_of_range_hold", False, 10)
    fallback["altitude_change_m"] = fallback["altitude_mean_m"] - restored["altitude_mean_m"]
    fallback["true_altitude_change_m"] = fallback["true_altitude_mean_m"] - restored["true_altitude_mean_m"]
    if max(abs(fallback["altitude_min_m"] - restored["altitude_mean_m"]),
           abs(fallback["altitude_max_m"] - restored["altitude_mean_m"]),
           abs(fallback["true_altitude_min_m"] - restored["true_altitude_mean_m"]),
           abs(fallback["true_altitude_max_m"] - restored["true_altitude_mean_m"])) > 0.75:
        raise RuntimeError("Altitude was not maintained after out-of-range readings")
    flight.step("range_recovery")
    flight.set_parameter("SIM_SONAR_POS_Z", 0)
    flight.wait("distance recovered within limits", lambda m: m.get_type() == "DISTANCE_SENSOR" and
        m.min_distance <= m.current_distance <= m.max_distance, sim_timeout=10)
    flight.duration(8)
    recovered = measurements(flight, "range_recovered_hold", True, 10)
    recovered["altitude_change_m"] = recovered["altitude_mean_m"] - restored["altitude_mean_m"]
    recovered["true_altitude_change_m"] = recovered["true_altitude_mean_m"] - restored["true_altitude_mean_m"]
    if max(abs(recovered["altitude_min_m"] - restored["altitude_mean_m"]),
           abs(recovered["altitude_max_m"] - restored["altitude_mean_m"]),
           abs(recovered["true_altitude_min_m"] - restored["true_altitude_mean_m"]),
           abs(recovered["true_altitude_max_m"] - restored["true_altitude_mean_m"])) > 0.75:
        raise RuntimeError("Altitude jumped after range recovery")


def verify_range_log(run_dir, summary):
    """Gate success on flight-controller states, not just GCS health-bit telemetry."""
    files = sorted((run_dir / "logs").glob("*.BIN"))
    if not files:
        raise RuntimeError("Rangefinder test requires a DataFlash BIN log")
    windows = summary["metrics"]["rangefinder"]
    phases = ("range_baseline", "range_bias_response", "range_bias_restored", "range_out_of_range_hold", "range_recovered_hold")
    evidence = {phase: {"RFND": [], "SURF": [], "CTUN": []} for phase in phases}
    for file in files:
        log = mavutil.mavlink_connection(str(file))
        try:
            while True:
                message = log.recv_match(type=["RFND", "SURF", "CTUN"])
                if message is None:
                    break
                timestamp = message.TimeUS / 1e6
                for phase in phases:
                    window = windows[phase]
                    if window["start_s"] + 0.5 <= timestamp <= window["end_s"]:
                        kind = message.get_type()
                        if kind == "RFND" and message.Instance == 0:
                            evidence[phase][kind].append(int(message.Stat))
                        elif kind == "SURF" and message.I == 0:
                            evidence[phase][kind].append(int(message.St))
                        elif kind == "CTUN":
                            evidence[phase][kind].append(math.isfinite(message.DSAlt))
        finally:
            log.close()
    report = {}
    for phase, values in evidence.items():
        if any(len(v) < 5 for v in values.values()):
            raise RuntimeError(f"{phase}: missing RFND/SURF/CTUN log evidence: { {k:len(v) for k,v in values.items()} }")
        fault = phase == "range_out_of_range_hold"
        report[phase] = {"sample_counts": {k: len(v) for k, v in values.items()},
                         "RFND_Stat_values": sorted(set(values["RFND"])),
                         "SURF_St_values": sorted(set(values["SURF"])),
                         "CTUN_DSAlt_finite_fraction": sum(values["CTUN"]) / len(values["CTUN"])}
        if fault:
            good = all(v == 3 for v in values["RFND"]) and all(v & 2 for v in values["SURF"]) and not any(values["CTUN"])
        else:
            good = all(v == 4 for v in values["RFND"]) and all(v & 1 and not v & 2 for v in values["SURF"]) and all(values["CTUN"])
        if not good:
            summary["range_log_evidence"] = report
            raise RuntimeError(f"{phase}: controller range status evidence did not match expected state: {report[phase]}")
    summary["range_log_evidence"] = report
