#!/usr/bin/env python3
"""Render measured prototype mission/rangefinder figures from a passed suite.

Optional dependency: install requirements-plots.txt in a separate environment.
This script never starts SITL or changes an existing run's data.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EARTH_RADIUS_M = 6378137.0
EXPECTED_WAYPOINTS = [(20, 0), (20, 20), (0, 20), (0, 0)]
COLORS = ["#2166ac", "#1b9e77", "#d95f02"]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def load_suite(path: Path) -> tuple[Path, dict]:
    suite_path = path if path.is_file() else path / "suite.json"
    suite_path = suite_path.resolve(strict=True)
    suite = read_json(suite_path)
    runs = suite.get("runs", [])
    if (suite.get("result") != "passed" or not suite.get("finished_utc") or not runs
            or suite.get("run_count") != len(runs)
            or suite.get("passed_count") != len(runs)
            or any(r.get("result") != "passed" or r.get("exit_code") != 0 for r in runs)):
        raise ValueError("Only a completed, fully passed suite can be plotted")
    labels = [r["label"] for r in runs]
    if len(set(labels)) != len(labels):
        raise ValueError("Suite contains duplicate run labels")
    return suite_path, suite


def load_run(suite_path: Path, entry: dict) -> dict:
    folder = (suite_path.parent / entry["directory"]).resolve(strict=True)
    if not folder.is_relative_to(suite_path.parent):
        raise ValueError("Run directory must remain inside the suite directory")
    summary_path, run_path, csv_path = (folder / name for name in
                                       ("summary.json", "run.json", "telemetry.csv"))
    if digest(summary_path) != entry.get("summary_sha256"):
        raise ValueError(f"{entry['label']}: summary SHA does not match suite")
    summary, metadata = read_json(summary_path), read_json(run_path)
    if summary.get("result") != "passed":
        raise ValueError(f"{entry['label']}: run summary is not passed")
    for field in ("label", "case", "scenario"):
        if summary.get(field) != entry[field] or metadata.get(field) != entry[field]:
            raise ValueError(f"{entry['label']}: inconsistent {field}")
    with csv_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{entry['label']}: empty telemetry")
    return {"entry": entry, "summary": summary, "metadata": metadata, "rows": rows,
            "source": {"run_label": entry["label"], "run_directory": entry["directory"],
                       "summary_sha256": digest(summary_path), "telemetry_sha256": digest(csv_path),
                       "run_metadata_sha256": digest(run_path),
                       "project_commit": metadata.get("project_commit"),
                       "binary_sha256": metadata.get("binary_sha256")}}


def finite(row: dict, field: str) -> float:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Missing/non-numeric telemetry field {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"Non-finite telemetry field {field}")
    return value


def planned_waypoints(run: dict) -> list[tuple[float, float]]:
    items = run["summary"]["mission"]["items"]
    by_seq = {item["seq"]: item for item in items}
    result = []
    for seq, expected in enumerate(EXPECTED_WAYPOINTS, 1):
        item = by_seq[seq]
        point = (float(item["north_m"]), float(item["east_m"]))
        if item["command"] != 16 or any(not math.isclose(a, b, abs_tol=1e-6)
                                        for a, b in zip(point, expected)):
            raise ValueError("Figure expects the recorded 20 m square waypoint mission")
        result.append(point)
    return result


def plot_mission(plt, runs: list[dict]):
    fig, axis = plt.subplots(figsize=(8.6, 7.4))
    fig.subplots_adjust(left=.12, right=.96, top=.88, bottom=.19)
    plan = planned_waypoints(runs[0])
    planned_n, planned_e = zip(*([(0.0, 0.0)] + plan))
    axis.plot(planned_e, planned_n, "--", color="#64748b", linewidth=2,
              label="Planned route", zorder=1)
    for color, run in zip(COLORS, runs):
        if planned_waypoints(run) != plan:
            raise ValueError("Mission repeats use different planned waypoints")
        home = run["summary"]["navigation_home"]
        lat0, lon0 = float(home["lat_deg"]), float(home["lon_deg"])
        points = []
        for row in run["rows"]:
            if row["phase"] != "auto_mission":
                continue
            age = finite(row, "true_sample_age_s")
            if not 0 <= age <= 1.0:
                raise ValueError(f"{run['entry']['label']}: stale mission truth sample")
            lat, lon = finite(row, "true_lat_deg"), finite(row, "true_lon_deg")
            north = math.radians(lat - lat0) * EARTH_RADIUS_M
            east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
            points.append((east, north))
        if len(points) < 20:
            raise ValueError(f"{run['entry']['label']}: insufficient AUTO trajectory samples")
        east, north = zip(*points)
        axis.plot(east, north, color=color, linewidth=1.4, alpha=.9,
                  label=f"Measured truth: repeat {run['entry']['repeat']}", zorder=2)
    for seq, (north, east) in enumerate(plan, 1):
        axis.scatter([east], [north], s=34, facecolors="white", edgecolors="#334155",
                     linewidths=1.1, zorder=4)
        label = f"WP{seq}" if seq != 4 else "WP4 / home"
        offset = (7, 8) if north < 10 else (7, -16)
        axis.annotate(label, (east, north), xytext=offset, textcoords="offset points",
                      fontsize=10, color="#334155")
    axis.set(xlabel="East from navigation home (m)", ylabel="North from navigation home (m)",
             xlim=(-2, 24), ylim=(-2, 23), aspect="equal")
    axis.legend(loc="center", bbox_to_anchor=(.5, -.155), ncol=2, frameon=False, fontsize=9)
    fig.suptitle("B-D prototype: AUTO mission, three repeats", x=.12, ha="left", fontsize=15)
    axis.set_title("20 m square; each run uses its own recorded navigation home", fontsize=10,
                   color="#475569", loc="left", pad=14)
    labels = ", ".join(run["entry"]["label"] for run in runs)
    fig.text(.12, .025, "Sources: " + labels.replace(", nominal-mission-repeat-3", ",\nnominal-mission-repeat-3"),
             fontsize=7.6, color="#475569", va="bottom")
    return fig, {"figure": "mission-trajectories", "description": "Recorded simulator true horizontal trajectories, not estimated GPS paths",
                 "coordinate_frame": "East/North relative to each run's navigation_home lat/lon using R=6378137m",
                 "planned_waypoints_ne_m": plan, "sources": [r["source"] for r in runs]}


def event_time(summary: dict, phase: str) -> float:
    matches = [float(e["time_boot_s"]) for e in summary["events"] if e["phase"] == phase]
    if len(matches) != 1 or not math.isfinite(matches[0]):
        raise ValueError(f"Expected one recorded event for phase {phase}")
    return matches[0]


def plot_rangefinder(plt, run: dict):
    summary = run["summary"]
    windows = summary["metrics"]["rangefinder"]
    start = float(windows["range_baseline"]["start_s"])
    end = float(windows["range_recovered_hold"]["end_s"])
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        raise ValueError("Invalid recorded rangefinder measurement interval")
    records = []
    for row in run["rows"]:
        timestamp = finite(row, "time_boot_ms") / 1000
        if not start <= timestamp <= end:
            continue
        if not row["phase"].startswith("range_"):
            raise ValueError("Unexpected phase within rangefinder test interval")
        if not 0 <= finite(row, "distance_age_s") <= 1.0:
            raise ValueError("Stale rangefinder sample in plotting interval")
        if not 0 <= finite(row, "true_sample_age_s") <= 1.0:
            raise ValueError("Stale true altitude sample in plotting interval")
        records.append((timestamp - start, finite(row, "relative_alt_m"),
                        finite(row, "true_altitude_relative_m"), finite(row, "distance_m")))
    if len(records) < 50:
        raise ValueError("Insufficient recorded rangefinder samples")
    times, estimated, truth, distance = zip(*records)
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("Rangefinder timestamps are not ordered")
    if min(distance) < 0 or max(distance) > 65:
        raise ValueError("Recorded distances exceed this figure's explicit0..65m range")
    valid_max = float(summary["initial_parameters"]["RNGFND1_MAX"])
    if not math.isfinite(valid_max) or not 0 < valid_max < 65:
        raise ValueError("Invalid configured rangefinder maximum")
    intervals = [
        ("range_bias_injection", "range_bias_restore", "Sensor offset +1 m", "#f6c56c"),
        ("range_bias_restore", "range_out_of_range_injection", "Offset removed", "#c5ddc6"),
        ("range_out_of_range_injection", "range_recovery", "Out-of-range input", "#edb4b4"),
        ("range_recovery", None, "Valid input restored", "#c9d9ef"),
    ]
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10.6, 7.7),
                             gridspec_kw={"height_ratios": [1, 1]})
    fig.subplots_adjust(left=.10, right=.97, top=.84, bottom=.19, hspace=.18)
    shading = []
    for first, last, label, color in intervals:
        a, b = event_time(summary, first), event_time(summary, last) if last else end
        if not start <= a < b <= end + .5:
            raise ValueError(f"Invalid event interval {first}..{last}")
        shading.append({"start_phase": first, "end_phase": last, "start_s": a, "end_s": b})
        for axis in axes:
            axis.axvspan(a-start, b-start, color=color, alpha=.38, linewidth=0,
                         label=label if axis is axes[0] else None)
            axis.axvline(a-start, color="#94a3b8", linewidth=.7, alpha=.65)
    estimate_line, = axes[0].plot(times, estimated, color="#2166ac", linewidth=1.5, label="Estimated altitude")
    truth_line, = axes[0].plot(times, truth, color="#111827", linewidth=1.25, linestyle="--", label="Simulator true altitude")
    axes[0].set_ylabel("Height above initial position (m)")
    altitude_min, altitude_max = min(*estimated, *truth), max(*estimated, *truth)
    axes[0].set_ylim(min(9.0, altitude_min-.2), max(11.5, altitude_max+.2))
    axes[0].legend(handles=[estimate_line, truth_line], loc="upper left", frameon=False, fontsize=9)
    axes[1].plot(times, distance, color="#0f766e", linewidth=1.4, label="Reported distance")
    axes[1].axhline(valid_max, color="#64748b", linestyle=":", linewidth=1,
                   label=f"Configured valid maximum: {valid_max:g} m")
    axes[1].set(xlabel="Simulation time from baseline start (s)", ylabel="Downward reported distance (m)",
                xlim=(0, end-start), ylim=(0, 65))
    axes[1].legend(loc="upper left", frameon=False, fontsize=9)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(facecolor=c, alpha=.5, label=label) for _, _, label, c in intervals],
               loc="lower center", bbox_to_anchor=(.535, .075), ncol=2, frameon=False, fontsize=9)
    fig.suptitle("B-D prototype: rangefinder response", x=.10, ha="left", fontsize=15, y=.97)
    fig.text(.10, .90, "Representative run: nominal-rangefinder-repeat-1\n"
             "+1 m virtual sensor offset shortens the reading; this is a synthetic input, not a terrain change.",
             fontsize=9.5, color="#475569", va="top")
    fig.text(.10, .025, "Source: " + run["entry"]["label"] + "; measured CSV values, five-Hz native sensor emulator.",
             fontsize=8, color="#475569")
    return fig, {"figure": "rangefinder-response", "representative_run": run["entry"]["label"],
                 "description": "One recorded repeat; estimated/true altitude and reported distance during synthetic sensor offset/out-of-range/recovery",
                 "start_time_boot_s": start, "end_time_boot_s": end,
                 "shaded_intervals": shading, "sources": [run["source"]]}


def save_figure(fig, output: Path, stem: str, provenance: dict, suite_path: Path, suite: dict):
    provenance.update(suite_sha256=digest(suite_path), suite_finished_utc=suite["finished_utc"],
                      plotting_script_sha256=digest(Path(__file__)))
    description = json.dumps(provenance, ensure_ascii=True, sort_keys=True)
    common = {"Title": stem, "Description": description}
    fig.savefig(output / (stem + ".svg"), format="svg", metadata={**common,
                "Creator": "drone-sitl/scripts/plot_prototype.py", "Date": suite["finished_utc"]})
    fig.savefig(output / (stem + ".png"), format="png", dpi=180, metadata={**common,
                "Software": "drone-sitl/scripts/plot_prototype.py"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path, help="Completed passed suite directory or its suite.json")
    parser.add_argument("--output", required=True, type=Path, help="Directory for four SVG/PNG figure files")
    args = parser.parse_args()
    suite_path, suite = load_suite(args.suite)
    mission_entries = sorted((r for r in suite["runs"] if r["case"] == "nominal" and r["scenario"] == "mission"),
                             key=lambda r: r["repeat"])
    range_entries = [r for r in suite["runs"] if r["case"] == "nominal" and r["scenario"] == "rangefinder" and r["repeat"] == 1]
    if [r["repeat"] for r in mission_entries] != [1, 2, 3] or len(range_entries) != 1:
        raise ValueError("Figures require nominal mission repeats1–3 and nominal rangefinder repeat1")
    missions = [load_run(suite_path, entry) for entry in mission_entries]
    range_run = load_run(suite_path, range_entries[0])
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".cache/matplotlib"))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as error:
        raise SystemExit("Optional plotting dependencies missing. Install requirements-plots.txt in a separate environment.") from error
    args.output.mkdir(parents=True, exist_ok=True)
    style = {"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False,
             "axes.spines.right": False, "axes.edgecolor": "#94a3b8", "axes.labelcolor": "#334155",
             "axes.grid": True, "grid.color": "#dce2e8", "grid.linewidth": .7, "axes.axisbelow": True,
             "figure.facecolor": "white", "savefig.facecolor": "white", "svg.fonttype": "none",
             "svg.hashsalt": "drone-sitl-prototype-01"}
    with plt.rc_context(style):
        for stem, figure in [("mission-trajectories", plot_mission(plt, missions)),
                             ("rangefinder-response", plot_rangefinder(plt, range_run))]:
            fig, provenance = figure
            save_figure(fig, args.output, stem, provenance, suite_path, suite)
            plt.close(fig)
            print(f"Wrote {stem}.svg and {stem}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
