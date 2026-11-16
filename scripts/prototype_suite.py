#!/usr/bin/env python3
"""Run prototype nominal scenarios and explicit mass sensitivity cases, retaining all outcomes."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenarios", nargs="+", choices=("flight", "mission", "rangefinder"),
                        default=["flight", "mission", "rangefinder"])
    parser.add_argument("--no-mass-sensitivity", action="store_true")
    parser.add_argument("--sensitivity-repeats", type=int, default=1)
    parser.add_argument("--speedup", type=float, default=5)
    parser.add_argument("--instance", type=int, default=40)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.repeats < 1 or args.sensitivity_repeats < 1:
        parser.error("Repetitions must be positive")
    output = args.output or ROOT / "runs" / ("prototype-01-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = ROOT / "config/prototype-01/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    cases = {c["label"]: c for c in manifest["cases"]}
    matrix = [("nominal", scenario, repeat) for scenario in args.scenarios for repeat in range(1, args.repeats + 1)]
    if not args.no_mass_sensitivity:
        matrix += [(case, "flight", repeat) for case in ("mass-low", "mass-high")
                   for repeat in range(1, args.sensitivity_repeats + 1)]
    suite = {"started_utc": datetime.now(timezone.utc).isoformat(), "manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
             "nominal_repeats": args.repeats, "sensitivity_repeats": args.sensitivity_repeats,
             "scenarios": args.scenarios, "mass_sensitivity": not args.no_mass_sensitivity,
             "runs": [], "result": "incomplete"}
    suite_path = output / "suite.json"
    suite_path.write_text(json.dumps(suite, indent=2) + "\n")
    for case, scenario, repeat in matrix:
        label = f"{case}-{scenario}-repeat-{repeat}"
        print(f"\nRunning {label}", flush=True)
        command = [sys.executable, str(ROOT / "scripts/prototype_test.py"), "--scenario", scenario,
                   "--model", str(manifest_path.parent / cases[case]["model"]), "--label", label,
                   "--output", str(output / label), "--instance", str(args.instance), "--speedup", str(args.speedup)]
        process = subprocess.run(command, cwd=ROOT)
        summary_path = output / label / "summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {"result": "failed", "error": "No summary produced"}
        suite["runs"].append({"label": label, "case": case, "scenario": scenario, "repeat": repeat,
            "directory": label, "exit_code": process.returncode, "result": summary.get("result", "failed"),
            "error": summary.get("error"),
            "summary_sha256": hashlib.sha256(summary_path.read_bytes()).hexdigest() if summary_path.exists() else None})
        suite_path.write_text(json.dumps(suite, indent=2) + "\n")
    passed = sum(r["result"] == "passed" and r["exit_code"] == 0 for r in suite["runs"])
    suite.update(passed_count=passed, run_count=len(suite["runs"]),
                 result="passed" if passed == len(suite["runs"]) else "failed",
                 finished_utc=datetime.now(timezone.utc).isoformat())
    suite_path.write_text(json.dumps(suite, indent=2) + "\n")
    print(f"Suite {suite['result']}: {passed}/{len(suite['runs'])}; {suite_path}", flush=True)
    return 0 if suite["result"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
