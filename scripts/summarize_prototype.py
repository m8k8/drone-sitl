#!/usr/bin/env python3
"""Print Markdown measurements from an existing prototype suite; never modify results."""

import argparse
import json
from pathlib import Path


def fmt(value, digits=3):
    return "未取得" if value is None else f"{value:.{digits}f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    args = parser.parse_args()
    folder = args.suite.resolve()
    suite = json.loads((folder / "suite.json").read_text())
    records = []
    print("| ケース | 試験 | 回 | 判定 | 質量 kg | 真値着地ずれ m | 飛行時間 s |")
    print("| --- | --- | ---: | --- | ---: | ---: | ---: |")
    for entry in suite["runs"]:
        path = folder / entry["directory"]
        summary = json.loads((path / "summary.json").read_text()) if (path / "summary.json").exists() else {}
        meta = json.loads((path / "run.json").read_text()) if (path / "run.json").exists() else {}
        records.append((entry, meta, summary))
        metrics = summary.get("metrics", {})
        status = "成功" if entry["result"] == "passed" and entry["exit_code"] == 0 else "失敗"
        print(f"| {entry['case']} | {entry['scenario']} | {entry['repeat']} | {status} | {fmt(meta.get('actual_mass_kg'),4)} | {fmt(metrics.get('landing',{}).get('truth_home_error_m'))} | {fmt(metrics.get('flight',{}).get('armed_sim_seconds'),1)} |")
    print("\n| ケース | 回 | 高度範囲 m（離陸点ホバー） | 平均PWM µs | 平均電流 A（参考） | 北20m真値ずれ最大 m |")
    print("| --- | ---: | --- | ---: | ---: | ---: |")
    for entry, _meta, summary in records:
        if entry["scenario"] != "flight":
            continue
        hover = summary.get("metrics", {}).get("hover_home", {})
        north = summary.get("metrics", {}).get("hover_north", {})
        alt = fmt(hover.get("altitude_min_m")) + "–" + fmt(hover.get("altitude_max_m"))
        print(f"| {entry['case']} | {entry['repeat']} | {alt} | {fmt(hover.get('servo_pwm_mean'),1)} | {fmt(hover.get('battery_current_mean_a'),2)} | {fmt(north.get('truth_horizontal_error_max_m'))} |")
    print("\n| 測距試験の回 | +1m注入時の推定／真値高度変化 m | 0復帰時の真値高度差 m | 範囲外中の真値高度範囲 m | 復旧後真値高度差 m | コントローラログ証拠 |")
    print("| ---: | --- | ---: | --- | ---: | --- |")
    for entry, _meta, summary in records:
        if entry["scenario"] != "rangefinder":
            continue
        ranges = summary.get("metrics", {}).get("rangefinder", {})
        fault = ranges.get("range_out_of_range_hold", {})
        alt = fmt(fault.get("true_altitude_min_m")) + "–" + fmt(fault.get("true_altitude_max_m"))
        response = ranges.get("range_bias_response", {})
        delta = fmt(response.get("altitude_change_m")) + " / " + fmt(response.get("true_altitude_change_m"))
        print(f"| {entry['repeat']} | {delta} | {fmt(ranges.get('range_bias_restored',{}).get('true_altitude_change_m'))} | {alt} | {fmt(ranges.get('range_recovered_hold',{}).get('true_altitude_change_m'))} | {'記録あり' if summary.get('range_log_evidence') else '未取得'} |")
    print("\n| AUTO試験の回 | WP1真値接近 m | WP2 m | WP3 m | WP4 m |")
    print("| ---: | ---: | ---: | ---: | ---: |")
    for entry, _meta, summary in records:
        if entry["scenario"] != "mission":
            continue
        closest = summary.get("metrics", {}).get("mission", {}).get("waypoint_closest", {})
        values = " | ".join(fmt(closest.get(str(i), {}).get("truth_error_m")) for i in range(1, 5))
        print(f"| {entry['repeat']} | {values} |")
    failed = [(e, s) for e, _, s in records if e["result"] != "passed" or e["exit_code"] != 0]
    if failed:
        print("\n失敗理由：")
        for entry, summary in failed:
            print(f"- {entry['label']}: {summary.get('error', entry.get('error', '不明'))}")


if __name__ == "__main__":
    main()
