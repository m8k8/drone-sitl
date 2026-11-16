#!/usr/bin/env python3
"""Print a Markdown table from an existing payload-suite run, without changing it."""

import argparse
import json
from pathlib import Path
import statistics


def fmt(value, digits=3):
    return '未取得' if value is None else f'{value:.{digits}f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    args = parser.parse_args()
    folder = args.suite.resolve()
    suite = json.loads((folder / 'suite.json').read_text())
    rows = []
    for entry in suite['runs']:
        location = folder / entry['directory']
        meta = json.loads((location / 'run.json').read_text()) if (location / 'run.json').exists() else {}
        summary = json.loads((location / 'summary.json').read_text()) if (location / 'summary.json').exists() else {}
        rows.append((entry, meta, summary))
    print('| 水量 | 総質量 kg | 回 | 判定 | 離陸点上空の高度範囲 m | 同・平均PWM µs | 北20mの位置ずれ最大 m（真値） | 着地ずれ m（真値） |')
    print('| --- | ---: | ---: | --- | --- | ---: | ---: | ---: |')
    for entry, meta, summary in rows:
        metrics = summary.get('metrics', {})
        hover = metrics.get('hover_home', {})
        north = metrics.get('hover_north', {})
        landing = metrics.get('landing', {})
        status = '成功' if entry.get('result') == 'passed' and entry.get('exit_code') == 0 else '失敗'
        alt = f"{fmt(hover.get('altitude_min_m'))}–{fmt(hover.get('altitude_max_m'))}"
        print(f"| {entry['payload_l']}L | {fmt(meta.get('actual_mass_kg'),4)} | {entry['repeat']} | {status} | {alt} | {fmt(hover.get('servo_pwm_mean'),1)} | {fmt(north.get('truth_horizontal_error_max_m'))} | {fmt(landing.get('truth_home_error_m'))} |")
    print('\n| 水量 | 成功 / 実施 | 平均PWM µs（離陸点ホバー） | 平均電流 A（モデル参考値） | 平均電圧 V（モデル参考値） |')
    print('| --- | ---: | ---: | ---: | ---: |')
    for payload in suite['payloads_l']:
        group = [(entry, summary) for entry, _meta, summary in rows if entry['payload_l'] == payload]
        successful = sum(entry.get('result') == 'passed' and entry.get('exit_code') == 0 for entry, _ in group)
        def average(field):
            values = [summary.get('metrics', {}).get('hover_home', {}).get(field) for _, summary in group]
            values = [value for value in values if value is not None]
            return statistics.fmean(values) if values else None
        print(f"| {payload}L | {successful}/{len(group)} | {fmt(average('servo_pwm_mean'),1)} | {fmt(average('battery_current_mean_a'),2)} | {fmt(average('battery_voltage_mean_v'),2)} |")
    failures = [(entry, summary) for entry, _, summary in rows if entry.get('result') != 'passed' or entry.get('exit_code') != 0]
    if failures:
        print('\n失敗理由：')
        for entry, summary in failures:
            print(f"- {entry['label']}: {summary.get('error', entry.get('error', '不明'))}")


if __name__ == '__main__':
    main()
