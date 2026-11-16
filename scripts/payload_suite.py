#!/usr/bin/env python3
"""Run every declared G06 payload case, retaining failures and repetitions."""

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
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--payloads', nargs='+', type=int, choices=(0, 1, 3, 5), default=[0, 1, 3, 5])
    parser.add_argument('--speedup', type=float, default=5)
    parser.add_argument('--instance', type=int, default=30)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    output = args.output or ROOT / 'runs' / ('g06-payload-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = ROOT / 'config/g06/manifest.json'
    suite = {
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'manifest_sha256': hashlib.sha256(manifest.read_bytes()).hexdigest(),
        'repeats': args.repeats,
        'payloads_l': args.payloads,
        'runs': [],
        'result': 'incomplete',
    }
    suite_path = output / 'suite.json'
    suite_path.write_text(json.dumps(suite, indent=2) + '\n')
    for payload in args.payloads:
        for repeat in range(1, args.repeats + 1):
            label = f'payload-{payload}l-repeat-{repeat}'
            run_dir = output / label
            print(f'\nRunning {label}', flush=True)
            command = [
                sys.executable, str(ROOT / 'scripts/payload_test.py'),
                '--model', str(ROOT / f'config/g06/models/payload-{payload}l.json'),
                '--params', str(ROOT / 'config/g06/defaults.parm'),
                '--label', label, '--output', str(run_dir),
                '--instance', str(args.instance), '--speedup', str(args.speedup),
            ]
            process = subprocess.run(command, cwd=ROOT)
            summary_path = run_dir / 'summary.json'
            result = json.loads(summary_path.read_text()) if summary_path.exists() else {'result': 'failed', 'error': 'No summary produced'}
            suite['runs'].append({
                'label': label, 'payload_l': payload, 'repeat': repeat,
                'directory': label, 'exit_code': process.returncode,
                'result': result.get('result', 'failed'),
                'error': result.get('error'),
                'summary_sha256': hashlib.sha256(summary_path.read_bytes()).hexdigest() if summary_path.exists() else None,
            })
            suite_path.write_text(json.dumps(suite, indent=2) + '\n')
    passed = sum(row['result'] == 'passed' and row['exit_code'] == 0 for row in suite['runs'])
    suite['passed_count'] = passed
    suite['run_count'] = len(suite['runs'])
    suite['result'] = 'passed' if passed == len(suite['runs']) else 'failed'
    suite['finished_utc'] = datetime.now(timezone.utc).isoformat()
    suite_path.write_text(json.dumps(suite, indent=2) + '\n')
    print(f"Suite {suite['result']}: {passed}/{len(suite['runs'])}; {suite_path}", flush=True)
    return 0 if suite['result'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
