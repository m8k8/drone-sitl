#!/usr/bin/env python3
"""Apply only the known model extension to exactly the pinned upstream files."""
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
UPSTREAM = HERE.parents[1] / "vendor/ardupilot"
manifest = json.loads((HERE/"manifest.json").read_text())
def git(*args):
    return subprocess.check_output(["git","-C",str(UPSTREAM),*args],text=True).strip()
if git("rev-parse","HEAD") != manifest["upstream_commit"]:
    raise SystemExit("Refusing unexpected ArduPilot commit")
if git("diff","--cached","--name-only"):
    raise SystemExit("Refusing staged upstream changes")
changed = set(filter(None,git("diff","--name-only").splitlines()))
if not changed.issubset(manifest["patch_files"]):
    raise SystemExit(f"Refusing unrelated upstream edits: {sorted(changed)}")
states = []
for relative, hashes in manifest["patch_files"].items():
    digest = hashlib.sha256((UPSTREAM/relative).read_bytes()).hexdigest()
    if digest == hashes["original_sha256"]:
        states.append("original")
    elif digest == hashes["patched_sha256"]:
        states.append("patched")
    else:
        raise SystemExit(f"Refusing unexpected upstream edit: {relative}")
if len(set(states)) != 1:
    raise SystemExit("Refusing partially applied patch")
patch = HERE/"reference-mass.patch"
if hashlib.sha256(patch.read_bytes()).hexdigest() != manifest["patch_sha256"]:
    raise SystemExit("Patch checksum mismatch")
if states[0] == "original":
    subprocess.run(["git","-C",str(UPSTREAM),"apply","--check",str(patch)],check=True)
    subprocess.run(["git","-C",str(UPSTREAM),"apply",str(patch)],check=True)
    print("Applied referenceMass model extension")
else:
    print("Known referenceMass model extension already applied")
for relative, hashes in manifest["patch_files"].items():
    assert hashlib.sha256((UPSTREAM/relative).read_bytes()).hexdigest() == hashes["patched_sha256"]
