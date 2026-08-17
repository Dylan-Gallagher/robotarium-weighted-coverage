#!/usr/bin/env python3
"""Verify committed simulator evidence without trusting repository metadata."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "simulator-3072698"
EXPECTED_SIMULATOR = "307269846b2761528586e9c3f47d0a8bec21692f"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_manifest(path: Path) -> dict[str, str]:
    entries: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        digest, filename = line.split(maxsplit=1)
        entries[filename.strip()] = digest
    return entries


def main() -> None:
    manifest = load_manifest(EVIDENCE / "MANIFEST.sha256")
    for filename, expected in manifest.items():
        actual = sha256(EVIDENCE / filename)
        if actual != expected:
            raise SystemExit(f"hash mismatch for {filename}: {actual} != {expected}")

    summary = json.loads(
        (EVIDENCE / "robotarium_coverage_summary.json").read_text(encoding="utf-8")
    )
    assert summary["evidence_scope"] == (
        "local Robotarium simulator only; no physical hardware run claimed"
    )
    assert summary["official_simulator_commit"] == EXPECTED_SIMULATOR
    assert summary["config"]["seed"] == 20260817
    assert summary["config"]["robots"] == 8
    assert summary["config"]["iterations"] == 3600
    assert summary["experiment_phase_validation_available"] is True
    assert summary["experiment_phase_hard_error_count"] == 0
    assert summary["coverage_cost_reduction_fraction"] > 0.37
    assert summary["minimum_simulator_collision_center_pair_distance_m"] > 0.15
    assert summary["minimum_conservative_body_boundary_clearance_m"] > 0.0

    trajectory = np.load(
        EVIDENCE / "robotarium_coverage_trajectory.npy", allow_pickle=False
    )
    assert trajectory.shape == (361, 2, 8)
    assert np.isfinite(trajectory).all()

    with (EVIDENCE / "robotarium_coverage_metrics.csv").open(
        newline="", encoding="utf-8"
    ) as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 361
    assert float(rows[-1]["coverage_cost_m2"]) < float(rows[0]["coverage_cost_m2"])

    simulator_log = (EVIDENCE / "simulator.log").read_text(encoding="utf-8")
    assert "No errors or warnings in your simulation" in simulator_log
    assert "local Robotarium simulator only" in simulator_log
    print(f"verified {len(manifest)} frozen files and simulator-only scope")


if __name__ == "__main__":
    main()
