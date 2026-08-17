#!/usr/bin/env python3
"""Compare a fresh simulator run with the frozen deterministic evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


FLOAT_PATHS = (
    ("nominal_control_duration_s",),
    ("coverage_cost_reduction_fraction",),
    ("minimum_simulator_collision_center_pair_distance_m",),
    ("minimum_reference_point_boundary_clearance_m",),
    ("minimum_conservative_body_boundary_clearance_m",),
    ("barrier_intervention_step_fraction",),
    ("global_actuator_scaling_step_fraction",),
    ("maximum_nominal_si_speed_mps",),
    ("maximum_safe_linear_speed_mps",),
    ("maximum_safe_angular_speed_radps",),
    ("sustained_centroid_residual_max_below_0_05m_at_s",),
    *(("initial", key) for key in (
        "coverage_cost_m2",
        "weighted_fraction_within_0_25m",
        "weighted_fraction_within_0_35m",
        "weighted_fraction_within_0_50m",
        "weighted_rms_distance_m",
    )),
    *(("final", key) for key in (
        "coverage_cost_m2",
        "weighted_fraction_within_0_25m",
        "weighted_fraction_within_0_35m",
        "weighted_fraction_within_0_50m",
        "weighted_rms_distance_m",
    )),
)


def nested(value: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = value
    for key in path:
        current = current[key]
    return current


def main() -> None:
    if len(sys.argv) != 5:
        raise SystemExit(
            "usage: compare_reproduction.py FROZEN_SUMMARY NEW_SUMMARY "
            "FROZEN_TRAJECTORY NEW_TRAJECTORY"
        )
    frozen_summary_path, new_summary_path, frozen_traj_path, new_traj_path = (
        Path(argument) for argument in sys.argv[1:]
    )
    frozen = json.loads(frozen_summary_path.read_text(encoding="utf-8"))
    reproduced = json.loads(new_summary_path.read_text(encoding="utf-8"))

    for key in (
        "evidence_scope",
        "official_simulator_commit",
        "experiment_phase_validation_available",
        "experiment_phase_validation_errors",
        "experiment_phase_hard_error_count",
    ):
        if reproduced[key] != frozen[key]:
            raise SystemExit(f"summary mismatch at {key}: {reproduced[key]!r}")

    for path in FLOAT_PATHS:
        expected = float(nested(frozen, path))
        actual = float(nested(reproduced, path))
        if not np.isclose(actual, expected, rtol=1e-10, atol=1e-12):
            dotted = ".".join(path)
            raise SystemExit(f"numeric mismatch at {dotted}: {actual} != {expected}")

    frozen_traj = np.load(frozen_traj_path, allow_pickle=False)
    new_traj = np.load(new_traj_path, allow_pickle=False)
    if frozen_traj.shape != new_traj.shape:
        raise SystemExit(
            f"trajectory shape mismatch: {new_traj.shape} != {frozen_traj.shape}"
        )
    np.testing.assert_allclose(new_traj, frozen_traj, rtol=0.0, atol=1e-12)
    print("reproduction matches stable summary fields and the sampled trajectory")


if __name__ == "__main__":
    main()
