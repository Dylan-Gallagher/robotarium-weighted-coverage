#!/usr/bin/env python3
"""Apply the documented cross-environment simulator reproduction contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


# These tolerances were introduced after the first hosted run (GitHub Actions
# 31993185231) exposed a cross-environment delta, then checked against a clean
# local run using the exact public direct pins. That public-pin local run
# matched the frozen trajectory byte for byte; the hosted run used the same
# direct pins but differed slightly. Transitive resolution, binary artifacts,
# and system numerical libraries are not locked, and the historical frozen
# environment is only partially recorded. This code therefore does not assign
# a sole numerical cause. README.md and PROVENANCE.md document the audit.
TRAJECTORY_MAX_ABS_TOLERANCE_M = 5e-5
TRAJECTORY_RMS_TOLERANCE_M = 5e-6

STRICT_CONFIG_FIELDS = (
    "robots",
    "iterations",
    "time_step_s",
    "grid_nx",
    "grid_ny",
    "control_gain",
    "nominal_si_speed_limit_mps",
    "barrier_velocity_limit_mps",
    "maximum_wheel_speed_radps",
    "maximum_commanded_angular_speed_radps",
    "barrier_safety_radius_m",
    "barrier_gain",
    "barrier_projection_distance_m",
    "log_stride",
    "seed",
    "skip_initialization",
)

# Path, absolute tolerance. Relative tolerance is deliberately zero so every
# accepted physical-unit envelope can be read directly from this file.
STABLE_NUMERIC_PATHS = (
    (("nominal_control_duration_s",), 1e-12),
    (("coverage_cost_reduction_fraction",), 1e-9),
    (("minimum_reference_point_boundary_clearance_m",), 5e-5),
    (("minimum_conservative_body_boundary_clearance_m",), 5e-5),
    (("maximum_nominal_si_speed_mps",), 1e-12),
    (("maximum_safe_linear_speed_mps",), 1e-9),
    (("maximum_safe_angular_speed_radps",), 1e-5),
    (("sustained_centroid_residual_max_below_0_05m_at_s",), 0.33),
    *((("initial", key), 1e-9) for key in (
        "coverage_cost_m2",
        "weighted_fraction_within_0_25m",
        "weighted_fraction_within_0_35m",
        "weighted_fraction_within_0_50m",
        "weighted_rms_distance_m",
    )),
    *((("final", key), 1e-9) for key in (
        "coverage_cost_m2",
        "weighted_fraction_within_0_25m",
        "weighted_fraction_within_0_35m",
        "weighted_fraction_within_0_50m",
        "weighted_rms_distance_m",
    )),
)

SPACING_ABS_TOLERANCE_M = 0.01


def nested(value: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = value
    for key in path:
        current = current[key]
    return current


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def compare_reproduction(
    frozen: dict[str, Any],
    reproduced: dict[str, Any],
    frozen_trajectory: np.ndarray,
    reproduced_trajectory: np.ndarray,
) -> dict[str, float]:
    """Validate one reproduction and return its cross-environment delta report."""

    for key in ("evidence_scope", "official_simulator_commit"):
        _require(
            reproduced.get(key) == frozen.get(key),
            f"summary mismatch at {key}: {reproduced.get(key)!r} != {frozen.get(key)!r}",
        )

    for key in (
        "published_recommended_spacing_m",
        "simulator_collision_center_offset_m",
        "conservative_body_disk_radius_m",
    ):
        _require(
            reproduced.get(key) == frozen.get(key),
            f"geometry invariant mismatch at {key}: "
            f"{reproduced.get(key)!r} != {frozen.get(key)!r}",
        )

    for key in STRICT_CONFIG_FIELDS:
        expected = frozen["config"][key]
        actual = reproduced["config"][key]
        _require(
            actual == expected,
            f"controller configuration mismatch at config.{key}: {actual!r} != {expected!r}",
        )

    for prefix, label in (
        ("initialization", "initialization"),
        ("simulator", "whole-run simulator"),
        ("experiment_phase", "experiment-phase simulator"),
    ):
        _require(
            reproduced.get(f"{prefix}_validation_available") is True,
            f"{label} validation must be available",
        )
        _require(
            reproduced.get(f"{prefix}_validation_errors") == {},
            f"{label} validation must report zero errors or warnings",
        )
    for key, label in (
        ("simulator_hard_error_count", "whole-run simulator"),
        ("experiment_phase_hard_error_count", "experiment-phase simulator"),
    ):
        _require(
            reproduced.get(key) == 0,
            f"{label} hard-error count must be zero",
        )

    for path, tolerance in STABLE_NUMERIC_PATHS:
        expected = float(nested(frozen, path))
        actual = float(nested(reproduced, path))
        _require(np.isfinite(actual), f"non-finite value at {'.'.join(path)}")
        _require(
            abs(actual - expected) <= tolerance,
            f"numeric mismatch at {'.'.join(path)}: {actual} != {expected} "
            f"(absolute tolerance {tolerance})",
        )

    spacing = float(reproduced["minimum_simulator_collision_center_pair_distance_m"])
    frozen_spacing = float(frozen["minimum_simulator_collision_center_pair_distance_m"])
    published_spacing = float(reproduced["published_recommended_spacing_m"])
    _require(np.isfinite(spacing), "minimum collision-center spacing must be finite")
    _require(
        spacing >= published_spacing,
        f"minimum collision-center spacing {spacing} m is below the published "
        f"{published_spacing} m recommendation",
    )
    _require(
        abs(spacing - frozen_spacing) <= SPACING_ABS_TOLERANCE_M,
        f"minimum collision-center spacing drifted by {spacing - frozen_spacing} m "
        f"(absolute tolerance {SPACING_ABS_TOLERANCE_M} m)",
    )

    conservative_clearance = float(
        reproduced["minimum_conservative_body_boundary_clearance_m"]
    )
    reference_clearance = float(
        reproduced["minimum_reference_point_boundary_clearance_m"]
    )
    _require(
        conservative_clearance > 0.0,
        "conservative body boundary clearance must be positive",
    )
    _require(
        reference_clearance > 0.0,
        "reference-point boundary clearance must be positive",
    )

    config = reproduced["config"]
    nominal_speed = float(reproduced["maximum_nominal_si_speed_mps"])
    safe_linear_speed = float(reproduced["maximum_safe_linear_speed_mps"])
    safe_angular_speed = float(reproduced["maximum_safe_angular_speed_radps"])
    _require(
        nominal_speed <= float(config["nominal_si_speed_limit_mps"]) + 1e-12,
        "maximum nominal SI speed exceeds its configured limit",
    )
    _require(
        safe_linear_speed <= float(config["barrier_velocity_limit_mps"]) + 1e-12,
        "maximum safe linear speed exceeds the configured barrier velocity limit",
    )
    _require(
        safe_angular_speed
        <= float(config["maximum_commanded_angular_speed_radps"]) + 1e-12,
        "maximum safe angular speed exceeds its configured actuator guard",
    )

    for key in (
        "barrier_intervention_step_fraction",
        "global_actuator_scaling_step_fraction",
    ):
        value = float(reproduced[key])
        _require(
            np.isfinite(value) and 0.0 <= value <= 1.0,
            f"invalid diagnostic {key}",
        )

    _require(
        reproduced_trajectory.shape == frozen_trajectory.shape,
        f"trajectory shape mismatch: {reproduced_trajectory.shape} != {frozen_trajectory.shape}",
    )
    _require(frozen_trajectory.size > 0, "frozen trajectory must not be empty")
    _require(
        np.isfinite(frozen_trajectory).all(),
        "frozen trajectory contains non-finite values",
    )
    _require(
        np.isfinite(reproduced_trajectory).all(),
        "reproduced trajectory contains non-finite values",
    )
    trajectory_delta = reproduced_trajectory - frozen_trajectory
    trajectory_max = float(np.max(np.abs(trajectory_delta)))
    trajectory_rms = float(np.sqrt(np.mean(np.square(trajectory_delta))))
    _require(
        trajectory_max <= TRAJECTORY_MAX_ABS_TOLERANCE_M,
        f"trajectory maximum absolute delta {trajectory_max} m exceeds "
        f"{TRAJECTORY_MAX_ABS_TOLERANCE_M} m",
    )
    _require(
        trajectory_rms <= TRAJECTORY_RMS_TOLERANCE_M,
        f"trajectory RMS delta {trajectory_rms} m exceeds "
        f"{TRAJECTORY_RMS_TOLERANCE_M} m",
    )

    return {
        "trajectory_max_abs_delta_m": trajectory_max,
        "trajectory_rms_delta_m": trajectory_rms,
        "minimum_collision_center_spacing_delta_m": spacing - frozen_spacing,
        "coverage_cost_reduction_delta": float(
            reproduced["coverage_cost_reduction_fraction"]
            - frozen["coverage_cost_reduction_fraction"]
        ),
        "barrier_intervention_fraction_delta": float(
            reproduced["barrier_intervention_step_fraction"]
            - frozen["barrier_intervention_step_fraction"]
        ),
        "actuator_scaling_fraction_delta": float(
            reproduced["global_actuator_scaling_step_fraction"]
            - frozen["global_actuator_scaling_step_fraction"]
        ),
    }


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
    frozen_trajectory = np.load(frozen_traj_path, allow_pickle=False)
    reproduced_trajectory = np.load(new_traj_path, allow_pickle=False)

    try:
        report = compare_reproduction(
            frozen, reproduced, frozen_trajectory, reproduced_trajectory
        )
    except (KeyError, TypeError, ValueError) as error:
        raise SystemExit(str(error)) from error

    print("reproduction satisfies the documented cross-environment semantic contract")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
