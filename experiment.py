"""Density-weighted multi-robot coverage for the Georgia Tech Robotarium.

The default entry point is intentionally hardware-portable: it uses only NumPy,
Matplotlib, CVXOPT (through Robotarium's supplied barrier certificate), and the
official ``rps`` API.  All evidence produced locally is simulation-only until an
approved Robotarium account executes this file on the physical testbed.
"""

from __future__ import annotations

import csv
import json
import os
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

import rps.robotarium as robotarium
from rps.utilities.barrier_certificates import create_uni_barrier_certificate_with_boundary
from rps.utilities.transformations import create_si_to_uni_dynamics


# Robotarium's published physical arena is [-1.6, 1.6] x [-1.0, 1.0] m.
# This smaller rectangle leaves 18 cm of x-margin and 18 cm of y-margin.
PHYSICAL_BOUNDS = np.array([-1.6, 1.6, -1.0, 1.0], dtype=float)
OPERATING_BOUNDS = np.array([-1.42, 1.42, -0.82, 0.82], dtype=float)
ROBOT_RADIUS_M = 0.055
SIMULATOR_COLLISION_CENTER_OFFSET_M = 0.025
INITIAL_CONDITIONS = np.array(
    [
        [-1.15, -0.38, 0.38, 1.15, -1.15, -0.38, 0.38, 1.15],
        [0.62, 0.62, 0.62, 0.62, -0.62, -0.62, -0.62, -0.62],
        [-0.45, -1.10, -2.05, -2.70, 0.45, 1.10, 2.05, 2.70],
    ],
    dtype=float,
)


@dataclass(frozen=True)
class ExperimentConfig:
    robots: int = 8
    iterations: int = 3600
    time_step_s: float = 0.033
    grid_nx: int = 45
    grid_ny: int = 27
    control_gain: float = 0.85
    nominal_si_speed_limit_mps: float = 0.11
    barrier_velocity_limit_mps: float = 0.14
    maximum_wheel_speed_radps: float = 12.0
    maximum_commanded_angular_speed_radps: float = 3.5
    barrier_safety_radius_m: float = 0.15
    barrier_gain: float = 150.0
    barrier_projection_distance_m: float = 0.03
    log_stride: int = 10
    seed: int = 20260817
    show_figure: bool = False
    sim_in_real_time: bool = False
    skip_initialization: bool = False
    save_plot: bool = True
    output_dir: str = "."

    @property
    def nominal_duration_s(self) -> float:
        return self.iterations * self.time_step_s

    def validate(self) -> None:
        if self.robots != INITIAL_CONDITIONS.shape[1]:
            raise ValueError("robots must match the eight-column initial-condition matrix")
        if not 1 <= self.robots <= 20:
            raise ValueError("hardware experiments must use between 1 and 20 robots")
        if self.iterations <= 0:
            raise ValueError("iterations must be positive")
        if self.nominal_duration_s > 900.0:
            raise ValueError("nominal experiment duration exceeds the published 900 s limit")
        if self.barrier_safety_radius_m < 0.15:
            raise ValueError("safety radius must respect the published 15 cm recommendation")
        if self.nominal_si_speed_limit_mps > 0.2:
            raise ValueError("nominal speed exceeds the published 0.2 m/s platform maximum")
        if self.maximum_wheel_speed_radps > 12.5:
            raise ValueError("wheel-speed guard exceeds the published 12.5 rad/s motor limit")
        if self.maximum_commanded_angular_speed_radps > 3.6:
            raise ValueError("angular-speed guard exceeds the published about-3.6 rad/s maximum")
        if self.grid_nx < 3 or self.grid_ny < 3:
            raise ValueError("coverage grid is too small")


def make_quadrature_grid(
    bounds: np.ndarray = OPERATING_BOUNDS,
    nx: int = 45,
    ny: int = 27,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return cell-centre samples, normalized density weights, and raw density."""

    xs = np.linspace(bounds[0], bounds[1], nx, endpoint=False)
    ys = np.linspace(bounds[2], bounds[3], ny, endpoint=False)
    xs += (bounds[1] - bounds[0]) / (2.0 * nx)
    ys += (bounds[3] - bounds[2]) / (2.0 * ny)
    xx, yy = np.meshgrid(xs, ys)
    points = np.vstack((xx.ravel(), yy.ravel()))
    density = workload_density(points)
    weights = density / density.sum()
    return points, weights, density


def workload_density(points: np.ndarray) -> np.ndarray:
    """Static, non-uniform workload used by the coverage objective.

    The broad uniform floor prevents agents being drawn into only the Gaussian
    peaks.  The three unequal peaks make this materially different from the
    simulator's canonical formation and go-to-goal examples.
    """

    if points.shape[0] != 2:
        raise ValueError("points must have shape 2 x M")
    x, y = points
    density = np.full(points.shape[1], 0.18, dtype=float)
    components = (
        (-0.92, 0.43, 0.31, 0.24, 1.15),
        (0.02, -0.50, 0.36, 0.22, 0.95),
        (0.91, 0.31, 0.29, 0.30, 1.08),
        (0.10, 0.47, 0.58, 0.16, 0.42),
    )
    for mux, muy, sigx, sigy, amplitude in components:
        exponent = -0.5 * (((x - mux) / sigx) ** 2 + ((y - muy) / sigy) ** 2)
        density += amplitude * np.exp(exponent)
    return density


def discrete_voronoi(
    positions: np.ndarray,
    points: np.ndarray,
    weights: np.ndarray,
    grid_shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute deterministic weighted Lloyd targets and a grid adjacency graph.

    ``np.argmin`` gives deterministic lowest-index tie-breaking.  Each robot's
    target is the density-weighted centroid of its own Voronoi cell.  The
    nominal feedback is therefore distributed-by-agent; this vectorized
    implementation uses the global poses supplied by ``r.get_poses()``.
    """

    if positions.ndim != 2 or positions.shape[0] != 2:
        raise ValueError("positions must have shape 2 x N")
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must have shape 2 x M")
    if weights.shape != (points.shape[1],):
        raise ValueError("weights must have one entry per point")
    if np.any(weights < 0) or not np.isfinite(weights).all():
        raise ValueError("weights must be finite and non-negative")

    delta = positions[:, :, None] - points[:, None, :]
    squared_distances = np.sum(delta * delta, axis=0)
    owners = np.argmin(squared_distances, axis=0)
    n = positions.shape[1]
    centroids = positions.copy()
    cell_mass = np.zeros(n, dtype=float)
    for robot_id in range(n):
        in_cell = owners == robot_id
        cell_mass[robot_id] = float(weights[in_cell].sum())
        if cell_mass[robot_id] > 0.0:
            centroids[:, robot_id] = (
                points[:, in_cell] @ weights[in_cell] / cell_mass[robot_id]
            )

    labels = owners.reshape(grid_shape)
    adjacency = np.zeros((n, n), dtype=bool)
    for lhs, rhs in (
        (labels[:, :-1], labels[:, 1:]),
        (labels[:-1, :], labels[1:, :]),
    ):
        boundary = lhs != rhs
        for i, j in zip(lhs[boundary], rhs[boundary], strict=True):
            adjacency[i, j] = True
            adjacency[j, i] = True
    return centroids, owners, cell_mass, adjacency


def clip_columns(vectors: np.ndarray, limit: float) -> np.ndarray:
    result = vectors.copy()
    norms = np.linalg.norm(result, axis=0)
    mask = norms > limit
    if np.any(mask):
        result[:, mask] *= limit / norms[mask]
    return result


def globally_limit_unicycle_wheels(
    commands: np.ndarray,
    wheel_radius_m: float = 0.016,
    axle_length_m: float = 0.11,
    maximum_wheel_speed_radps: float = 12.0,
    maximum_angular_speed_radps: float = 3.5,
) -> tuple[np.ndarray, float]:
    """Apply a conservative common actuator scale to all commands.

    Robotarium ultimately clips wheel speeds.  A single global scale factor is
    used here instead of per-robot clipping: for the safe set (where barrier
    right-hand sides are non-negative), common scaling preserves the linear
    barrier inequalities.  The defaults leave margin below both the published
    12.5 rad/s wheel threshold and the approximately 3.6 rad/s body-rotation
    maximum.
    """

    wheel_speeds = np.vstack(
        (
            (2.0 * commands[0] - axle_length_m * commands[1]) / (2.0 * wheel_radius_m),
            (2.0 * commands[0] + axle_length_m * commands[1]) / (2.0 * wheel_radius_m),
        )
    )
    wheel_peak = float(np.max(np.abs(wheel_speeds))) if wheel_speeds.size else 0.0
    angular_peak = float(np.max(np.abs(commands[1]))) if commands.size else 0.0
    wheel_scale = (
        min(1.0, maximum_wheel_speed_radps / wheel_peak)
        if wheel_peak > 0.0
        else 1.0
    )
    angular_scale = (
        min(1.0, maximum_angular_speed_radps / angular_peak)
        if angular_peak > 0.0
        else 1.0
    )
    scale = min(wheel_scale, angular_scale)
    return commands * scale, scale


def simulator_validation_snapshot(instance: Any) -> tuple[bool, dict[str, int]]:
    """Read optional local-simulator counters without requiring them on hardware.

    ``_errors`` is an implementation detail of the local Python simulator, not
    part of the public Robotarium experiment API.  Keeping the guarded access
    in this helper makes the uploaded script portable to runtimes that omit it;
    those runs report validation as unavailable rather than claiming zero.
    """

    raw_errors = getattr(instance, "_errors", None)
    if not isinstance(raw_errors, dict):
        return False, {}
    try:
        return True, {str(key): int(value) for key, value in raw_errors.items()}
    except (TypeError, ValueError):
        return False, {}


def weighted_coverage_metrics(
    positions: np.ndarray,
    points: np.ndarray,
    weights: np.ndarray,
) -> dict[str, float]:
    delta = positions[:, :, None] - points[:, None, :]
    nearest_sq = np.min(np.sum(delta * delta, axis=0), axis=0)
    return {
        "coverage_cost_m2": float(weights @ nearest_sq),
        "weighted_fraction_within_0_25m": float(weights[nearest_sq <= 0.25**2].sum()),
        "weighted_fraction_within_0_35m": float(weights[nearest_sq <= 0.35**2].sum()),
        "weighted_fraction_within_0_50m": float(weights[nearest_sq <= 0.50**2].sum()),
        "weighted_rms_distance_m": float(np.sqrt(weights @ nearest_sq)),
    }


def minimum_collision_center_pair_distance(
    poses: np.ndarray,
    center_offset_m: float = SIMULATOR_COLLISION_CENTER_OFFSET_M,
) -> float:
    """Match the simulator's offset collision-center spacing check.

    This is centre-to-centre spacing after the simulator's 2.5 cm forward
    offset, not robot-body edge-to-edge separation.
    """

    if poses.shape[1] < 2:
        return float("inf")
    centers = poses[:2] + center_offset_m * np.vstack(
        (np.cos(poses[2]), np.sin(poses[2]))
    )
    delta = centers[:, :, None] - centers[:, None, :]
    distances = np.sqrt(np.sum(delta * delta, axis=0))
    np.fill_diagonal(distances, np.inf)
    return float(np.min(distances))


def minimum_reference_point_boundary_clearance(
    positions: np.ndarray,
    bounds: np.ndarray = PHYSICAL_BOUNDS,
) -> float:
    """Return reference-pose distance to the nearest physical arena edge."""

    clearances = np.vstack(
        (
            positions[0] - bounds[0],
            bounds[1] - positions[0],
            positions[1] - bounds[2],
            bounds[3] - positions[1],
        )
    )
    return float(np.min(clearances))


def minimum_conservative_body_boundary_clearance(
    positions: np.ndarray,
    bounds: np.ndarray = PHYSICAL_BOUNDS,
    robot_radius_m: float = ROBOT_RADIUS_M,
) -> float:
    """Return disk-edge clearance using the published 11 cm robot diameter."""

    return minimum_reference_point_boundary_clearance(positions, bounds) - robot_radius_m


def _write_csv(rows: list[dict[str, float]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def _first_sustained_threshold(
    rows: list[dict[str, float]],
    field: str,
    threshold: float,
) -> float | None:
    values = np.array([row[field] for row in rows])
    for index in range(values.size):
        if np.all(values[index:] <= threshold):
            return float(rows[index]["time_s"])
    return None


def _save_summary_plot(
    path: Path,
    points: np.ndarray,
    density: np.ndarray,
    config: ExperimentConfig,
    trajectories: np.ndarray,
    rows: list[dict[str, float]],
    final_targets: np.ndarray,
) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    density_grid = density.reshape(config.grid_ny, config.grid_nx)
    xs = points[0].reshape(config.grid_ny, config.grid_nx)
    ys = points[1].reshape(config.grid_ny, config.grid_nx)
    times = np.array([row["time_s"] for row in rows])
    costs = np.array([row["coverage_cost_m2"] for row in rows])
    min_dist = np.array(
        [row["minimum_simulator_collision_center_pair_distance_m"] for row in rows]
    )
    hit = np.array([row["weighted_fraction_within_0_35m"] for row in rows])

    fig = plt.figure(figsize=(12.0, 6.6), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=(1.45, 1.0))
    ax_map = fig.add_subplot(grid[:, 0])
    ax_cost = fig.add_subplot(grid[0, 1])
    ax_safety = fig.add_subplot(grid[1, 1])

    contour = ax_map.contourf(xs, ys, density_grid, levels=18, cmap="YlOrRd", alpha=0.82)
    fig.colorbar(contour, ax=ax_map, label="workload density")
    colors = plt.cm.tab10(np.linspace(0, 1, config.robots))
    for robot_id in range(config.robots):
        ax_map.plot(
            trajectories[:, 0, robot_id],
            trajectories[:, 1, robot_id],
            color=colors[robot_id],
            linewidth=1.5,
        )
        ax_map.scatter(
            trajectories[0, 0, robot_id],
            trajectories[0, 1, robot_id],
            marker="x",
            s=38,
            color=colors[robot_id],
        )
        ax_map.scatter(
            trajectories[-1, 0, robot_id],
            trajectories[-1, 1, robot_id],
            marker="o",
            edgecolor="black",
            linewidth=0.6,
            s=58,
            color=colors[robot_id],
        )
    ax_map.scatter(final_targets[0], final_targets[1], marker="+", s=85, color="black")
    ax_map.set(
        xlim=(PHYSICAL_BOUNDS[0], PHYSICAL_BOUNDS[1]),
        ylim=(PHYSICAL_BOUNDS[2], PHYSICAL_BOUNDS[3]),
        xlabel="x (m)",
        ylabel="y (m)",
        title="Eight-robot density-weighted coverage\n× start, ● finish, + final Lloyd centroid",
        aspect="equal",
    )
    ax_map.grid(alpha=0.18)

    ax_cost.plot(times, costs, color="#1f77b4", linewidth=2.0)
    ax_cost.set(title="Weighted coverage objective", xlabel="time (s)", ylabel="J (m²)")
    ax_cost.grid(alpha=0.25)

    ax_safety.plot(
        times,
        min_dist,
        color="#2ca02c",
        label="offset collision-center spacing",
    )
    ax_safety.axhline(0.15, color="#d62728", linestyle="--", label="15 cm recommendation")
    ax_safety.set(xlabel="time (s)", ylabel="spacing (m)")
    ax_safety.grid(alpha=0.25)
    ax_hit = ax_safety.twinx()
    ax_hit.plot(times, hit, color="#9467bd", alpha=0.75, label="weight within 35 cm")
    ax_hit.set_ylabel("weighted coverage fraction")
    lines, labels = ax_safety.get_legend_handles_labels()
    lines2, labels2 = ax_hit.get_legend_handles_labels()
    ax_safety.legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    fig.suptitle("Robotarium local simulation evidence — not a hardware result", fontweight="bold")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def run_experiment(config: ExperimentConfig | None = None) -> dict[str, Any]:
    """Execute the experiment and return its machine-readable summary."""

    cfg = config or ExperimentConfig()
    cfg.validate()
    np.random.seed(cfg.seed)
    random.seed(cfg.seed)

    output_dir = Path(cfg.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    points, weights, density = make_quadrature_grid(
        OPERATING_BOUNDS, cfg.grid_nx, cfg.grid_ny
    )
    ids = np.arange(cfg.robots)

    r = robotarium.Robotarium(
        number_of_robots=cfg.robots,
        show_figure=cfg.show_figure,
        initial_conditions=INITIAL_CONDITIONS.copy(),
        sim_in_real_time=cfg.sim_in_real_time,
        skip_initialization=cfg.skip_initialization,
    )
    si_to_uni = create_si_to_uni_dynamics(
        linear_velocity_gain=1.0,
        angular_velocity_limit=2.2,
    )
    barrier = create_uni_barrier_certificate_with_boundary(
        safety_radius=cfg.barrier_safety_radius_m,
        barrier_gain=cfg.barrier_gain,
        projection_distance=cfg.barrier_projection_distance_m,
        velocity_magnitude_limit=cfg.barrier_velocity_limit_mps,
        boundary_points=OPERATING_BOUNDS.copy(),
    )

    rows: list[dict[str, float]] = []
    sampled_trajectories: list[np.ndarray] = []
    all_control_ms: list[float] = []
    minimum_collision_center_spacing = float("inf")
    minimum_reference_point_wall_clearance = float("inf")
    minimum_conservative_body_wall_clearance = float("inf")
    maximum_nominal_si_speed = 0.0
    maximum_safe_linear_speed = 0.0
    maximum_safe_angular_speed = 0.0
    barrier_intervention_steps = 0
    actuator_scaling_steps = 0
    final_targets = INITIAL_CONDITIONS[:2].copy()
    initialization_validation_available, initialization_validation_errors = (
        simulator_validation_snapshot(r)
    )

    for step in range(cfg.iterations):
        poses = r.get_poses()
        tic = time.perf_counter()
        centroids, _, cell_mass, adjacency = discrete_voronoi(
            poses[:2], points, weights, (cfg.grid_ny, cfg.grid_nx)
        )
        dxi_nominal = cfg.control_gain * (centroids - poses[:2])
        dxi_nominal = clip_columns(dxi_nominal, cfg.nominal_si_speed_limit_mps)
        dxu_nominal = si_to_uni(dxi_nominal, poses)
        dxu_barrier = barrier(dxu_nominal, poses)
        dxu_safe, actuator_scale = globally_limit_unicycle_wheels(
            dxu_barrier,
            maximum_wheel_speed_radps=cfg.maximum_wheel_speed_radps,
            maximum_angular_speed_radps=cfg.maximum_commanded_angular_speed_radps,
        )
        control_ms = 1000.0 * (time.perf_counter() - tic)
        all_control_ms.append(control_ms)

        intervention = np.linalg.norm(dxu_barrier - dxu_nominal, axis=0)
        if float(np.max(intervention)) > 1e-6:
            barrier_intervention_steps += 1
        if actuator_scale < 1.0 - 1e-12:
            actuator_scaling_steps += 1
        reference_point_clearance = minimum_reference_point_boundary_clearance(poses[:2])
        conservative_body_clearance = minimum_conservative_body_boundary_clearance(
            poses[:2]
        )
        minimum_collision_center_spacing = min(
            minimum_collision_center_spacing,
            minimum_collision_center_pair_distance(poses),
        )
        minimum_reference_point_wall_clearance = min(
            minimum_reference_point_wall_clearance,
            reference_point_clearance,
        )
        minimum_conservative_body_wall_clearance = min(
            minimum_conservative_body_wall_clearance,
            conservative_body_clearance,
        )
        maximum_nominal_si_speed = max(
            maximum_nominal_si_speed,
            float(np.max(np.linalg.norm(dxi_nominal, axis=0))),
        )
        maximum_safe_linear_speed = max(
            maximum_safe_linear_speed, float(np.max(np.abs(dxu_safe[0])))
        )
        maximum_safe_angular_speed = max(
            maximum_safe_angular_speed, float(np.max(np.abs(dxu_safe[1])))
        )

        if step % cfg.log_stride == 0 or step == cfg.iterations - 1:
            coverage = weighted_coverage_metrics(poses[:2], points, weights)
            residuals = np.linalg.norm(centroids - poses[:2], axis=0)
            degrees = adjacency.sum(axis=0)
            row: dict[str, float] = {
                "step": float(step),
                "time_s": float(step * cfg.time_step_s),
                **coverage,
                "minimum_simulator_collision_center_pair_distance_m": (
                    minimum_collision_center_pair_distance(poses)
                ),
                "minimum_reference_point_boundary_clearance_m": (
                    reference_point_clearance
                ),
                "minimum_conservative_body_boundary_clearance_m": (
                    conservative_body_clearance
                ),
                "centroid_residual_mean_m": float(np.mean(residuals)),
                "centroid_residual_max_m": float(np.max(residuals)),
                "cell_mass_min": float(np.min(cell_mass)),
                "cell_mass_max": float(np.max(cell_mass)),
                "voronoi_edges": float(np.count_nonzero(np.triu(adjacency, 1))),
                "voronoi_degree_mean": float(np.mean(degrees)),
                "barrier_intervention_mean": float(np.mean(intervention)),
                "barrier_intervention_max": float(np.max(intervention)),
                "global_actuator_scale": float(actuator_scale),
                "control_compute_ms": control_ms,
            }
            rows.append(row)
            sampled_trajectories.append(poses[:2].copy())
        final_targets = centroids

        r.set_velocities(ids, dxu_safe)
        r.step()

    final_poses = r.get_poses()
    r.set_velocities(ids, np.zeros((2, cfg.robots)))
    r.step()
    # The current official Python simulator and examples use debug().  The
    # 2024-11-20 PDF guide still shows the legacy call_at_scripts_end() name.
    r.debug()

    final_coverage = weighted_coverage_metrics(final_poses[:2], points, weights)
    initial_coverage = {
        key: rows[0][key]
        for key in (
            "coverage_cost_m2",
            "weighted_fraction_within_0_25m",
            "weighted_fraction_within_0_35m",
            "weighted_fraction_within_0_50m",
            "weighted_rms_distance_m",
        )
    }
    compute_ms = np.asarray(all_control_ms)
    validation_available, validation_errors = simulator_validation_snapshot(r)
    experiment_validation_available = (
        initialization_validation_available and validation_available
    )
    if experiment_validation_available:
        experiment_validation_errors = {
            key: validation_errors.get(key, 0)
            - initialization_validation_errors.get(key, 0)
            for key in set(validation_errors) | set(initialization_validation_errors)
            if validation_errors.get(key, 0)
            - initialization_validation_errors.get(key, 0)
            != 0
        }
    else:
        experiment_validation_errors = {}
    hard_errors = (
        validation_errors.get("robots_too_close", 0)
        + validation_errors.get("robots_outside_boundaries", 0)
        if validation_available
        else None
    )
    experiment_hard_errors = (
        experiment_validation_errors.get("robots_too_close", 0)
        + experiment_validation_errors.get("robots_outside_boundaries", 0)
        if experiment_validation_available
        else None
    )
    summary: dict[str, Any] = {
        "evidence_scope": "local Robotarium simulator only; no physical hardware run claimed",
        "official_simulator_commit": "307269846b2761528586e9c3f47d0a8bec21692f",
        "config": asdict(cfg),
        "nominal_control_duration_s": cfg.nominal_duration_s,
        "initial": initial_coverage,
        "final": final_coverage,
        "coverage_cost_reduction_fraction": float(
            1.0 - final_coverage["coverage_cost_m2"] / initial_coverage["coverage_cost_m2"]
        ),
        "minimum_simulator_collision_center_pair_distance_m": (
            minimum_collision_center_spacing
        ),
        "simulator_collision_center_offset_m": (
            SIMULATOR_COLLISION_CENTER_OFFSET_M
        ),
        "published_recommended_spacing_m": 0.15,
        "minimum_reference_point_boundary_clearance_m": (
            minimum_reference_point_wall_clearance
        ),
        "minimum_conservative_body_boundary_clearance_m": (
            minimum_conservative_body_wall_clearance
        ),
        "conservative_body_disk_radius_m": ROBOT_RADIUS_M,
        "barrier_intervention_step_fraction": barrier_intervention_steps / cfg.iterations,
        "global_actuator_scaling_step_fraction": actuator_scaling_steps / cfg.iterations,
        "maximum_nominal_si_speed_mps": maximum_nominal_si_speed,
        "maximum_safe_linear_speed_mps": maximum_safe_linear_speed,
        "maximum_safe_angular_speed_radps": maximum_safe_angular_speed,
        "control_compute_ms": {
            "mean": float(np.mean(compute_ms)),
            "p95": float(np.percentile(compute_ms, 95)),
            "p99": float(np.percentile(compute_ms, 99)),
            "max": float(np.max(compute_ms)),
            "published_control_period_ms": cfg.time_step_s * 1000.0,
            "steps_over_control_period": int(
                np.count_nonzero(compute_ms > cfg.time_step_s * 1000.0)
            ),
            "steps_over_0_5s_watchdog": int(np.count_nonzero(compute_ms > 500.0)),
        },
        "sustained_centroid_residual_max_below_0_05m_at_s": _first_sustained_threshold(
            rows, "centroid_residual_max_m", 0.05
        ),
        "simulator_validation_available": validation_available,
        "simulator_validation_errors": validation_errors if validation_available else None,
        "simulator_hard_error_count": hard_errors,
        "initialization_validation_available": initialization_validation_available,
        "initialization_validation_errors": (
            initialization_validation_errors
            if initialization_validation_available
            else None
        ),
        "experiment_phase_validation_available": experiment_validation_available,
        "experiment_phase_validation_errors": (
            experiment_validation_errors if experiment_validation_available else None
        ),
        "experiment_phase_hard_error_count": experiment_hard_errors,
        "artifact_files": {
            "metrics_text_csv": "robotarium_coverage_metrics.txt",
            "trajectory_npy": "robotarium_coverage_trajectory.npy",
            "summary_json_text": "robotarium_coverage_summary.txt",
            "summary_png": "robotarium_coverage_summary.png" if cfg.save_plot else None,
        },
    }

    metrics_path = output_dir / "robotarium_coverage_metrics.txt"
    trajectory_path = output_dir / "robotarium_coverage_trajectory.npy"
    summary_path = output_dir / "robotarium_coverage_summary.txt"
    plot_path = output_dir / "robotarium_coverage_summary.png"
    _write_csv(rows, metrics_path)
    trajectories = np.stack(sampled_trajectories)
    np.save(trajectory_path, trajectories)
    if cfg.save_plot:
        _save_summary_plot(
            plot_path,
            points,
            density,
            cfg,
            trajectories,
            rows,
            final_targets,
        )
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return summary


def main() -> None:
    # Local CI can set these environment variables.  A hardware upload can use
    # the untouched defaults, which enable the Robotarium visualization and its
    # real-time pacing while preserving the same controller and experiment.
    config = ExperimentConfig(
        show_figure=os.environ.get("ROBOTARIUM_HEADLESS", "0") != "1",
        sim_in_real_time=os.environ.get("ROBOTARIUM_FAST", "0") != "1",
        output_dir=os.environ.get("ROBOTARIUM_OUTPUT_DIR", "."),
    )
    run_experiment(config)


if __name__ == "__main__":
    main()
