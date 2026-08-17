import json
from pathlib import Path

import numpy as np

from experiment import (
    ExperimentConfig,
    INITIAL_CONDITIONS,
    OPERATING_BOUNDS,
    ROBOT_RADIUS_M,
    SIMULATOR_COLLISION_CENTER_OFFSET_M,
    clip_columns,
    discrete_voronoi,
    globally_limit_unicycle_wheels,
    make_quadrature_grid,
    minimum_collision_center_pair_distance,
    minimum_conservative_body_boundary_clearance,
    minimum_reference_point_boundary_clearance,
    run_experiment,
    simulator_validation_snapshot,
    weighted_coverage_metrics,
)


def test_default_configuration_stays_inside_published_limits():
    cfg = ExperimentConfig()
    cfg.validate()
    assert cfg.robots == 8
    assert np.isclose(cfg.nominal_duration_s, 118.8)
    assert cfg.nominal_duration_s < 900.0
    assert cfg.barrier_safety_radius_m >= 0.15
    assert cfg.nominal_si_speed_limit_mps < 0.2
    assert cfg.maximum_wheel_speed_radps < 12.5
    assert cfg.maximum_commanded_angular_speed_radps < 3.6
    frozen = json.loads(
        (Path(__file__).parents[1] / "config.json").read_text(encoding="utf-8")
    )
    assert frozen["robots"] == cfg.robots
    assert frozen["iterations"] == cfg.iterations
    assert np.isclose(
        frozen["nominal_control_duration_seconds"], cfg.nominal_duration_s
    )
    assert frozen["operating_bounds_m"] == OPERATING_BOUNDS.tolist()
    assert frozen["published_robot_radius_m"] == ROBOT_RADIUS_M
    assert (
        frozen["simulator_collision_center_offset_m"]
        == SIMULATOR_COLLISION_CENTER_OFFSET_M
    )
    assert frozen["maximum_wheel_speed_radps"] == cfg.maximum_wheel_speed_radps
    assert (
        frozen["maximum_commanded_angular_speed_radps"]
        == cfg.maximum_commanded_angular_speed_radps
    )


def test_weighted_voronoi_is_deterministic_and_finite():
    points, weights, _ = make_quadrature_grid(OPERATING_BOUNDS, 21, 13)
    positions = INITIAL_CONDITIONS[:2]
    first = discrete_voronoi(positions, points, weights, (13, 21))
    second = discrete_voronoi(positions, points, weights, (13, 21))
    for lhs, rhs in zip(first, second, strict=True):
        np.testing.assert_array_equal(lhs, rhs)
    centroids, owners, cell_mass, adjacency = first
    assert np.isfinite(centroids).all()
    assert owners.shape == (21 * 13,)
    assert np.all(cell_mass > 0.0)
    assert np.isclose(cell_mass.sum(), 1.0)
    assert np.array_equal(adjacency, adjacency.T)
    assert not np.any(np.diag(adjacency))


def test_exact_lloyd_centroid_update_does_not_increase_discrete_cost():
    points, weights, _ = make_quadrature_grid(OPERATING_BOUNDS, 31, 19)
    positions = INITIAL_CONDITIONS[:2]
    before = weighted_coverage_metrics(positions, points, weights)["coverage_cost_m2"]
    centroids, _, _, _ = discrete_voronoi(positions, points, weights, (19, 31))
    after = weighted_coverage_metrics(centroids, points, weights)["coverage_cost_m2"]
    assert after <= before + 1e-12


def test_geometry_metrics_and_speed_clipping():
    poses = INITIAL_CONDITIONS.copy()
    assert minimum_collision_center_pair_distance(poses) > 0.15
    reference_clearance = minimum_reference_point_boundary_clearance(poses[:2])
    body_clearance = minimum_conservative_body_boundary_clearance(poses[:2])
    assert reference_clearance > 0.15
    assert np.isclose(body_clearance, reference_clearance - ROBOT_RADIUS_M)
    assert body_clearance > 0.15
    velocities = np.array([[0.3, 0.0], [0.4, -0.5]])
    clipped = clip_columns(velocities, 0.1)
    assert np.all(np.linalg.norm(clipped, axis=0) <= 0.1 + 1e-12)
    dxu = np.array([[0.2, 0.1], [4.0, -2.0]])
    limited, scale = globally_limit_unicycle_wheels(dxu)
    wheel_speeds = np.vstack(
        (
            (2.0 * limited[0] - 0.11 * limited[1]) / (2.0 * 0.016),
            (2.0 * limited[0] + 0.11 * limited[1]) / (2.0 * 0.016),
        )
    )
    assert scale < 1.0
    assert np.max(np.abs(wheel_speeds)) <= 12.0 + 1e-12
    assert np.max(np.abs(limited[1])) <= 3.5 + 1e-12


def test_simulator_validation_snapshot_is_optional_and_guarded():
    class HardwareLikeRuntime:
        pass

    class LocalSimulator:
        _errors = {"robots_too_close": 0, "exceeded_actuator_limits": 2}

    assert simulator_validation_snapshot(HardwareLikeRuntime()) == (False, {})
    assert simulator_validation_snapshot(LocalSimulator()) == (
        True,
        {"robots_too_close": 0, "exceeded_actuator_limits": 2},
    )


def test_short_headless_simulator_smoke_run(tmp_path: Path):
    summary = run_experiment(
        ExperimentConfig(
            iterations=40,
            grid_nx=17,
            grid_ny=11,
            log_stride=10,
            show_figure=False,
            sim_in_real_time=False,
            skip_initialization=True,
            save_plot=False,
            output_dir=str(tmp_path),
        )
    )
    assert summary["experiment_phase_validation_available"] is True
    assert summary["experiment_phase_hard_error_count"] == 0
    assert (
        summary["minimum_simulator_collision_center_pair_distance_m"] > 0.15
    )
    assert np.isclose(
        summary["minimum_conservative_body_boundary_clearance_m"],
        summary["minimum_reference_point_boundary_clearance_m"] - ROBOT_RADIUS_M,
    )
    assert summary["final"]["coverage_cost_m2"] < summary["initial"]["coverage_cost_m2"]
    loaded = json.loads((tmp_path / "robotarium_coverage_summary.txt").read_text())
    assert loaded["evidence_scope"].startswith("local Robotarium simulator")
    assert (tmp_path / "robotarium_coverage_metrics.txt").is_file()
    assert (tmp_path / "robotarium_coverage_trajectory.npy").is_file()
