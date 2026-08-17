import copy
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.compare_reproduction import compare_reproduction


EVIDENCE = Path(__file__).parents[1] / "evidence" / "simulator-3072698"


def frozen_inputs():
    summary = json.loads(
        (EVIDENCE / "robotarium_coverage_summary.json").read_text(encoding="utf-8")
    )
    trajectory = np.load(
        EVIDENCE / "robotarium_coverage_trajectory.npy", allow_pickle=False
    )
    return summary, trajectory


def test_exact_baseline_returns_a_zero_delta_report():
    frozen, trajectory = frozen_inputs()
    report = compare_reproduction(frozen, frozen, trajectory, trajectory)
    assert set(report.values()) == {0.0}


def test_observed_first_hosted_run_delta_envelope_passes():
    frozen, frozen_trajectory = frozen_inputs()
    reproduced = copy.deepcopy(frozen)
    reproduced.update(
        {
            "coverage_cost_reduction_fraction": 0.37958188087515887,
            "minimum_simulator_collision_center_pair_distance_m": (
                0.5021325901338078
            ),
            "minimum_reference_point_boundary_clearance_m": (
                frozen["minimum_reference_point_boundary_clearance_m"] + 7.8e-16
            ),
            "minimum_conservative_body_boundary_clearance_m": (
                frozen["minimum_conservative_body_boundary_clearance_m"] + 7.8e-16
            ),
            "maximum_safe_linear_speed_mps": (
                frozen["maximum_safe_linear_speed_mps"] + 9.9e-15
            ),
            "maximum_safe_angular_speed_radps": 3.4909043705816285,
            "barrier_intervention_step_fraction": 0.42722222222222223,
            "global_actuator_scaling_step_fraction": 0.036111111111111108,
        }
    )
    reproduced["final"]["coverage_cost_m2"] = 0.082542521865082624
    reproduced["final"]["weighted_rms_distance_m"] = 0.28730214385744257

    reproduced_trajectory = frozen_trajectory + 1.8628449475410525e-6
    reproduced_trajectory[133, 1, 7] = (
        frozen_trajectory[133, 1, 7] - 2.360891978403501e-5
    )

    report = compare_reproduction(
        frozen, reproduced, frozen_trajectory, reproduced_trajectory
    )
    assert np.isclose(
        report["minimum_collision_center_spacing_delta_m"],
        -0.005629846505483038,
    )
    assert report["trajectory_max_abs_delta_m"] < 5e-5
    assert report["trajectory_rms_delta_m"] < 5e-6


def test_safety_validation_and_configuration_gates_are_strict():
    frozen, trajectory = frozen_inputs()

    unsafe = copy.deepcopy(frozen)
    unsafe["minimum_simulator_collision_center_pair_distance_m"] = 0.149
    with pytest.raises(ValueError, match="published 0.15 m recommendation"):
        compare_reproduction(frozen, unsafe, trajectory, trajectory)

    simulator_error = copy.deepcopy(frozen)
    simulator_error["experiment_phase_validation_errors"] = {
        "robots_outside_boundaries": 1
    }
    simulator_error["experiment_phase_hard_error_count"] = 1
    with pytest.raises(ValueError, match="zero errors or warnings"):
        compare_reproduction(frozen, simulator_error, trajectory, trajectory)

    changed_guard = copy.deepcopy(frozen)
    changed_guard["config"]["maximum_wheel_speed_radps"] = 12.5
    with pytest.raises(ValueError, match="configuration mismatch"):
        compare_reproduction(frozen, changed_guard, trajectory, trajectory)


def test_outcome_and_actuator_regressions_fail():
    frozen, trajectory = frozen_inputs()

    worse_cost = copy.deepcopy(frozen)
    worse_cost["final"]["coverage_cost_m2"] += 2e-9
    with pytest.raises(ValueError, match="final.coverage_cost_m2"):
        compare_reproduction(frozen, worse_cost, trajectory, trajectory)

    excessive_rotation = copy.deepcopy(frozen)
    excessive_rotation["maximum_safe_angular_speed_radps"] = 3.5000001
    with pytest.raises(ValueError, match="maximum_safe_angular_speed_radps"):
        compare_reproduction(frozen, excessive_rotation, trajectory, trajectory)


def test_maximum_and_rms_trajectory_regressions_fail():
    frozen, trajectory = frozen_inputs()

    one_large_delta = trajectory.copy()
    one_large_delta[100, 0, 0] += 5.1e-5
    with pytest.raises(ValueError, match="maximum absolute delta"):
        compare_reproduction(frozen, frozen, trajectory, one_large_delta)

    distributed_drift = trajectory + 5.1e-6
    with pytest.raises(ValueError, match="RMS delta"):
        compare_reproduction(frozen, frozen, trajectory, distributed_drift)
