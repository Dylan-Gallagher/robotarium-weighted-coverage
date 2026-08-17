# Density-weighted multi-robot coverage for Robotarium

> **Evidence status:** local Robotarium Python Simulator only. No physical
> Robotarium run, hardware validation, account submission, or peer-to-peer
> communication claim is made anywhere in this repository.

This repository contains a fixed-seed eight-robot coverage-control pilot.
Each robot follows the density-weighted centroid of its discrete Voronoi cell;
the official Robotarium single-integrator-to-unicycle transform and
boundary-aware barrier certificate filter the resulting commands. A final
common actuator scale leaves margin below the platform limits documented by
Georgia Tech.

The frozen local simulation reduced the density-weighted coverage objective by
**37.96%**, from **0.133043 m²** to **0.082543 m²**, while the simulator
reported no errors or warnings. These are simulator measurements, not physical
robot results.

![Local simulation summary](evidence/simulator-3072698/robotarium_coverage_summary.png)

## Why this is a standalone repository

Only the original experiment, tests, configuration, and generated evidence are
stored here. The Georgia Tech simulator is **not copied or vendored**. Setup
fetches the [official Robotarium Python Simulator at commit
`307269846b2761528586e9c3f47d0a8bec21692f`](https://github.com/robotarium/robotarium_python_simulator/tree/307269846b2761528586e9c3f47d0a8bec21692f).
That exact dependency is the base on which the pilot was developed and the
version used for the frozen evidence.

The publication candidate was extracted without modifying `experiment.py`
from reviewed local pilot commit
`a233f3ddb350322654f800b8884571a198d9bf0b`, an unpublished descendant of the
official base. The source hash is recorded in
[`PROVENANCE.md`](PROVENANCE.md).

## Method

The operational region is a 2.84 x 1.64 m rectangle inset by 18 cm from the
published 3.2 x 2.0 m Robotarium boundary. A deterministic 45 x 27 quadrature
grid represents a nonuniform workload with a uniform floor, three unequal
Gaussian peaks, and a low ridge. Eight robots start in a fixed, well-separated
2 x 4 arrangement.

At each 0.033 s control step, the experiment:

1. obtains the eight poses through the public `get_poses()` API;
2. assigns every workload sample to its nearest robot, using lowest robot ID
   for deterministic tie-breaking;
3. computes each cell's density-weighted centroid;
4. caps nominal single-integrator feedback at 0.11 m/s and maps it to unicycle
   commands with the official Robotarium transform;
5. filters commands with the official boundary-aware unicycle barrier
   certificate, configured with a 0.15 m safety radius and 0.14 m/s magnitude
   limit; and
6. applies one common scale so requested wheel speed remains at or below
   12.0 rad/s and requested body rotation remains at or below 3.5 rad/s.

Each nominal command is defined by one robot's own Voronoi cell, but this
implementation vectorizes the partition from globally tracked poses and uses
the simulator's centralized barrier QP. It is a distributed-by-agent coverage
law implemented on a centralized testbed, **not** a peer-to-peer system.

## Frozen simulator result

The 3,600-step control phase represents 118.8 s. Including ordinary simulator
initialization, the pinned simulator reported 185.30 s of simulated execution.
The random seed is `20260817`.

| Metric | Initial | Final / worst local-simulator value |
|---|---:|---:|
| Density-weighted coverage cost | 0.133043 m² | 0.082543 m² (-37.96%) |
| Workload within 0.35 m of a robot | 56.54% | 75.91% (+19.36 points) |
| Workload within 0.50 m of a robot | 84.34% | 98.63% (+14.30 points) |
| Weighted RMS distance | 0.3648 m | 0.2873 m |
| Minimum offset collision-center spacing | - | 0.5078 m |
| Minimum reference-pose boundary clearance | - | 0.3752 m |
| Minimum conservative disk-edge boundary clearance | - | 0.3202 m |
| Maximum commanded linear speed | - | 0.1097 m/s |
| Maximum commanded angular speed | - | 3.4909 rad/s |
| Local control compute time | - | 2.99 ms p99; 4.83 ms max |
| Local simulator validation | - | no errors or warnings |

The spacing metric follows the pinned simulator's collision check: pose
references are shifted 2.5 cm forward, then measured center-to-center. It is
not an edge-to-edge body gap. The conservative boundary metric subtracts a
5.5 cm disk radius from the reference-pose clearance, based on the published
11 cm robot diameter.

The timing values above describe the machine that generated the frozen local
run; they are not hardware latency measurements and are expected to vary on
other computers. On that same frozen host, barrier filtering changed the
nominal command on 20.28% of control steps, and the common actuator scale
activated on 1.42%. Those two fractions count threshold crossings near
equilibrium; they are environment-sensitive diagnostics, not cross-environment
outcome gates.

## Cross-environment reproduction audit

The first GitHub-hosted full reproduction, [Actions run
31993185231](https://github.com/Dylan-Gallagher/robotarium-weighted-coverage/actions/runs/31993185231),
used the same controller, seed, pinned simulator commit, public direct-package
pins, and 3,600 control steps. It reproduced the coverage reduction within
`8.81e-12`, had a maximum sampled XY-coordinate difference of `2.361e-5 m`
(23.61 micrometres) and an XY RMS difference of `1.863e-6 m`. Simulator
validation remained available with zero errors or warnings.

Its minimum offset collision-center spacing was `0.5021326 m`, compared with
`0.5077624 m` on the frozen host: a `0.0056298 m` (1.109%) difference. The
hosted value still exceeded the published 0.15 m recommendation by 0.3521 m.
The 0.5078 m value in the table remains the exact frozen-host measurement; it
is not relabelled as a byte-exact cross-environment result.

The frozen PNG embeds Matplotlib 3.11.1, while the public pin and hosted run use
3.10.8. The original frozen run did not record NumPy or CVXOPT versions, so the
frozen environment is only partially known. The five committed frozen files
pass their SHA-256 manifest; that integrity check does not claim a later run
regenerated timing-bearing metrics, summary, log, or plot bytes.

A fresh Fedora control under the exact public direct pins (NumPy 2.2.6,
Matplotlib 3.10.8, CVXOPT 1.3.2) reproduced the frozen trajectory byte for byte
and all gated summary fields exactly. The hosted Ubuntu run used the same
public direct pins, and both install logs list the same Python package version
strings. However, transitive packages are resolver-selected, wheel builds and
system math libraries are not locked, and the operating system, BLAS runtime,
and CPU environment differ. The audit therefore establishes bounded
cross-environment variability, not a single isolated cause.

The first hosted XY delta above `1e-12 m` occurred at 33.66 s, well after the
3.63 s sustained-convergence marker. The pinned official barrier utility solves
through CVXOPT with relative and feasibility tolerances of `1e-2`, and the
larger trace differences are in angular/heading behavior near equilibrium.
This pattern is consistent with environment-sensitive barrier-QP numerics, but
it does not isolate CVXOPT, BLAS, package artifacts, the operating system, or
another environment component as the sole cause. Threshold diagnostics changed
more visibly (barrier intervention: 20.28% frozen versus 42.72% hosted;
actuator scaling: 1.42% versus 3.61%) while XY motion, safety margin, and
coverage were stable.

The cross-environment comparison contract was introduced transparently after
that first hosted run and corrected after the clean public-pin control
established the evidence above. It retains exact controller configuration,
simulator commit, evidence scope, validation availability, and zero-error
requirements. It also enforces configured command limits, positive boundary
clearance, at least 0.15 m collision-center spacing, no more than 0.01 m
spacing drift from the frozen run, field-specific outcome tolerances, and
trajectory limits of 50 micrometres per XY coordinate and 5 micrometres RMS.
Tests demonstrate that unsafe spacing, simulator errors, changed guards,
outcome regressions, actuator regressions, and meaningful trajectory drift all
fail the comparison.

## Reproduce from a clean checkout

Prerequisites are Git and Python 3.10-3.12. The direct package versions and
official simulator commit are pinned in [`pyproject.toml`](pyproject.toml);
transitive packages retain the constraints selected by those distributions.

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pip freeze --all
.venv/bin/python scripts/verify_frozen_evidence.py
.venv/bin/python -m pytest -q

mkdir -p results/reproduction
ROBOTARIUM_HEADLESS=1 ROBOTARIUM_FAST=1 \
  ROBOTARIUM_OUTPUT_DIR=results/reproduction \
  .venv/bin/python experiment.py | tee results/reproduction/simulator.log
.venv/bin/python scripts/compare_reproduction.py \
  evidence/simulator-3072698/robotarium_coverage_summary.json \
  results/reproduction/robotarium_coverage_summary.txt \
  evidence/simulator-3072698/robotarium_coverage_trajectory.npy \
  results/reproduction/robotarium_coverage_trajectory.npy
```

The comparison deliberately excludes wall-clock compute-time fields, plot
bytes, and the two environment-sensitive threshold diagnostics. It reports
those diagnostic deltas while enforcing the documented semantic outcome,
safety, actuator, validation, and full sampled-trajectory gates above. The test
suite also includes a short end-to-end simulator run. Continuous integration
performs the same integrity, test, and full-reproduction checks.

## Evidence files

[`evidence/simulator-3072698`](evidence/simulator-3072698) contains the frozen
local run:

- comma-delimited time-series metrics;
- sampled poses as a NumPy array with shape `361 x 2 x 8`;
- machine-readable summary JSON;
- summary figure; and
- captured simulator output.

All hashes are pinned in
[`evidence/simulator-3072698/MANIFEST.sha256`](evidence/simulator-3072698/MANIFEST.sha256).
Run the integrity checker before quoting any result.

## Hardware boundary

`experiment.py` uses only the public `get_poses`, `set_velocities`, `step`, and
`debug` methods for control. One isolated helper optionally reads the local
simulator's private `_errors` dictionary for evidence reporting. If a hardware
runtime omits it, validation is marked unavailable rather than falsely reported
as zero; control and file generation do not depend on it.

The pinned 2024-11-20 guide uses the older `call_at_scripts_end()` name, while
the pinned simulator source exposes `debug()` and its examples use `debug()`.
This project follows the tested pinned source. Confirm the live runtime before
any future upload.

A physical run requires a separately approved Robotarium account and submission.
After a real run, preserve the returned script, settings, logs, data, and video;
verify them independently before adding any hardware claim. Until that happens,
the accurate description is **Robotarium-ready local simulation**.

## Official references

- [Robotarium Get Started](https://www.robotarium.gatech.edu/get-started)
- [Robotarium FAQ](https://www.robotarium.gatech.edu/faqs)
- [Pinned official Python simulator](https://github.com/robotarium/robotarium_python_simulator/tree/307269846b2761528586e9c3f47d0a8bec21692f)
- [Pinned official Python guide](https://github.com/robotarium/robotarium_python_simulator/blob/307269846b2761528586e9c3f47d0a8bec21692f/Robotarium_Python_Guide.pdf)

See [`NOTICE.md`](NOTICE.md) for dependency attribution and license boundaries.
This project is independent and is not endorsed by or affiliated with Georgia
Tech or the Robotarium organization.

## License

Original project files are released under the MIT License; see [`LICENSE`](LICENSE).
No third-party package is redistributed here. Installed dependencies retain
their own licenses, including CVXOPT's GPL-3.0-or-later terms; see
[`NOTICE.md`](NOTICE.md).
