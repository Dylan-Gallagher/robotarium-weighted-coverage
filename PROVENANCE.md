# Provenance and evidence scope

## Source lineage

- Official upstream repository: `https://github.com/robotarium/robotarium_python_simulator`
- Official upstream base: `307269846b2761528586e9c3f47d0a8bec21692f`
- Reviewed local pilot commit: `a233f3ddb350322654f800b8884571a198d9bf0b`
- Extraction date: 2026-08-17

The standalone `experiment.py` is byte-for-byte identical to the file at
`pilot/weighted_coverage/experiment.py` in the reviewed local pilot commit:

```text
48555dc3ba5bf9f03f80e3217a5fb6596477d2752876956a1ac41cc3d0021ab4  experiment.py
```

`config.json` is also byte-for-byte identical. The test import path was changed
mechanically from the former nested pilot package to the standalone root module;
test behavior is otherwise unchanged.

No file from the upstream `rps` package, upstream examples, upstream guide, or
upstream setup metadata is included here. Reproduction fetches that dependency
from its pinned official Git commit.

## Evidence scope

The committed evidence was generated locally with the official simulator at
the pinned base commit and seed `20260817`. It contains no physical Robotarium
measurement. The summary image includes an explicit simulator-only banner, and
the JSON summary identifies its scope as:

```text
local Robotarium simulator only; no physical hardware run claimed
```

The evidence filenames use `.csv` and `.json` extensions in this standalone
repository for accurate media identification. Their bytes are unchanged from
the original simulator outputs named `robotarium_coverage_metrics.txt` and
`robotarium_coverage_summary.txt`, respectively.

Wall-clock computation fields vary by host and are not reproduction gates.
Controller outputs, trajectories, safety geometry, and coverage measurements
are verified independently by the supplied comparison script.

## First hosted-run delta audit

GitHub Actions run
[`31993185231`](https://github.com/Dylan-Gallagher/robotarium-weighted-coverage/actions/runs/31993185231)
was the first hosted 3,600-step reproduction. It used the same controller,
seed, official simulator commit, and public direct-package pins as the local
public-pin control described below. The simulator reported no errors or
warnings.

The full audit measured these hosted-minus-frozen deltas:

| Field | Frozen | Hosted | Hosted - frozen |
|---|---:|---:|---:|
| Coverage-cost reduction | 0.3795818808839646 | 0.3795818808751589 | -8.8057e-12 |
| Final coverage cost (m²) | 0.08254252186392068 | 0.08254252186508262 | +1.1620e-12 |
| Final weighted RMS distance (m) | 0.2873021438554204 | 0.2873021438574426 | +2.0222e-12 |
| Minimum offset collision-center spacing (m) | 0.5077624366392909 | 0.5021325901338078 | -0.0056298465 |
| Maximum safe angular speed (rad/s) | 3.4908995712287867 | 3.4909043705816285 | +4.7994e-6 |
| Barrier-intervention step fraction | 0.2027777777777778 | 0.4272222222222222 | +0.2244444444 |
| Actuator-scaling step fraction | 0.0141666666666667 | 0.0361111111111111 | +0.0219444444 |

Across all `361 x 2 x 8` sampled XY coordinates, the maximum absolute delta
was `2.3608919784e-5 m`, the RMS delta was `1.8628449475e-6 m`, and the final
maximum XY delta was `3.37677e-6 m`.

### Environment boundary and three-way control

The frozen PNG metadata identifies Matplotlib 3.11.1. The public repository
pins Matplotlib 3.10.8, so the frozen and hosted environments were not fully
identical. The original frozen files do not embed NumPy or CVXOPT versions;
their original values are unknown and are not inferred here. The integrity
verifier confirms the five committed files against their frozen manifest; it
does not claim a later experiment regenerated timing-bearing file bytes.

A separate clean control of the candidate's unchanged controller and comparison
gates used Python 3.12.13 on Fedora Linux with the exact public direct pins:
NumPy 2.2.6, Matplotlib 3.10.8, and CVXOPT 1.3.2. It reproduced the frozen
trajectory hash byte for byte and returned zero deltas for every comparator
field. The hosted Ubuntu run used Python 3.12.13 and the same public direct
pins. Both install logs list the same Python package version strings, but
transitive packages remain resolver-selected, wheel artifacts and system math
libraries are not locked, and the OS, BLAS runtime, and CPU environment differ.

This audit separates the historical artifact's incomplete dependency provenance
from the two public-pin controls. Their observed difference is
cross-environment; the available records do not isolate a single dependency,
binary artifact, numerical library, or platform component as its cause.

Inspection of the pinned official source showed that its CVXOPT barrier QP uses
`reltol=1e-2` and `feastol=1e-2`. The hosted trajectory first exceeds a
`1e-12 m` delta at sample 102 (33.66 s), well after both summaries' 3.63 s
sustained-convergence marker, and larger differences are concentrated in
angular/heading behavior with negligible XY effect. This is consistent with
near-equilibrium QP numerical sensitivity, but it does not prove CVXOPT, BLAS,
package artifacts, the operating system, or any other component as the sole
cause. No simulator error or coverage regression was observed.

Barrier-intervention and actuator-scaling fractions are therefore reported as
environment-sensitive threshold diagnostics rather than outcome gates.

The semantic cross-environment contract in `scripts/compare_reproduction.py` was
introduced after this first hosted run and corrected after the clean public-pin
control. It was not part of the original frozen run. The contract preserves
strict scope, configuration, zero-error, safety, boundary, and configured
actuator invariants; tight field-specific outcome tolerances; a 0.01 m spacing
envelope around the frozen measurement; and full-trajectory limits of `5e-5 m`
maximum per coordinate and `5e-6 m` RMS.
