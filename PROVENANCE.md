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

Wall-clock computation fields vary by host and are not used as deterministic
reproduction gates. Controller outputs, trajectories, safety geometry, and
coverage measurements are verified independently by the supplied comparison
script.
