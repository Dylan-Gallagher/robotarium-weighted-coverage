# Third-party notice and provenance boundary

This repository contains original experiment code, tests, documentation,
configuration, and generated simulator evidence. It does **not** contain a copy
of the Robotarium Python Simulator or any other third-party package.

The project imports the official `rps` API at runtime. Reproduction installs
that API directly from Georgia Tech's public repository at the exact commit
listed below. The package in turn declares NumPy, Matplotlib, and CVXOPT as its
runtime dependencies.

| Component | Pinned version | License in primary source | Role |
|---|---|---|---|
| [Robotarium Python Simulator](https://github.com/robotarium/robotarium_python_simulator/blob/307269846b2761528586e9c3f47d0a8bec21692f/LICENSE) | commit `307269846b2761528586e9c3f47d0a8bec21692f` | MIT; copyright Robotarium Organization | simulator and `rps` control/safety API |
| [NumPy](https://github.com/numpy/numpy/blob/v2.2.6/LICENSE.txt) | 2.2.6 | BSD-3-Clause for the main project; distributions may carry additional notices | numerical arrays |
| [Matplotlib](https://github.com/matplotlib/matplotlib/blob/v3.10.8/LICENSE/LICENSE) | 3.10.8 | Matplotlib License (PSF-compatible) | optional evidence plot |
| [CVXOPT](https://github.com/cvxopt/cvxopt/blob/1.3.2/LICENSE) | 1.3.2 | GPL-3.0-or-later | quadratic program used by the Robotarium barrier certificate |
| [pytest](https://github.com/pytest-dev/pytest/blob/9.0.2/LICENSE) | 9.0.2 | MIT | tests only |

The official simulator's pinned [`setup.py`](https://github.com/robotarium/robotarium_python_simulator/blob/307269846b2761528586e9c3f47d0a8bec21692f/setup.py)
declares NumPy, Matplotlib, and CVXOPT and labels the simulator itself MIT.
These primary files were checked on 2026-08-17.

The MIT license in this repository applies only to its original project files.
Installing or redistributing a combined runtime does not replace any dependency
license. In particular, CVXOPT is GPL-3.0-or-later; downstream users and
distributors are responsible for satisfying those terms. Package wheels can
also include separately licensed native components, whose notices travel with
the installed distributions.

"Robotarium" and Georgia Tech names identify the target research platform and
their upstream work. They do not imply sponsorship, certification, affiliation,
or endorsement. The simulator's printed acceptance message is retained in the
evidence log as output from a local run, not as a Georgia Tech hardware result.
