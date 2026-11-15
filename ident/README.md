# Cable-drive dynamics identification (`cable_ident`)

Identifies an arm's J1-J4 dynamics and writes the parameters the controller
compiles in (`src/lib/cable_params_*.h`, used by `--model cable`).

## Model

    tau = W(q, dq, ddq) beta            rigid body, 22 base parameters
        + TM^T diag(I) TM ddq           motor rotors
        + TM^T f(TM q) + f_J2,J3(q)     cable friction: Jenkins elements
        + TM^T diag(v) TM dq            viscous friction at the motors
        + offset                        per joint

`TM` maps joint angles to motor angles. Motors 2 and 3 drive J2 and J3
together through the cable differential, so the friction is placed in motor
coordinates, with a second bank on J2 and J3.

The WAM's cable capstans have no backlash, and the motor encoders see the
joint's motion amplified 18 to 42 times. After a reversal the friction rises
with how far the motor has turned, not with how fast. Each friction element is
a spring of slip displacement `delta` in series with a Coulomb slider of force
`w`:

    z <- clamp(z + dx, -delta, delta)        f = sum_i w_i z_i / delta_i

The slip displacements are a fixed grid (5 to 170 encoder counts), so the
model is linear in `w`. The elements are driven by position increments and do
not chatter with velocity noise at rest. With `w >= 0` each element is
passive.

The controller's version is `src/lib/cable_drive_model.h`.

## Identification

    min  sum_j w_j ||G_j theta - y_j||^2 / N
         + sum_i lambda_i D_logdet(J_i, J_i^CAD) + s sum_k (w_k - w_k+1)^2
    s.t. J_i >= eps I  (links 1-4),   w >= 0,  v >= 0,  I >= 0

One semidefinite program (`cable_ident/ident.py`, solved with Clarabel):

* the pseudo-inertia constraints keep the rigid body physically realizable
  (Wensing et al., 2017);
* the log-determinant divergence to the CAD model regularizes links 1-3
  (Lee et al., 2020). Link 4 carries the wrist and gets no prior;
* nonnegative element forces keep the friction passive.

The data passes through the controller's own signal chain before the fit: its
velocity, its acceleration filter, encoder increments for the elements. The
model that is identified is the one that runs at 500 Hz.

## Setup

On any computer with the recordings; ROS is not needed.

```bash
python3 -m venv ~/.venvs/cable_ident
~/.venvs/cable_ident/bin/pip install -r requirements.txt
cd wam_ws/src/wam_teleop/ident
~/.venvs/cable_ident/bin/python -m cable_ident --help
```

The tool compiles the controller's regressor (`src/lib/regressor_W.h`) with
`g++` when first run. Bag extractions are cached in `~/.cache/cable_ident`
(`CABLE_IDENT_CACHE` moves it).

## Commands

| Command | What it does |
| --- | --- |
| `fit --arm NAME` | fits the model on the arm's excitation logs and self-driven teleop segments; writes `src/lib/<header>` and a `.json` record |
| `cv --arm NAME` | leave-one-session-out: fits without one teleop session and scores that session |
| `ablate --arm NAME` | the same for the model's variants (no friction elements, joint-space elements, no rotor inertia, ...) |
| `hysteresis --arm NAME` | friction after reversals against displacement and against time |
| `design --arm NAME --out traj.txt` | the next excitation trajectory: D-optimal against the data already fitted, inside the configurations the arm visited in teleop, plus reversal ladders |
| `testvectors` | rewrites `test/data` for the C++ tests |

`--exc-weight` sets the excitation's weight relative to the teleop segments.

## Data

`cable_ident/arms.py` lists, per arm, its configuration and its logs. Paths are
relative to the workspace: `excitation/` and `teleop_logs/` beside `wam_ws/`.

* Excitation logs, `excitation/*.dat`, from `dynamics_excitation_*` or
  `wam cal excite`. Record several runs. They are stacked into one fit, not
  averaged first, so every sample counts and one run can be left out to
  check the others. Both formats are read: the four-joint one (t, q, tau at
  250 Hz) and the full one written with `--trajectory` (t, q, dq, tau at
  500 Hz).
* Self-driven teleop segments. In every session each arm drives itself to the
  sync pose before anyone touches it. `data.self_driven` finds these moves in
  the session's bag. They are free motion in the teleop's own workspace, which
  the standard excitation does not cover.

After a zero or gravity calibration start a new list in `arms.py`. Data from
before mixes two joint offsets or gravity models.

## One iteration

1. Design a trajectory:
   `python -m cable_ident design --arm follower --out trajectories/round1.txt`
2. Run it on the robot, E-stop in hand:
   `dynamics_excitation_legacy --trajectory <traj> excitation/<name>.dat`
   (the leader: `dynamics_excitation_canopen`). It moves to the start, waits
   for Enter, plays the file under position control and logs every joint at
   500 Hz. It stops if a joint leaves its bounds or lags the reference by more
   than 0.1 rad.
3. Add the log to the arm's `excitation` list in `arms.py`, then run `cv` and
   `fit`.
4. Rebuild the package and run the teleop with `--model cable`.

Stop when another round no longer lowers the `cv` error.

## In the controller

With `--model cable` (`src/lib/cable_drive_dynamics.h`):

* the external-torque estimate subtracts the whole model;
* the feedforward adds `--ff-friction` of the friction and `--ff-inertia` of
  the inertia. The key `m` changes both live. Compensating all of the friction
  can drive a joint by itself, so raise the share in steps and keep it at 0.5
  or below.

`test/test_cable_drive_model.cpp` checks the C++ model against this package
sample by sample, and the elements' passivity. Run `testvectors` after a new
fit.

## References

* P. M. Wensing, S. Kim, J.-J. E. Slotine. Linear matrix inequalities for
  physically consistent inertial parameter identification: a statistical
  perspective on the mass distribution. IEEE RA-L, 2018.
* T. Lee, P. M. Wensing, F. C. Park. Geometric robot dynamic identification:
  a convex programming approach. IEEE T-RO, 2020.
* C. D. Sousa, R. Cortesão. Physical feasibility of robot base inertial
  parameter identification: a linear matrix inequality approach. IJRR, 2014.
