# wam-cable-teleop

Four-channel bilateral teleoperation between two Barrett WAM arms, with an
identified cable-drive friction model behind the force channels.

- **Leader:** the wam8g, a 4-DOF WAM base with a passive J5 and a three-joint
  haptic wrist, on CANopen pucks.
- **Follower:** a 7-DOF WAM, here with a Barrett force/torque sensor and a
  Robotiq 2F-140 gripper.

Each arm runs one program with a 500 Hz control loop, and the two programs
exchange one UDP packet per cycle. ROS starts the programs and carries the
gripper commands. It is not in the control path, and recordings are written by
the control loops themselves.

The repository is a catkin package, `wam_teleop`. It extends the WAM_Teleop
package by Amir Noohian and Dylan Miller that is described in
[Noohian et al., 2026](https://arxiv.org/abs/2607.01201).

## Layout

| Path | Contents |
| --- | --- |
| `src/leader_wam8g.cpp`, `src/follower.cpp` | the two arm programs |
| `src/lib/` | control law, UDP link, recorder, cable-drive model, sensor and gripper logic |
| `src/dynamics_excitation.cpp` | excitation trajectories for the identification |
| `ident/` | `cable_ident`: excitation design and the model fit (Python) |
| `workspace/` | the `wam` launcher, its settings and the leader's configuration |
| `config/` | follower configurations and controller mappings |
| `scripts/` | recording reader, gripper weight fit, CAN setup |
| `test/` | offline tests |
| `docs/` | controllers, running, calibration, sensor, gripper |

## Requirements

- ROS Noetic, Eigen 3, Boost, tmux, and a low-latency kernel on the host.
- Two installs of libbarrett: the CANopen build in `/opt/barrett-canopen` for
  the leader and the standard one in `/opt/barrett` for the follower. The
  paths are `BARRETT_LEADER_PREFIX` and `BARRETT_FOLLOWER_PREFIX` in
  `CMakeLists.txt`.
- Two CAN interfaces at 1 Mbit/s.
- Optional: [libgripper](https://github.com/dmiller12/libgripper), branch
  `robotiq`, for the Robotiq gripper node.
- Python 3 with numpy for `scripts/`, and `ident/requirements.txt` for the fit.

The launcher assumes a Docker workspace: the package at
`<workspace>/wam_ws/src/wam_teleop` inside the container, and the files of
`workspace/` at the top of `<workspace>`. See
[workspace/README.md](workspace/README.md).

## Build

```bash
cd <workspace>/wam_ws
catkin_make
```

CMake reports which optional parts were built (`follower: --hand available`,
`robotiq_teleop: built`).

## Run the teleop

```bash
wam teleop
```

This opens a tmux session with one pane per program. Then:

1. Shift-Activate each arm when its pane asks.
2. Type `l` and Enter in each arm's pane. The arm moves to the sync pose by
   itself.
3. Press Enter in each pane to link.

`wam stop` ends the session; Shift-Idle each arm when its program asks.
`wam pp on` drops both arms to plain position coupling at any time.

Options live in `workspace/wam_settings.sh`. Commands, keys and
troubleshooting are in [docs/running.md](docs/running.md).

## Identify the cable-drive model

1. Calibrate the arm: `wam cal zero`, `wam cal gravity`
   ([docs/calibration.md](docs/calibration.md)).
2. Record excitation runs: a warm-up run that is not used, then at least
   three. Each is the same 60 s trajectory.

   ```bash
   wam cal excite warmup
   wam cal excite 1        # writes excitation/slax_<date>_1.dat
   wam cal excite 2
   wam cal excite 3
   ```

   The leader uses `dynamics_excitation_canopen` directly.
3. List the runs in `ident/cable_ident/arms.py` and fit. All listed runs go
   into one fit, which writes the parameter header the controller compiles in:

   ```bash
   cd ident
   python -m cable_ident fit --arm slax
   ```

4. Rebuild and run with `--model cable`.

The model, the fit and the data it needs are described in
[ident/README.md](ident/README.md).

## How it works

Every cycle each arm commands

    tau = gravity compensation + joint PD toward the other arm + u

where `u = -g * ext_other + (model torque - gravity)` on J1-J4. `ext` is an
arm's external torque: the commanded torque minus the torque its dynamics model
explains, or on the follower the wrench of the force/torque sensor mapped
through the Jacobian transpose.

The cable-drive model adds what a rigid-body model lacks on a WAM: rotor
inertia and a friction that is a rate-independent hysteresis of motor
displacement, written as spring-and-slider (Jenkins) elements at the motors.
It is linear in its parameters, so the whole model is fitted in one convex
program together with physically consistent link inertias.

Details: [docs/controllers.md](docs/controllers.md).

## Tests

The tests are built with the package and need no robot:

```bash
cd <workspace>/wam_ws/devel/lib/wam_teleop
./low_latency_tests && ./wam8g_teleop_tests && ./cable_model_tests
./wam8g_loopback_tests && ./wam8g_loopback_tests_legacy
./hand_teleop_tests && ./ft_bus_tests && ./robotiq_teleop_tests
```

`test/robotiq_docker.sh` runs all of them, the gripper node against a
simulated gripper and the launcher checks in a container. It needs a local
image with both libbarrett builds.

## Safety

These programs move two robot arms. Several commands move an arm by itself:
`l`, `wam ft-cal`, and every `wam cal` command. Keep the workspace clear and
an E-stop in hand. Nothing in the launcher activates an arm; that is always
done at the pendant.

## Credits and license

Based on WAM_Teleop by Amir Noohian and Dylan Miller. The 30-parameter
rigid-body models follow the physically feasible identification of Sousa and
Cortesão.

BSD 3-Clause, see [LICENSE](LICENSE). The arm programs link against libbarrett,
which is GPL-3.0.
