# Calibrating the follower

Run these in this order. Each one depends on the ones before it, so do all of
them or none: a new zero with an old gravity calibration is worse than before.

| Command | What runs |
| --- | --- |
| `wam cal check` | prints where the calibrations read and write and whether the programs load. Starts nothing |
| `wam cal zero` | Barrett's `bt-wam-zerocal`. The arm moves to two poses. Each joint is turned with the arrow keys until its mark lines up; Enter records it. Writes `zerocal.conf` |
| `wam cal gravity` | Barrett's `bt-wam-gravitycal`. Nine poses around the base, holding torques measured in each. Writes `gravitycal.conf` |
| `wam ft-cal` | the gripper's weight for the force/torque sensor. 15 wrist poses, about a minute and a half |
| `wam cal excite NAME` | one 60 s excitation run for the dynamics fit, then straight up and home. Writes `excitation/slax_<date>_NAME.dat` |

Run the excitation several times: once to warm the arm up, since friction is
higher when it is cold, then at least three runs to keep. The trajectory is
the same each time. The fit uses all of them together, and the difference
between runs shows how repeatable the torque is.

Each command starts from home with the follower in Idle and asks for
Shift-Activate. The arm moves by itself in all of them: clear the space and
keep the E-stop in hand.

## Notes

* Barrett's utilities look for their files under `~/.barrett`, not under
  `BARRETT_CONFIG_FILE`. The launcher runs them with a `HOME` whose `.barrett`
  links to the follower's configuration folder, so they read and write the
  follower's files.
* `wam cal zero` runs without gravity compensation, so the arm sags a little
  in the first pose. The recorded angle is the measured one, so joints with
  scribe marks are not affected. For J6 line the tool plate up square to the
  forearm, not level to the room.
* After a zero calibration the arm must be put at home within a few degrees
  per joint at every power-up.
* Redo `wam ft-cal` whenever the gripper changes.
* After a zero or gravity calibration, earlier excitation data no longer
  matches the arm. Record new runs and fit again (`ident/README.md`).

## The leader

The leader's calibrations are the same two Barrett utilities from the CANopen
library, run after `source scripts/setup_leader_wam8g.sh`. Its excitation is
`dynamics_excitation_canopen`:

```bash
source scripts/setup_leader_wam8g.sh
rosrun wam_teleop dynamics_excitation_canopen excitation/wam8g_1.dat
```

The program raises the safety board's velocity limits for the run, holds the
wrist at its start angles, and at the end moves the arm straight up and then
home. If a joint leaves its bounds the run stops; reduce the amplitude factors
in `FourierTrajectory` in that case.
