# Running the teleop

The launcher is `workspace/wam`, installed as `wam` on the robot computer. Its
settings are in `wam_settings.sh` next to it.

## Start a session

1. Both arms at home. Put the follower in Idle before starting: its
   force/torque sensor only powers up in Idle.
2. `wam teleop`
3. Read the panes before activating. The leader prints its reference mode and
   gains table; the follower prints `Force/torque sensor: read every cycle`
   and the gripper's mass if the sensor is in use.
4. Shift-Activate each arm when its pane asks.
5. In each arm's pane type `l`, Enter. The arm moves to the sync pose by
   itself. The follower then takes the sensor's zero there, so keep clear of
   the gripper and keep it empty.
6. At `Press [Enter] to link` press Enter in each pane. Both panes then say
   `Linked with remote WAM`.

## End a session

1. `wam stop` from any terminal, or `x` in each arm's pane.
2. Shift-Idle each arm when its program asks. The programs wait for that
   before they exit.
3. If one is stuck: Ctrl-C while it waits, or `wam stop --force`. A killed
   program stops controlling its arm, so Shift-Idle or E-stop that arm first.

## Commands

| Command | What it does |
| --- | --- |
| `wam teleop` | start leader, follower and, if configured, the gripper pane |
| `wam stop [--force]` | end the session |
| `wam status` | what is running |
| `wam attach` | back into the tmux session. Ctrl-b d leaves it running, Ctrl-b and an arrow changes pane, Ctrl-b z zooms one |
| `wam pp on\|off` | position-position only on both arms, and back. Fades over 1 s |
| `wam 4c off\|on` | the force channels off on both arms, and back |
| `wam ft-zero` | take the force/torque sensor's zero again. Nothing may touch the gripper for a second |
| `wam log [file ...]` | timing and link summary of the latest recording |
| `wam gripper restart\|status` | restart the Robotiq gripper, or print its state |
| `wam joytest` | show the navigation controller's buttons as they are pressed |
| `wam cal ...`, `wam ft-cal` | calibration, see [calibration.md](calibration.md) |

## Keys in the arm panes

Type the key, then Enter.

| Key | Leader pane | Follower pane |
| --- | --- | --- |
| `l` | go to the sync pose, then link (again: unlink) | the same; also takes the sensor's zero |
| `c` | force channels off / on, both arms | the same |
| `p` | position-position only on / off, both arms | the same |
| `m` | friction and inertia shares of the cable model's feedforward, e.g. `0.5 0` | the same for its own model, e.g. `0.5 0.25` |
| `w` | wrist force gain and deadband in degrees, e.g. `0.5 0` | |
| `h` | wrist load scaling gain, 0 to 1 | |
| `z` | | zero the force/torque sensor |
| `f` | J6 friction feedforward, N m | |
| `t` | tune a position gain | tune a position gain |
| `x` | exit, then Shift-Idle | exit, then Shift-Idle |

Values set by key last until the program exits.

## If something goes wrong

| Symptom | What to do |
| --- | --- |
| Buzzing, swinging, or it feels wrong | `wam pp on`. If it is violent, E-stop |
| A steady push with nothing touching | hands off the gripper, then `wam ft-zero`. The sensor drifts |
| A joint buzzes or creeps by itself | lower that arm's friction share with `m` |
| A carried weight feels too heavy at the wrist | leader pane, `h` then `1` (wrist load scaling) |
| The follower does not find the force/torque sensor | it was not in Idle when its program started. `wam stop`, Shift-Idle it, start again |
| "The leader is not active ... teleop stopped" | a fault, E-stop or Idle on one arm. The other arm holds. `wam stop`, then `wam teleop` |
| An arm drops to Idle in a fast move | velocity fault. Limits: leader 4 rad/s per joint, follower 3 m/s. Restart the session |
| A CAN interface is down | `wam teleop` brings it up and asks for the sudo password |
| The gripper does not react | hold L2. `wam gripper status` says what it is waiting for |

## What the programs do for safety

* From the first link, each program stops the teleop when its own arm leaves
  ACTIVE (`lib/safety_watch.h`). The stopped side sends nothing more, the
  other side's link times out within 20 ms, and it holds. Restart both
  programs to link again.
* Each program sets its safety module's velocity limit at start-up: 4 rad/s
  per joint on the leader, 3 m/s Cartesian on the follower.
  `--velocity-limit` changes it.
* After `x`, Ctrl-C or SIGTERM a program holds its arm and waits for
  Shift-Idle before it exits (`lib/clean_exit.h`).

## Recordings

With `RECORD=native` each arm program writes `teleop_logs/teleop_<time>_<arm>.tlog`
from its control loop, one row per cycle: joint state, the three parts of the
commanded torque, the reference used, the external torques, link timing and
the sensor's readings. A slow disk drops rows instead of delaying the loop.

```bash
wam log
python3 scripts/teleop_log.py teleop_logs/teleop_<time>_leader.tlog --npz out.npz
```

`scripts/teleop_log.py` needs only numpy. Each pane's text output is kept in
`pane_logs/<time>_<pane>.log`.

## Without the launcher

```bash
source scripts/setup_leader_wam8g.sh
rosrun wam_teleop leader_wam8g --law u4 --wrist 127.0.0.1 5555 5554
```
```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower --sync wam8g --law u4 127.0.0.1 5554 5555
```

The leader receives on the port the follower sends to, and the other way
round. `--help` lists the options of each program.
