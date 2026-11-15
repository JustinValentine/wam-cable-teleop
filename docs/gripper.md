# Grippers

Both grippers are worked from a Sony navigation controller. Only one can be
used at a time.

## Robotiq 2F

`robotiq_teleop` owns the gripper's serial port (Modbus RTU through
libgripper) and reads the controller. The arm programs know nothing about it.
With `GRIPPER=robotiq` in `wam_settings.sh`, `wam teleop` starts it in its own
pane.

| Controller | Gripper |
| --- | --- |
| L2 + stick up / down | closes / opens, faster with more stick. Let go and it holds |
| L2 + cross | opens fully |
| L2 + circle, held 1 s | restarts the gripper |
| off or out of range | the gripper stops after 0.5 s and keeps its grip |

The gripper takes positions, not velocities. The stick moves a target opening,
which never gets more than `lead` ahead of the measured opening. Closed on an
object the request therefore stays a little beyond the contact, the gripper
keeps its force on, and it opens at once when the stick goes down.

Settings: `GRIPPER`, `GRIPPER_PORT` and `GRIPPER_ARGS` in `wam_settings.sh`;
speeds, force, lead, buttons and the Modbus address in
`config/robotiq_nav_controller.yaml`.

### Restarting it

Hold L2 + circle for a second, run `wam gripper restart`, or call
`rosservice call /robotiq_teleop/restart`. The node closes and opens the
serial port, resets the gripper, activates it and waits until it is ready.

**The activation runs the fingers through their whole travel, so whatever the
gripper holds falls.** That is why a restart waits to be asked for. After
power-on the gripper is not activated; clear the fingers and restart it once.

Not everything needs a restart:

| Gripper | Node |
| --- | --- |
| does not answer for 0.5 s | sends nothing and reopens the port every second. When the gripper answers and is still activated it carries on; nothing moves |
| answers but is not activated | waits for a restart |
| reports a fault that needs a reset | prints the code and its meaning, waits for a restart |
| reports a fault that clears by itself | prints it and waits; suggests a restart after 5 s |
| answers out of step | reopens the port, which empties the line |

`wam gripper status` prints the node's state in one line. `wam stop` ends the
node without sending the gripper anything, so it keeps holding what it holds.

Launch arguments: `auto_activate:=true` activates the gripper at start,
`auto_restart:=true` restarts without asking whenever a restart is needed.

### Setup

1. Power the gripper and plug in its RS-485 adapter. Put the adapter's name
   from `/dev/serial/by-id/` in `GRIPPER_PORT`; `/dev/ttyUSB0` can change
   when it is plugged in again.
2. Install libgripper's branch `robotiq` and rebuild. CMake prints
   `robotiq_teleop: built`.
3. `GRIPPER=robotiq` and `HAND_CONTROLLER=no` in `wam_settings.sh`.

Without the arms:

```bash
roslaunch wam_teleop robotiq_nav_controller.launch port:=<device>
```

Do not run another program on the same serial port at the same time.

### Tests

`robotiq_teleop_tests` runs the node's logic in simulated time.
`test/robotiq_e2e.sh` runs the real node and launch file against
`test/robotiq_sim.py`, a simulated gripper on a pseudo-terminal.
`python3 test/robotiq_sim.py --link /tmp/robotiq_sim` alone gives a gripper to
try the node against.

## BarrettHand

With `--hand` the follower drives a BarrettHand on its own CAN bus.
`launch/hand_nav_controller.launch` feeds it from the controller, and
`HAND_CONTROLLER=yes` in `wam_settings.sh` starts that with the teleop.

| Controller | Hand |
| --- | --- |
| L2 + stick up / down | closes / opens the fingers |
| L2 + cross | steps the spread: 0 %, 50 % (pinch), 100 %, back to 0 % |

A spread step opens the fingers first, so the hand lets go.
`--no-spread-poses` (`SPREAD_POSES=no`) keeps the spread at 0 %.
`hand_teleop_tests` runs this against a simulated bus.

## The controller

With the udev rule `99-nav_controller.rules` installed, the controller on
Bluetooth appears as `/dev/nav_controller`. `/dev/input/js0` works on a USB
cable as well.
`joy_dev:=...` in `GRIPPER_ARGS` selects the device. `wam joytest` shows the
buttons as they are pressed.
