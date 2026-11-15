# Force/torque sensor on the follower

Without a sensor the follower's force comes from its dynamics model, so its
friction and model error reach the leader as force. With a Barrett
force/torque sensor on the tool plate, a contact on the gripper is measured
instead. The arithmetic is in `src/lib/ft_core.h`.

## Modes

| Option | Effect |
| --- | --- |
| `follower --ft log` | reads the sensor every cycle and records it. The teleop is unchanged |
| `follower --ft arm --ft-tool F` | the external torque sent to the leader for J1-J4 is the sensor's wrench through the Jacobian transpose, with the gripper's weight removed |
| `follower --ft wrist --ft-tool F` | sends the measured wrist torques in the packet's three wrist slots |
| `follower --ft all --ft-tool F` | both |
| `leader_wam8g --wrist-force g` | gain (0 to 1) on those wrist torques at the leader's wrist. Low-passed at 15 Hz, nothing below 0.03 N m, at most 0.8 N m. Key `w` changes it live |
| `leader_wam8g --wrist-deadband deg` | the leader wrist's position coupling becomes a tether: no pull while the follower's wrist is within `deg` of the leader's |
| `follower --wrist-friction nm` | friction feedforward on the follower's wrist so that it lags less. Start at 0.1 |
| `leader_wam8g --hand-reflect a` | wrist load scaling: below 1 Hz the leader's J6 and J7 feel the measured wrist torque times handle lever / grasp lever, in place of what the position coupling delivers. Gain 0 to 1, default 0. Key `h` changes it live |

The sensor only sees what acts beyond it. A push on the forearm reaches the
leader through the position coupling alone.

## Setup

1. `wam ft-cal`, once per gripper. It writes `ft_tool.conf` in the follower's
   configuration folder.
2. `FOLLOWER_EXTRA="--ft all --ft-tool $FT_TOOL_IN"` in `wam_settings.sh`.
3. Put the follower in Idle before `wam teleop`. Its pane must print
   `Force/torque sensor: read every cycle` and the gripper's mass.
4. Link as usual. The sensor's zero is taken at the link pose, in the air.
   `wam ft-zero` takes it again at any time.

## The gripper's calibration

The sensor reads the gripper's weight too, and that changes as the wrist
turns. `--ft arm`, `wrist` and `all` therefore need the gripper's mass and
centre of mass.

`wam ft-cal` starts the follower alone. After Shift-Activate the arm goes to
the sync pose and turns its wrist through 15 poses (`src/lib/ft_cal_poses.h`),
pausing 2.5 s in each. `scripts/ft_tool_cal.py` then fits the mass, the centre
of mass, the sensor's axes in the tool frame and its sign convention from the
moments at rest. The file is written only if the fit leaves less than 1 N rms
and no pause is more than 2 N off.

Only the poses with J7 at the sync angle are fitted. Whatever crosses J7 to
the gripper (its cable) winds up as J7 turns and loads the sensor by several
newtons, which is not weight. The last poses turn J7 to show that load; it is
printed, not fitted. In teleoperation it appears as a small false force near
the ends of the wrist twist.

`wam ft-cal FILE` fits an existing recording made with `--ft log`, in which
nothing touched the gripper while the wrist was turned through its range.

## Reading the sensor

libbarrett's `ForceTorqueSensor::update()` waits for the sensor's replies with
the bus locked. `src/lib/ft_barrett.h` instead takes the replies that have
arrived and sends the next request, never waiting, so a reading is one cycle
old when used. If the sensor stops answering, its torques fade out within
0.1 s and J1-J4 go back to the model's estimate.

A reading is limited to 60 N, 6 N m, 2000 N/s and 200 N m/s and low-passed at
40 Hz (`--ft-filter`) before it becomes a torque.

## Tuning the wrist force

The wrist's force channel also carries the gripper's own inertia as the wrist
accelerates. Start `--wrist-force` at 0.2, raise it with `w`, and stop where a
tap on the leader's wrist rings.

## Carried weights

The gripper's grasp point is three times further from its wrist than the
handle is from the leader's, so a carried weight turns the leader's wrist too
hard. `--hand-reflect` (key `h`) scales the wrist's steady torque to the
handle's lever; see "Wrist load scaling" in [controllers.md](controllers.md).
Start at `h` then `0.5` and go to `1`.
