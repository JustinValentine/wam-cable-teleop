# Controllers

What runs on each arm. File names are relative to `src/`.

Both programs run one control cycle every 2 ms and exchange one UDP packet per
cycle (`lib/teleop_link.h`): joint positions, joint velocities, the external
torque, the sample time and a few flags.

## 1. The torque on each arm

    tau = tau_gravity + tau_PID + u

| Term | What it is | Where |
| --- | --- | --- |
| `tau_gravity` | gravity compensation from the arm's gravity calibration | libbarrett |
| `tau_PID` | joint-space PID that makes this arm's joints follow the other arm's: the position coupling | libbarrett's `jpController`; reference and gains are set here (section 2) |
| `u` | the law's feedforward: force channel and dynamics compensation | `lib/teleop_law.h`, `lib/leader_wam8g.h`, `lib/follower.h` |

There is no task-space or null-space control. Every joint of one arm is coupled
to the matching joint of the other. The only Jacobian used is the follower's,
to turn the sensor's wrench into joint torques.

## 2. Position coupling

One PID per joint:

    e    = q_ref - q
    I   += ki * T * e_previous            (limited)
    tau  = kp * e + I + kd * (e - e_previous) / T     (limited)

`q_ref` is the other arm's joint position from the latest packet.

* `--reference age`: `q_ref = q_other + age * dq_other`, where `age` is the
  time since the other arm sampled its joints, at most 6 ms
  (`teleop_law::agedReference`). Without it the reference is held and the
  coupling drags by `kp * age` per rad/s.
* The wrists are mapped joint to joint relative to the sync pose
  (`lib/wam8g_wrist_map.h`): wam8g J6 to follower J5, J7 to J6 (reversed), J8
  to J7. The wam8g's J5 is a passive pin and is never commanded.
* If no packet arrives for 20 ms the link is dropped and each arm holds.

Gains (N m/rad, N m s/rad). The integral gains are zero with `--ki-scale 0`
and `ki = 0` in the follower's configuration.

| Leader (wam8g) | J1 | J2 | J3 | J4 | J6 | J7 | J8 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kp | 750 | 1000 | 400 | 200 | 20 | 10 | 10 |
| kd | 10 | 8 | 3.3 | 1.2 | 0.1 | 0.1 | 0.1 |
| limit (N m) | 50 | 80 | 50 | 30 | 5 | 5 | 5 |

| Follower | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kp | 750 | 1000 | 400 | 200 | 20 | 10 | 8 |
| kd | 10 | 8 | 3.3 | 0.8 | 0.5 | 0.5 | 0.05 |
| limit (N m) | 50 | 80 | 50 | 30 | 5 | 5 | 5 |

The leader's J1-J4 gains are compiled in (`lib/wam8g_teleop.h`). Its wrist
gains come from `workspace/config/wam8g/wam8g.conf`, the follower's from
`config/follower-slax-skid-ft/wam7w.conf`.

## 3. The law, J1-J4

`teleop_law::compute`:

| Law | `u` | |
| --- | --- | --- |
| `u1` | 0 | position-position |
| `u2` | dyn - grav | plus dynamics compensation |
| `u4` | -g ext_other + dyn - grav | four-channel: plus the force channels |
| `u6` | u4 - 0.15 (ext_other + ext_own) | four-channel with local force feedback |

* `g` is `--force-gain` (0.5 if not given; four comma-separated values set
  J1-J4 separately).
* `ext_other` is the other arm's external torque from the packet. It fades in
  over 2 s after both arms are linked. `--force-notch 35` notches it at 35 Hz
  (Q 0.7); without it the pair rings at 30-43 Hz.
* `ext` is defined as commanded torque minus model torque, which is minus the
  torque from outside. So `-g * ext_other` pushes this arm the way the other
  arm is being pushed.
* `dyn - grav` is this arm's dynamics compensation (section 6), replacing
  libbarrett's gravity torque by the fitted model's.

The mode can be switched while linked, on both arms at once
(`lib/teleop_mode.h`). Each arm fades its law's terms over 1 s.

| Mode | Key | Launcher | Law |
| --- | --- | --- | --- |
| full | `c` or `p` again | `wam 4c on`, `wam pp off` | as started |
| force channels off | `c` | `wam 4c off` | u2 |
| position-position | `p` | `wam pp on` | u1 |

## 4. External torque from the model

    ext = (u + tau_gravity + tau_PID) - tau_model(q, dq, ddq)

`lib/dynamic_external_torque.h`. The acceleration is the joint velocity through
a first-order high-pass (25 rad/s) and a low-pass (50 rad/s), so the model's
inertial torque is limited to a few hertz.

* `--model cable`: `tau_model` is the cable-drive model
  (`lib/cable_drive_model.h`): a rigid body with 22 base parameters, rotor
  inertia, viscous friction at the motors, and cable friction as
  spring-and-slider (Jenkins) elements driven by motor position, six per motor
  and six each on J2 and J3. Parameters: `lib/cable_params_wam8g.h` for the
  leader, `lib/cable_params_slax_20261112.h` or `lib/cable_params_zeus_hand.h`
  for the follower. See `ident/README.md`.
* `--model legacy` (default): `tau_model = W(q, dq, s ddq) beta`
  (`lib/estimator_systems.h`, regressor in `lib/regressor_W.h`) with 30
  parameters: gravity, Coriolis, viscous friction and a share `s` of the
  inertial torque (0 on the leader, 0.25 on the follower). No Coulomb
  friction, because its sign chatters at rest.

The leader sends its `ext` for J1-J4 to the follower.

## 5. Force/torque sensor (follower)

`lib/ft_barrett.h` reads the sensor every cycle without waiting for it;
`lib/ft_core.h` and `lib/ft_systems.h` turn the reading into joint torques:

    W      = sign * rot * (reading - reading_at_zero) - (weight_now - weight_at_zero)
    tau_ft = J_tool(q)^T W

* `weight` is the gripper's, from `ft_tool.conf` (written by `wam ft-cal`).
  The zero is taken at the link pose and again by `wam ft-zero`.
* `W` is limited to 60 N and 6 N m, to 2000 N/s and 200 N m/s, and low-passed
  at 40 Hz.
* With `--ft arm` or `--ft all` the external torque the follower sends for
  J1-J4 is `(1 - use) * ext - tau_ft`. `use` is 1 while the sensor answers and
  falls to 0 within 0.1 s if it stops, back to the model's estimate. The
  leader's force channel then carries what touches the gripper, not the
  follower's friction or model error.
* With `--ft all` the packet's three wrist slots carry `-tau_ft` for J5-J7.
  The leader applies them to its wrist (`lib/leader_wam8g.h`):

      tau_wrist = -g_w * deadzone(lowpass_15Hz(sign * ext_wrist), 0.03 N m),  at most 0.8 N m

  with the position map's signs. `g_w` is `--wrist-force` or the key `w`.

The follower itself still uses the leader's model-based `ext` in its own `u`.

### Wrist load scaling

The wrists are coupled joint to joint, so a load on the gripper reaches the
leader as a wrist torque. Two things make that torque wrong for the hand:

* The grasp point is 0.21 m from the follower's wrist centre and the handle
  0.07 m from the leader's. The same force should turn the leader's wrist with
  a third of the torque it puts on the follower's.
* The coupling does not copy the load anyway. It copies what the follower's
  position controller pushes with, and at rest the wrist's friction holds part
  of the load. In recordings the controller carried about half of a 2 to 4 N m
  load and a tenth to a quarter of a 0.3 N m one.

With `leader_wam8g --hand-reflect a` (key `h`, 0 to 1) the leader's J6 and J7
get

    c = a * lowpass_1Hz( k * tau_sensed - tau_delivered ),   at most 3 N m

`tau_sensed` is the follower's measured wrist torque mapped to the joint,
`k` is handle lever / grasp lever (`--hand-lever`, `--grasp-lever`), and
`tau_delivered` is what this joint's position controller and wrist force
channel apply. At `a = 1` the wrist settles on `k * tau_sensed`: a carried
weight is felt as its force at the handle, whatever the follower's friction
does. Above 1 Hz the coupling acts as before, so a contact starts as stiff as
it did and then settles to the scaled force (`lib/hand_reflect.h`).

It needs the follower's `--ft wrist` or `all` and fades out with the sensor.
J8 is not changed: its partner is the follower's tool roll, which has no lever
in common with it. With nothing measured, the follower's wrist drag is also
taken out of the leader's wrist below 1 Hz. Off by default.

## 6. Dynamics compensation (`dyn - grav`)

* Cable model: gravity, Coriolis, viscous friction and offsets, plus a share
  of the friction elements (`--ff-friction`) and of the inertial torque
  (`--ff-inertia`). The key `m` changes both shares live
  (`lib/cable_drive_dynamics.h`). The estimate of section 4 always uses the
  whole model. Only these shares are fed forward, because feeding all of the
  friction or a filtered inertia forward feeds the arm's own motion back. Keep
  the friction share at 0.5 or below.
* Legacy model: the model's torque as in section 4.
* Leader J6: a friction feedforward `nm * tanh(dq / 0.05 rad/s)` for its
  bearing drag (`--j6-friction nm`).

## 7. Typical options

    leader_wam8g --law u4 --force-gain 0.4 --force-notch 35 --wrist --j6-friction 0.20 \
                 --model cable --ff-friction 0.2 --ki-scale 0 --reference age
    follower     --sync wam8g --law u4 --force-gain 0.4 --force-notch 35 \
                 --ft all --ft-tool config/follower-slax-skid-ft/ft_tool.conf \
                 --ki-scale 0 --reference age --model cable --ff-friction 0.4 --ff-inertia 0.25

`workspace/wam_settings.sh` holds these and `workspace/wam` starts both
programs. `leader_wam8g --help` and `follower --help` list every option.

## 8. What comes from libbarrett

The 500 Hz loop, CAN communication, the joint PID of section 2, gravity
compensation, kinematics and the safety system. The follower uses the standard
libbarrett; the leader uses a CANopen build of libbarrett 3.0 with support for
the wam8g.

The PID step, from the library's `systems/detail/pid_controller-inl.h`:

    error = reference - feedback
    intError = saturate(intError + ki * T_s * error_1, intErrorLimit)
    controlSignal = saturate(kp * error + intError + kd * (error - error_1) / T_s, controlSignalLimit)
    error_1 = error

The programs set its reference with `wam.trackReferenceSignal(...)` and add
`u` with `connect(wamJPOutput, wam.input)`.

## 9. Limits of joint-to-joint reflection

Joint torques are copied joint for joint between two arms that differ beyond
the elbow. A force on the gripper therefore does not reach the operator as one
force at the hand, and part of it acts along the leader's elbow swivel.

The same holds for the wrist: a load at the follower's tool produces a wrist
torque of weight times the tool's length, and the operator holds that torque
on a much shorter handle, so a carried weight feels too heavy. The wrist load
scaling of section 5 corrects the wrist's share when the force/torque sensor
is in use.
