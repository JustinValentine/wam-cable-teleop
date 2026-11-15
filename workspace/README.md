# Files that live beside the package

The repository root is the `wam_teleop` ROS package. This folder holds the
files that sit around it in the Docker workspace on the robot computer.
`<workspace>` is that checkout.

| Here | Goes to | What it is |
| --- | --- | --- |
| `wam` | `<workspace>/wam` | the launcher: starts the arm programs in tmux panes, checks the CAN interfaces, runs the calibrations, summarises recordings |
| `wam_settings.sh` | `<workspace>/wam_settings.sh` | the launcher's settings |
| `config/wam8g/` | `<workspace>/config/wam8g/` | the leader's libbarrett configuration and calibration |
| `tools/ft_cal_check/` | anywhere | prints how well the `wam ft-cal` poses spread gravity over the tool frame |
| `tools/gravtool/` | anywhere | evaluates libbarrett's gravity model at given joint positions |

The package itself goes to `<workspace>/wam_ws/src/wam_teleop/`.

Not in this repository: the Docker files and build scripts of the workspace,
the libbarrett builds, libgripper, and recordings (`teleop_logs/`,
`excitation/`).
