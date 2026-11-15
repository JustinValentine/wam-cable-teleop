#!/usr/bin/env python3
"""Where along the tool axis the follower's force/torque sensor sees a load, from a recording:
the value for `leader_wam8g --grasp-lever` (wrist load scaling, lib/hand_reflect.h).

  grasp_lever.py RECORDING_follower.tlog [...] [--min-force N] [--plate METRES]

The recording: any teleop session with the sensor in use (`follower --ft log|wrist|arm|all`) in
which the load is held as it will be carried, the gripper level (its axis across gravity), still
for a few seconds. Only the moments at rest with at least --min-force across the tool axis are
used.

A force F at a point p turns the wrist with the moment p x F about its centre. With p on the tool
axis z, p = lever * z, the moment across z is lever * (z x F), so
  lever = sum (z x F) . M / sum |z x F|^2
over the samples, M the sensed moment about the wrist centre.

This is the lever as the sensor sees it. How far its measuring origin lies beyond the tool plate
is not calibrated (ft_tool_cal.py --origin-z), so it differs from a ruler's by that distance. The
scaling divides the sensor's wrist torque by it, so it is the one to use.

Only numpy is needed.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_log import load  # noqa: E402


def moments(path, min_force, plate, rest_speed):
    """(z x F, M) in the tool's x and y for the usable samples, and their period."""
    log = load(path)
    if 'ft_f' not in log or 'ft_use' not in log:
        raise ValueError('%s has no force/torque sensor columns (a follower recording?)' % path)
    f, n = log['ft_f'], log['ft_n']
    c = np.stack([-f[:, 1], f[:, 0]], 1)
    m = np.stack([n[:, 0] - plate * f[:, 1], n[:, 1] + plate * f[:, 0]], 1)
    use = (log['ft_use'] > 0.99) & (np.abs(log['dq']).max(1) < rest_speed) & (np.hypot(f[:, 0], f[:, 1]) >= min_force)
    period = float(np.median(np.diff(log['t']))) if len(log['t']) > 1 else 0.0
    return c[use], m[use], period


def report(name, c, m, seconds):
    if len(c) < 100:
        print('%s: %.1f s of usable moments: too few' % (name, seconds))
        return None
    each = (c * m).sum(1) / (c ** 2).sum(1)
    lever = (c * m).sum() / (c ** 2).sum()
    left = m - lever * c
    q = np.percentile(each, [25, 50, 75])
    print('%s: %.1f s at rest, %.1f N across the tool axis (median)' % (name, seconds, np.median(np.hypot(c[:, 0], c[:, 1]))))
    print('  lever %.3f m from the wrist centre (quartiles %.3f, %.3f, %.3f)' % (lever, q[0], q[1], q[2]))
    print('  a point on the tool axis leaves %.2f of %.2f Nm rms unexplained' %
          (np.sqrt((left ** 2).sum(1).mean()), np.sqrt((m ** 2).sum(1).mean())))
    return lever


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('recordings', nargs='+', help='follower recordings')
    p.add_argument('--min-force', type=float, default=8.0, help='N across the tool axis a moment needs (default 8)')
    p.add_argument('--plate', type=float, default=0.0609,
                   help='wrist centre to the tool plate, m: the last d of the arm\'s kinematics (default 0.0609)')
    p.add_argument('--rest-speed', type=float, default=0.03, help='rad/s every joint stays below at rest (default 0.03)')
    a = p.parse_args()

    cs, ms, seconds = [], [], 0.0
    lever = None
    for path in a.recordings:
        try:
            c, m, period = moments(path, a.min_force, a.plate, a.rest_speed)
        except (OSError, ValueError) as e:
            print(e)
            return 1
        lever = report(os.path.basename(path), c, m, len(c) * period)
        cs.append(c)
        ms.append(m)
        seconds += len(c) * period
    if len(a.recordings) > 1:
        lever = report('all', np.concatenate(cs), np.concatenate(ms), seconds)
    if lever is None:
        print('Nothing to go by: hold the load level and still for a few seconds, or lower --min-force.')
        return 1
    if not 0.01 <= lever <= 0.60:
        print('A lever of %.3f m is not a point on the gripper: was the load held level and still?' % lever)
        return 1
    print('leader_wam8g --grasp-lever %.3f' % lever)
    return 0


if __name__ == '__main__':
    sys.exit(main())
