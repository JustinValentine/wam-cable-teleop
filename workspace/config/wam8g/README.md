# wam8g configuration

libbarrett configuration and calibration of the wam8g leader: a 4-DOF WAM base,
a passive J5 and a three-joint wrist on CANopen pucks.

Programs select it with `BARRETT_CONFIG_FILE=<workspace>/config/wam8g/default.conf`;
`scripts/setup_leader_wam8g.sh` in the package sets that. `bus.port` in
`default.conf` is the number of the CAN interface the leader is on (1 = `can1`).

`calibration_data/wam8g/` holds the zero and gravity calibrations that
`bt-wam-zerocal` and `bt-wam-gravitycal` write.
