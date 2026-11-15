# Settings for ./wam. The next `wam teleop` reads them; a running session is
# not affected.

# The Docker container the arm programs run in.
CONTAINER=wam_ros_4c

# J6 friction feedforward on the wam8g, N m: about half the breakaway torque
# that wam8g_j6_friction measures.
J6_FRICTION=0.20

# Options of the two arm programs (docs/controllers.md). Force gain 0.5 grew
# an elbow oscillation in testing; 0.3 is the conservative value.
LEADER_ARGS="--law u4 --force-gain 0.4 --force-notch 35 --wrist --j6-friction $J6_FRICTION"
FOLLOWER_ARGS="--sync wam8g --law u4 --force-gain 0.4 --force-notch 35"
FOLLOWER_CONF=/home/user/wam_ros/wam_ws/src/wam_teleop/config/follower-slax-skid-ft/follower.conf

# BarrettHand on the follower, from the navigation controller.
HAND_CONTROLLER=no
SPREAD_POSES=yes      # L2 + X steps the spread 0/50/100%; no: grasp only

# Robotiq 2F gripper from the navigation controller (docs/gripper.md):
# no | robotiq. Not together with HAND_CONTROLLER=yes: there is one controller.
GRIPPER=robotiq
# Its serial adapter as the container sees it. A name under /dev/serial/by-id/
# survives re-plugging; /dev/ttyUSB0 can come back as /dev/ttyUSB1.
GRIPPER_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
# More arguments for launch/robotiq_nav_controller.launch, for example
# auto_activate:=true or auto_restart:=true. joy_dev is the joystick device:
# js0 is the controller whether it is on USB or on Bluetooth.
GRIPPER_ARGS="joy_dev:=/dev/input/js0"

# Recording, in teleop_logs/:
#   native  each arm program writes teleop_<time>_<arm>.tlog from its control
#           loop, one row per cycle, without ROS (`wam log`, scripts/teleop_log.py)
#   bag     a rosbag of the joint states
#   both, no
RECORD=native

# CPU each arm's control thread is pinned to (empty: not pinned). Use two
# different physical cores.
LEADER_CPU=2
FOLLOWER_CPU=4

# CAN interface of each arm, matching bus.port in its configuration.
FOLLOWER_CAN=can0
LEADER_CAN=can1
# Transmit queue length `wam teleop` sets on each interface. Linux's default
# of 10 frames is too short for a PCI/PCIe card.
CAN_TXQUEUELEN=100

# The gripper's weight for the follower's force/torque sensor; `wam ft-cal`
# writes it.
FT_TOOL=wam_ws/src/wam_teleop/config/follower-slax-skid-ft/ft_tool.conf
FT_TOOL_IN=/home/user/wam_ros/$FT_TOOL   # the same file inside the container
FT_ORIGIN_Z=""   # m from the tool plate to the sensor's measuring origin, if known

# Extra options per arm:
#   --model cable        the cable-drive model; --ff-friction and --ff-inertia
#                        set how much of its friction and inertia is fed
#                        forward (key m changes them live; keep friction <= 0.5)
#   --reference age      advance the other arm's position by each packet's age
#   --ki-scale 0         no integral action in the position coupling (J1-J4)
#   --ft log|arm|wrist|all   follower: use the force/torque sensor
#                        (docs/force-torque-sensor.md)
#   --wrist-force G      leader: gain on the sensor's wrist torques (key w)
LEADER_EXTRA="--model cable --ff-friction 0.2 --ki-scale 0 --reference age"
FOLLOWER_EXTRA="--ft all --ft-tool $FT_TOOL_IN --ki-scale 0 --reference age --model cable --ff-friction 0.4 --ff-inertia 0.25"
