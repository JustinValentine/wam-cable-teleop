cmake_minimum_required(VERSION 3.5)
project(wam_teleop)

# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   message_generation
)

add_message_files(
   FILES
   GravityTorque.msg
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)


## The two arms use different builds of libbarrett: the CANopen one for the
## wam8g leader, the standard one for the follower.
set(BARRETT_LEADER_PREFIX /opt/barrett-canopen CACHE PATH "CANopen libbarrett (leader)")
set(BARRETT_FOLLOWER_PREFIX /opt/barrett CACHE PATH "libbarrett (follower)")
include_directories(${catkin_INCLUDE_DIRS})

# link_arm_library(target prefix [WAM8G]): pass WAM8G for programs written for
# the eight-joint wam8g.
function(link_arm_library target prefix)
   if(NOT EXISTS "${prefix}/share/barrett/barrett-config.cmake" OR
      NOT EXISTS "${prefix}/lib/libbarrett.so")
      message(FATAL_ERROR "Missing libbarrett installation for ${target}: ${prefix}")
   endif()
   include("${prefix}/share/barrett/barrett-config.cmake")
   target_include_directories(${target} BEFORE PRIVATE ${BARRETT_INCLUDE_DIRS})
   target_compile_options(${target} PRIVATE ${BARRETT_DEFINITIONS})
   if("${prefix}" STREQUAL "${BARRETT_LEADER_PREFIX}" AND NOT "WAM8G" IN_LIST ARGN)
      # No wam8g joint mapping in this program: refuse that arm at start-up.
      target_compile_definitions(${target} PRIVATE BARRETT_SMF_NO_WAM8G)
   endif()
   list(REMOVE_ITEM BARRETT_LIBRARIES barrett)
   target_link_libraries(${target} "${prefix}/lib/libbarrett.so"
      ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} Boost::system Boost::thread)
   set_target_properties(${target} PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "${prefix}/lib;/usr/local/lib"
      INSTALL_RPATH_USE_LINK_PATH TRUE)
   message(STATUS "${target}: libbarrett from ${prefix}")
endfunction()

find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(follower src/follower.cpp src/lib/udp_handler.cpp)
link_arm_library(follower "${BARRETT_FOLLOWER_PREFIX}")
add_dependencies(follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## follower --hand drives a BarrettHand on the follower's bus from ROS topics
## (launch/hand_nav_controller.launch). Needs gripper_ros_common from libgripper.
find_package(gripper_ros_common QUIET)
if(gripper_ros_common_FOUND)
   target_include_directories(follower PRIVATE ${gripper_ros_common_INCLUDE_DIRS})
   target_compile_definitions(follower PRIVATE FOLLOWER_HAND)
   add_dependencies(follower ${gripper_ros_common_EXPORTED_TARGETS})
   message(STATUS "follower: --hand available")
else()
   message(STATUS "follower: no gripper_ros_common, built without --hand")
endif()

## Leader: the eight-joint wam8g (pair it with follower --sync wam8g).
add_executable(leader_wam8g src/leader_wam8g.cpp src/lib/udp_handler.cpp)
link_arm_library(leader_wam8g "${BARRETT_LEADER_PREFIX}" WAM8G)
target_compile_definitions(leader_wam8g PRIVATE LEADER_WAM8G)
add_dependencies(leader_wam8g ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Excitation trajectories for the dynamics fit, one build per library.
add_executable(dynamics_excitation_canopen src/dynamics_excitation.cpp)
link_arm_library(dynamics_excitation_canopen "${BARRETT_LEADER_PREFIX}" WAM8G)
target_compile_definitions(dynamics_excitation_canopen PRIVATE EXCITATION_CANOPEN)

## Measures the wam8g's J6 breakaway friction (for leader_wam8g --j6-friction).
add_executable(wam8g_j6_friction src/wam8g_j6_friction.cpp)
link_arm_library(wam8g_j6_friction "${BARRETT_LEADER_PREFIX}" WAM8G)

add_executable(dynamics_excitation_legacy src/dynamics_excitation.cpp)
link_arm_library(dynamics_excitation_legacy "${BARRETT_FOLLOWER_PREFIX}")

## Offline tests. None of them needs a robot.
find_package(Eigen3 REQUIRED)

## The C++ cable-drive model against the Python identification (test/data).
add_executable(cable_model_tests test/test_cable_drive_model.cpp)
target_include_directories(cable_model_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)
target_compile_definitions(cable_model_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

## lib/arm_gravity.h against the identification's gravity model:
## arm_gravity_tests <wam8g.conf> <wam7w.conf>
add_executable(arm_gravity_tests test/test_arm_gravity.cpp)
link_arm_library(arm_gravity_tests "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(arm_gravity_tests PRIVATE src/lib)
target_compile_definitions(arm_gravity_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

## The UDP link, the recorder, the estimator arithmetic and the control
## thread's setup, without libbarrett.
add_executable(low_latency_tests test/test_low_latency.cpp)
target_include_directories(low_latency_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)
target_link_libraries(low_latency_tests pthread)

add_executable(wam8g_teleop_tests test/test_wam8g_teleop.cpp)
target_include_directories(wam8g_teleop_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)

## lib/hand_reflect.h: the wrist load scaling.
add_executable(hand_reflect_tests test/test_hand_reflect.cpp)
target_include_directories(hand_reflect_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)

add_executable(wam8g_loopback_tests test/test_wam8g_loopback.cpp src/lib/udp_handler.cpp)
link_arm_library(wam8g_loopback_tests "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(wam8g_loopback_tests PRIVATE src/lib)
# The same tests against the follower's library.
add_executable(wam8g_loopback_tests_legacy test/test_wam8g_loopback.cpp src/lib/udp_handler.cpp)
link_arm_library(wam8g_loopback_tests_legacy "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(wam8g_loopback_tests_legacy PRIVATE src/lib)

## lib/clean_exit.h: shutdown on a signal or closed input.
add_executable(clean_exit_tests test/test_clean_exit.cpp)
target_include_directories(clean_exit_tests PRIVATE src/lib)
target_link_libraries(clean_exit_tests pthread)

## lib/teleop_mode.h between two processes. With roscore running:
## `teleop_mode_tests listen 6 &` then `teleop_mode_tests send pp`.
add_executable(teleop_mode_tests test/test_teleop_mode.cpp)
target_include_directories(teleop_mode_tests PRIVATE src/lib)
target_link_libraries(teleop_mode_tests ${catkin_LIBRARIES})

## follower --hand on a simulated CAN bus, next to the arm's 500 Hz traffic.
add_executable(hand_teleop_tests test/test_hand_teleop.cpp)
link_arm_library(hand_teleop_tests "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(hand_teleop_tests PRIVATE src/lib)

## lib/ft_barrett.h: non-blocking force/torque sensor reads on the simulated bus.
add_executable(ft_bus_tests test/test_ft_bus.cpp)
link_arm_library(ft_bus_tests "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(ft_bus_tests PRIVATE src/lib)

## End to end for the hand: start roscore and launch/hand_nav_controller.launch,
## then run hand_chain_tests (it plays the joystick).
if(gripper_ros_common_FOUND)
   add_executable(hand_chain_tests test/test_hand_chain.cpp)
   link_arm_library(hand_chain_tests "${BARRETT_FOLLOWER_PREFIX}")
   target_include_directories(hand_chain_tests PRIVATE src/lib ${gripper_ros_common_INCLUDE_DIRS})
   add_dependencies(hand_chain_tests ${gripper_ros_common_EXPORTED_TARGETS})
endif()

## Robotiq 2F gripper. The decision logic (lib/robotiq_teleop_core.h) has no
## dependencies, so its tests are always built.
add_executable(robotiq_teleop_tests test/test_robotiq_teleop.cpp)
target_include_directories(robotiq_teleop_tests PRIVATE src/lib)

## The robotiq_teleop node needs gripper::robotiq::RobotiqGripper from
## libgripper (github.com/dmiller12/libgripper, branch "robotiq"). Without it
## the node is skipped. The check is repeated at every configure.
find_package(gripper QUIET)
find_package(std_srvs QUIET)
unset(GRIPPER_HAS_ROBOTIQ CACHE)
if(gripper_FOUND AND std_srvs_FOUND)
   include(CheckCXXSourceCompiles)
   set(CMAKE_REQUIRED_LIBRARIES gripper::gripper)
   set(CMAKE_REQUIRED_QUIET TRUE)
   check_cxx_source_compiles("
      #include <gripper/robotiq/robotiq_gripper.h>
      int main() { gripper::robotiq::RobotiqGripper g; return g.isConnected() ? 1 : 0; }"
      GRIPPER_HAS_ROBOTIQ)
   unset(CMAKE_REQUIRED_LIBRARIES)
   unset(CMAKE_REQUIRED_QUIET)
endif()
if(GRIPPER_HAS_ROBOTIQ)
   add_executable(robotiq_teleop src/robotiq_teleop.cpp)
   target_include_directories(robotiq_teleop PRIVATE ${std_srvs_INCLUDE_DIRS})
   target_link_libraries(robotiq_teleop ${catkin_LIBRARIES} ${std_srvs_LIBRARIES} gripper::gripper pthread)
   add_dependencies(robotiq_teleop ${catkin_EXPORTED_TARGETS})
   message(STATUS "robotiq_teleop: built (libgripper with the Robotiq classes)")
elseif(NOT gripper_FOUND)
   message(STATUS "robotiq_teleop: not built (no libgripper installed)")
elseif(NOT std_srvs_FOUND)
   message(STATUS "robotiq_teleop: not built (no std_srvs)")
else()
   message(STATUS "robotiq_teleop: not built (the installed libgripper has no gripper::robotiq::RobotiqGripper: its branch robotiq has)")
endif()
