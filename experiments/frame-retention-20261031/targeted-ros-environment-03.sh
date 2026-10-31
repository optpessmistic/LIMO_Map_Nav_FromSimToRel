#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RUN_OBSERVATION_ROS_TEST=1
export PYTHONPATH="$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B outputs/setup/run-frame-retention-check.py frame-retention-targeted-ros-03 -v \
  test_mission_inspection_ros.RealMissionInspectionTests.test_second_goal_moving_timeout_does_not_reuse_first_goal_evidence \
  test_observation_ros.ObservationActionDDSTests.test_paused_clock_stage_deadline_waits_for_owned_terminal_before_return \
  test_observation_ros.ObservationActionDDSTests.test_two_headings_use_real_action_uuid_and_new_dds_windows_then_return
