#!/usr/bin/env bash
# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
# Source this in every terminal that should see the robot / simulator topics
# (ros2 topic echo, rviz, rosbag, your own nodes):
#
#   source $(ros2 pkg prefix go2_policy_runner)/share/go2_policy_runner/scripts/dds_env.sh lo        # unitree_mujoco
#   source $(ros2 pkg prefix go2_policy_runner)/share/go2_policy_runner/scripts/dds_env.sh enp3s0    # real Go2
#
# Arguments: <network_interface|auto> [domain_id=0]

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced: source $0 <network_interface> [domain_id]" >&2
  exit 1
fi

_iface="${1:-lo}"
_domain="${2:-0}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${_domain}"
if [[ "${_iface}" == "auto" ]]; then
  unset CYCLONEDDS_URI
else
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${_iface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi
echo "DDS: rmw_cyclonedds_cpp, interface=${_iface}, ROS_DOMAIN_ID=${_domain}"
unset _iface _domain
