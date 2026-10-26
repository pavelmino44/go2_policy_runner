# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
"""Run the Go2 policy controller (+ teleop) against the real robot or unitree_mujoco.

Simulation and robot use exactly the same nodes and configuration. The only difference is
the DDS network interface the robot (or simulator) is reachable on:

    # unitree_mujoco on this machine (simulate/config.yaml: interface: "lo", domain_id: 0)
    ros2 launch go2_policy_runner policy_runner.launch.py network_interface:=lo \
        policy_dir:=/path/to/logs/rsl_rl/unitree_go2_velocity

    # real Go2 on the Ethernet port enp3s0 (192.168.123.x)
    ros2 launch go2_policy_runner policy_runner.launch.py network_interface:=enp3s0 \
        policy_dir:=/path/to/logs/rsl_rl/unitree_go2_velocity

Control device (input:=...):
    unitree     Unitree wireless remote, read from LowState (remote_teleop -> cmd_vel)
    dualshock4  DualShock 4 on this computer (input_teleop -> remote + cmd_vel)
    keyboard    keyboard of this computer (input_teleop -> remote + cmd_vel), F12 = on / off
    none        no teleop node; the FSM still listens to the Unitree remote, cmd_vel comes from
                elsewhere
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _dds_environment(context):
    iface = LaunchConfiguration('network_interface').perform(context).strip()
    domain = LaunchConfiguration('domain_id').perform(context).strip()
    actions = [
        # unitree_go messages are only wire-compatible with the robot through CycloneDDS
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
    ]
    if iface and iface != 'auto':
        uri = (
            '<CycloneDDS><Domain><General><Interfaces>'
            f'<NetworkInterface name="{iface}" priority="default" multicast="default"/>'
            '</Interfaces></General></Domain></CycloneDDS>'
        )
        actions.append(SetEnvironmentVariable('CYCLONEDDS_URI', uri))
    actions.append(LogInfo(msg=f'DDS: cyclonedds, interface={iface or "auto"}, domain={domain}'))
    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('go2_policy_runner'), 'config', 'go2.yaml'])

    args = [
        DeclareLaunchArgument(
            'network_interface', default_value='lo',
            description='Network interface of the robot / simulator: lo for unitree_mujoco '
                        'on this machine, e.g. enp3s0 for the real robot, auto = let DDS choose'),
        DeclareLaunchArgument(
            'domain_id', default_value='0',
            description='DDS domain (unitree robots use 0; set domain_id: 0 in '
                        'unitree_mujoco/simulate/config.yaml as well)'),
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Parameter file'),
        DeclareLaunchArgument(
            'policy_dir', default_value='',
            description='Policy directory for rl_policy modes without their own policy_dir '
                        '(<dir>/exported/policy.onnx + <dir>/params/deploy.yaml, or a '
                        'logs/rsl_rl/<experiment> folder -> latest run)'),
        DeclareLaunchArgument(
            'input', default_value='unitree',
            choices=['unitree', 'dualshock4', 'keyboard', 'none'],
            description='Control device: unitree (wireless remote), dualshock4, keyboard, '
                        'none (no teleop node)'),
        DeclareLaunchArgument(
            'input_device', default_value='',
            description='dualshock4 / keyboard: /dev/input/eventN to use ("" = find it)'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=_dds_environment),
        OpaqueFunction(function=_nodes),
    ])


def _nodes(context):
    config = LaunchConfiguration('config')
    log_level = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    device = LaunchConfiguration('input').perform(context)
    from_topic = device in ('dualshock4', 'keyboard')

    controller = Node(
        package='go2_policy_runner',
        executable='policy_controller',
        name='policy_controller',
        output='screen',
        emulate_tty=True,
        parameters=[config, {
            'default_policy_dir': LaunchConfiguration('policy_dir'),
            'remote_source': 'topic' if from_topic else 'lowstate',
        }],
        arguments=log_level,
        # give the controller time to switch the motors to damping on Ctrl-C
        sigterm_timeout='5',
    )
    nodes = [controller]

    if device == 'unitree':
        nodes.append(Node(
            package='go2_policy_runner',
            executable='remote_teleop',
            name='remote_teleop',
            output='screen',
            parameters=[config],
            arguments=log_level,
        ))
    elif from_topic:
        input_params = {'device': device}
        input_device = LaunchConfiguration('input_device').perform(context).strip()
        if input_device:
            input_params['device_path'] = input_device
        nodes.append(Node(
            package='go2_policy_runner',
            executable='input_teleop',
            name='input_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[config, input_params],
            arguments=log_level,
        ))
    return nodes
