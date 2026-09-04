#!/usr/bin/env python3
# Copyright 2026 Mitchell Solomon
# BSD-3-Clause, see include/as2_mocap_guarded/mocap_pose_guarded.hpp

"""
Launch the STOCK as2_state_estimator node with the mocap_pose_guarded plugin.

WHY THIS FILE EXISTS INSTEAD OF REUSING as2_state_estimator's launch files
-------------------------------------------------------------------------
The plugin itself is found by pluginlib without any help: the node builds its
lookup name as `<plugin_name parameter> + "::Plugin"` and its ClassLoader scans
the ament resource index `as2_state_estimator__pluginlib__plugin` across every
prefix on AMENT_PREFIX_PATH, so `plugin_name: "mocap_pose_guarded"` resolves to
`mocap_pose_guarded::Plugin` in libmocap_pose_guarded.so from this package.

The stock LAUNCH FILE, however, cannot be used, for two independent reasons --
both verified by reading
/opt/ros/humble/share/as2_state_estimator/launch/state_estimator_launch.py:

  1. `plugin_name` is declared with
         choices=get_available_plugins('as2_state_estimator')
     and as2_core.launch_plugin_utils.get_available_plugins() parses ONLY
     <as2_state_estimator share>/plugins.xml. Our plugin is declared in this
     package's plugins.xml, so `plugin_name:=mocap_pose_guarded` is rejected as
     an invalid choice before the node is ever created.

  2. The plugin's default config file path is hard-wired to
         <as2_state_estimator share>/plugins/<plugin_name>/config/plugin_default.yaml
     and as2_core.LaunchConfigurationFromConfigFile.perform() open()s that path
     unconditionally, even when the user supplies plugin_config_file:=... . For a
     plugin living in another package that path does not exist, so the launch
     fails with FileNotFoundError regardless of what is passed on the command line.

Hence: this launch file, which launches the same node (package
'as2_state_estimator', executable 'as2_state_estimator_node') with parameters
this package controls. Nothing in the installed as2_state_estimator is modified.

USAGE
-----
    ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py \
        namespace:=drone0 rigid_body_name:=drone0

    # non-identity lab frame, and a different mocap topic
    ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py \
        namespace:=drone1 rigid_body_name:=drone1 \
        mocap_topic:=/mocap/rigid_bodies \
        earth_to_map_x:=1.0 earth_to_map_yaw:=0.0
"""

__authors__ = 'Mitchell Solomon'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLUGIN_NAME = 'mocap_pose_guarded'

# Launch arguments that map straight onto plugin parameters. An empty value means
# "leave whatever the config file says"; anything else overrides it. The second
# element is the coercion applied before the value reaches the node, because
# launch arguments are always strings and rclcpp will reject a string where the
# plugin asks for a double.
_PARAM_ARGS = (
    ('rigid_body_name', str, 'Rigid body name in Motive. Must match EXACTLY.'),
    ('mocap_topic', str, 'mocap4r2_msgs/RigidBodies topic to subscribe to.'),
    ('mocap_qos_reliability', str,
     'Subscription reliability: best_effort | reliable | system_default.'),
    ('earth_to_map_x', float, 'earth->map translation x [m].'),
    ('earth_to_map_y', float, 'earth->map translation y [m].'),
    ('earth_to_map_z', float, 'earth->map translation z [m].'),
    ('earth_to_map_yaw', float, 'earth->map yaw [rad].'),
    ('twist_smooth_filter_cte', float, 'Velocity smoother alpha in (0, 1]. 1 = off.'),
    ('quaternion_tolerance', float, 'Max |norm(q) - 1| before a sample is dropped.'),
    ('mocap_timeout', float, 'Seconds without a good sample before tracked=false.'),
    ('mocap_health_rate', float, 'mocap_health publication rate [Hz].'),
)


def launch_setup(context, *args, **kwargs):
    """Build the node with only the overrides the user actually supplied."""
    config_file = LaunchConfiguration('config_file').perform(context)
    plugin_config_file = LaunchConfiguration('plugin_config_file').perform(context)

    for path, label in ((config_file, 'config_file'),
                        (plugin_config_file, 'plugin_config_file')):
        if not os.path.isfile(path):
            raise RuntimeError(f'{label} does not exist: {path}')

    # plugin_name is set here, not in a yaml, so it can never be silently changed
    # to the stock `mocap_pose` by an inherited config file.
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        'plugin_name': PLUGIN_NAME,
    }

    for name, caster, _ in _PARAM_ARGS:
        raw = LaunchConfiguration(name).perform(context)
        if raw == '':
            continue
        overrides[name] = caster(raw)

    return [
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            # Order matters: later entries win, so CLI overrides beat the yamls.
            parameters=[config_file, plugin_config_file, overrides],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Entry point for launch file."""
    pkg_share = get_package_share_directory('as2_mocap_guarded')

    args = [
        DeclareLaunchArgument(
            'namespace', default_value='drone0',
            description='Drone namespace. mocap_health lands on /<namespace>/mocap_health.'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Logging level'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'state_estimator_default.yaml'),
            description='Frame-name configuration file for the state estimator node.'),
        DeclareLaunchArgument(
            'plugin_config_file',
            default_value=os.path.join(
                pkg_share, 'plugins', PLUGIN_NAME, 'config', 'plugin_default.yaml'),
            description='Configuration file for the mocap_pose_guarded plugin.'),
    ]
    args += [
        DeclareLaunchArgument(name, default_value='', description=desc)
        for name, _, desc in _PARAM_ARGS
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
