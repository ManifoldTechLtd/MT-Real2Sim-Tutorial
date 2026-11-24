import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sim_bridge_ros2')
    default_cfg = os.path.join(pkg_share, 'config', 'go2_sim.yaml')

    cfg_arg = DeclareLaunchArgument('bridge_config', default_value=default_cfg,
                                    description='sim_bridge 参数 yaml（帧名/静态TF偏移/TCP）')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Stamp TF with the Isaac /clock (required by ONS2)')
    color_lut_arg = DeclareLaunchArgument(
        'color_lut_path', default_value='',
        description='Optional voxel RGB LUT (.npz) for colored PointCloud2')

    def _make_node(context, *args, **kwargs):
        cfg_path = LaunchConfiguration('bridge_config').perform(context)
        with open(cfg_path, 'r') as f:
            params = yaml.safe_load(f) or {}
        return Node(
            package='sim_bridge_ros2',
            executable='bridge_node',
            name='sim_bridge',
            output='screen',
            parameters=[params,
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'color_lut_path': LaunchConfiguration('color_lut_path')}],
        )

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        cfg_arg, use_sim_time_arg, color_lut_arg,
        OpaqueFunction(function=lambda ctx: [_make_node(ctx)])])
