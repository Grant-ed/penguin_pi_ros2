import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_share_path = get_package_share_directory("ppi_config")
    ekf_config_path = join(config_share_path, "config", "ekf.yaml")
    default_model_path = join(
        config_share_path, "urdf", "penguin_pi.urdf"
    )
    default_rviz_config_path = join(
        config_share_path, "rviz", "default.rviz"
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[ekf_config_path]
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    microcontroller_node = launch_ros.actions.Node(
        package='ppi_controller',
        executable='microcontroller_node',
        name='microcontroller_node',
        output='screen',
        parameters=[ekf_config_path]
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )
    encoder_twist_translator_node = launch_ros.actions.Node(
        package='localiser',
        executable='encoder_to_twist_node',
        name='encoder_to_twist_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        microcontroller_node,
        robot_localization_node,
        encoder_twist_translator_node,
        rviz_node
    ])
