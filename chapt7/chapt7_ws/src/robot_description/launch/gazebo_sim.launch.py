import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    pkg_name = 'robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'first_robot.urdf')
    default_xacro_path = os.path.join(pkg_share, 'xacro', 'tracer', 'tracer.xacro')
    pkg_path = pkg_share.split('/install/')[0]
    default_rviz_config_path = os.path.join(pkg_path,'src', pkg_name, 'config', 'gazebo.rviz')
    default_world_path = os.path.join(pkg_path,'src', pkg_name, 'worlds', 'my_room_wall.world')
    action_declare_model_path_xacro = launch.actions.DeclareLaunchArgument(
        name = 'model', default_value = default_xacro_path
        )
    cmd_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(cmd_result, value_type = str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    action_rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    action_spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', '/robot_description', '-entity', 'my_robot']
    )
    return launch.LaunchDescription([
        action_launch_gazebo,
        action_declare_model_path_xacro,
        action_robot_state_publisher,
        # joint_state_publisher,
        # action_rviz2,
        action_spawn_entity
    ])