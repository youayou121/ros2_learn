import launch
import launch_ros


def generate_launch_description():
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument('launch_background_g', default_value='0')
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{'background_g': launch.substitutions.LaunchConfiguration('launch_background_g', default='0')}],
        # parameters=[{'background_g': 255}],
        output = 'screen'
    )
    action_node_turtle_control_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_patrol',
        output = 'log'
    )
    action_node_patrol_client_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        output = 'both'
    )
    return launch.LaunchDescription([
        action_declare_arg_background_g,
        action_node_turtlesim_node,
        action_node_turtle_control_node,
        action_node_patrol_client_node
    ])