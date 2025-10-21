import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    multisim_launch_path = [get_package_share_directory('turtlesim')+'/launch' + '/multisim.launch.py']
    # 1 include
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )
    # 2 log info
    action_log_info = launch.actions.LogInfo(msg=['Including multisim launch from: ', str(multisim_launch_path)])
    # 3 execute process
    action_topic_list = launch.actions.ExecuteProcess(
        cmd = ['ros2', 'topic', 'list'],
        output = 'log'
    )
    # 4 action group
    action_group = launch.actions.GroupAction(
        [
            launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
            launch.actions.TimerAction(period=4.0, actions=[action_log_info]),
            launch.actions.TimerAction(period=6.0, actions=[action_topic_list])
        ]
    )
    # 5 rqt
    action_declare_start_rqt = launch.actions.DeclareLaunchArgument(
        'start_rqt', default_value='False'
    )
    start_rqt = launch.substitutions.LaunchConfiguration('start_rqt', default='False')
    action_rqt = launch.actions.ExecuteProcess(
        condition=launch.conditions.IfCondition(start_rqt),
        cmd = ['rqt'],
    )
    return launch.LaunchDescription(
        [
            action_log_info,
            action_group,
            action_rqt,
        ]
    )
