import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    move = launch_ros.actions.Node(
        package='fishbot_application',
        executable='move',
        output='screen'
    )
    collect_data = launch_ros.actions.Node(
        package='fishbot_application',
        executable='collect_data',
        output='screen'
    )
    collect_data_gazebo= launch_ros.actions.Node(
        package='fishbot_application',
        executable='collect_gazebo_data',
        output= 'screen'

    )


    launch_dlo = launch.actions.ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'cd /home/tangrui/ros2_ws && '
            'colcon build  && '
            'source install/setup.bash && '
            'ros2 launch direct_lidar_odometry dlo.launch.py'
        ],
        output='log'
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'fishbot_description',
            'gazebo_sim.launch.py'
        ],
        output='log'
    )



    delayed_dlo = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_dlo]
    )

    # 8秒后机器人运动
    delayed_move = launch.actions.TimerAction(
        period=8.0,
        actions=[move]
    )

    # 10秒后开始采集
    delayed_collect = launch.actions.TimerAction(
        period=15.0,
        actions=[collect_data]
    )

    delayed_collect_gazebo = launch.actions.TimerAction(
        period=15.0,
        actions=[collect_data_gazebo]
    )

    return launch.LaunchDescription([
        launch_gazebo,
        delayed_dlo,
        delayed_move,       
        delayed_collect,
        delayed_collect_gazebo 
    ])




# IncludeLaunchDescription
# = “把另一个 launch 文件并入我这个 launch 里运行”

# ExecuteProcess
# = “像在终端里敲一条命令一样，启动一个外部程序”