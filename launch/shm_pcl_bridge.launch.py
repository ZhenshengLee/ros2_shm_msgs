import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package='shm_msgs', executable='shm_pcl2m_bridge', output='screen',
            remappings=[
                ('shm_pc_input', 'shm_pc_2m'),
                ('sensor_pc_output', 'sensor_pc_2m')
            ],
            parameters=[
            ]
        ),
    ])
