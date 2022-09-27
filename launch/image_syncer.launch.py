import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
# from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package='shm_msgs', executable='image_syncer', output='screen',
            name='image_syncer',
            remappings=[
                # ('image', 'image/image0')
            ],
            parameters=[
            ]
        )
    ])
