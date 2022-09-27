import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
# from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener0',
            remappings=[
                ('image', 'image/image0')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener1',
            remappings=[
                ('image', 'image/image1')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener2',
            remappings=[
                ('image', 'image/image2')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener3',
            remappings=[
                ('image', 'image/image3')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener4',
            remappings=[
                ('image', 'image/image4')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener5',
            remappings=[
                ('image', 'image/image5')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener6',
            remappings=[
                ('image', 'image/image6')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_listener', output='screen',
            name='image_listener7',
            remappings=[
                ('image', 'image/image7')
            ],
            parameters=[
            ]
        ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_listener', output='screen',
        #     name='image_listener8',
        #     remappings=[
        #         ('image', 'image/image8')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_listener', output='screen',
        #     name='image_listener9',
        #     remappings=[
        #         ('image', 'image/image9')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_listener', output='screen',
        #     name='image_listener10',
        #     remappings=[
        #         ('image', 'image/image10')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_listener', output='screen',
        #     name='image_listener11',
        #     remappings=[
        #         ('image', 'image/image11')
        #     ],
        #     parameters=[
        #     ]
        # ),
    ])
