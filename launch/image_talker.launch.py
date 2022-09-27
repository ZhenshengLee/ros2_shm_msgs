import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
# from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker0',
            remappings=[
                ('image', 'image/image0')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker1',
            remappings=[
                ('image', 'image/image1')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker2',
            remappings=[
                ('image', 'image/image2')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker3',
            remappings=[
                ('image', 'image/image3')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker4',
            remappings=[
                ('image', 'image/image4')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker5',
            remappings=[
                ('image', 'image/image5')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker6',
            remappings=[
                ('image', 'image/image6')
            ],
            parameters=[
            ]
        ),
        launch_ros.actions.Node(
            package='shm_msgs', executable='image_talker', output='screen',
            name='image_talker7',
            remappings=[
                ('image', 'image/image7')
            ],
            parameters=[
            ]
        ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_talker', output='screen',
        #     name='image_talker8',
        #     remappings=[
        #         ('image', 'image/image8')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_talker', output='screen',
        #     name='image_talker9',
        #     remappings=[
        #         ('image', 'image/image9')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_talker', output='screen',
        #     name='image_talker10',
        #     remappings=[
        #         ('image', 'image/image10')
        #     ],
        #     parameters=[
        #     ]
        # ),
        # launch_ros.actions.Node(
        #     package='shm_msgs', executable='image_talker', output='screen',
        #     name='image_talker11',
        #     remappings=[
        #         ('image', 'image/image11')
        #     ],
        #     parameters=[
        #     ]
        # ),
    ])
