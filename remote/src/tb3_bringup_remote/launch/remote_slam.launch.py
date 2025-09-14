import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_bringup_remote')

    slam_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    viz_params  = os.path.join(pkg_share, 'config', 'viz_params.yaml')

    # --- Relay 노드: /rgbd_image → /relay/rgbd_image ---
    relay_rgbd = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgbd',
        arguments=['/rgbd_image', '/relay/rgbd_image'],
        parameters=[{
            'qos_overrides./relay/rgbd_image.reliability': 'best_effort',
            'qos_overrides./relay/rgbd_image.history': 'keep_last',
            'qos_overrides./relay/rgbd_image.depth': 5,
            'qos_overrides./relay/rgbd_image.durability': 'volatile',
        }],
        output='screen'
    )

    # --- SLAM 노드 ---
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[slam_params, {'subscribe_rgbd': True}],
        remappings=[
            ('rgbd_image', '/relay/rgbd_image'),
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # --- Viz 노드 ---
    viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        parameters=[viz_params, {'subscribe_rgbd': True}],
        remappings=[
            ('rgbd_image', '/relay/rgbd_image'),
        ],
        output='screen'
    )

    return LaunchDescription([
        relay_rgbd,
        slam_node,
        viz_node
    ])
