# tb3_fusion.launch.py  (ROS 2 Humble)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ---- Launch Args (필요하면 값 바꿔서 쓰면 됨)
    camera_ns     = DeclareLaunchArgument('camera_ns', default_value='/camera')
    width         = DeclareLaunchArgument('width',     default_value='320')
    height        = DeclareLaunchArgument('height',    default_value='240')
    color_fps     = DeclareLaunchArgument('color_fps', default_value='3')
    depth_fps     = DeclareLaunchArgument('depth_fps', default_value='3')
    use_device_time = DeclareLaunchArgument('use_device_time', default_value='false')

    approx_sync   = DeclareLaunchArgument('approx_sync', default_value='true')
    approx_sync_max_interval = DeclareLaunchArgument('approx_sync_max_interval', default_value='0.20')
    queue_size    = DeclareLaunchArgument('queue_size', default_value='20')
    qos_numeric   = DeclareLaunchArgument('qos_numeric', default_value='2')   # 네가 쓰던 qos:=2

    subscribe_compressed = DeclareLaunchArgument('subscribe_compressed', default_value='true')
    use_tb3_bringup = DeclareLaunchArgument('use_tb3_bringup', default_value='true')

    # ---- Substitutions
    cam_ns   = LaunchConfiguration('camera_ns')
    W        = LaunchConfiguration('width')
    H        = LaunchConfiguration('height')
    CFPS     = LaunchConfiguration('color_fps')
    DFPS     = LaunchConfiguration('depth_fps')
    UDT      = LaunchConfiguration('use_device_time')

    AS       = LaunchConfiguration('approx_sync')
    AS_INT   = LaunchConfiguration('approx_sync_max_interval')
    QSIZE    = LaunchConfiguration('queue_size')
    QOS_NUM  = LaunchConfiguration('qos_numeric')
    SUB_COMP = LaunchConfiguration('subscribe_compressed')
    USE_TB3  = LaunchConfiguration('use_tb3_bringup')

    # ---- Env (네가 쓰던 설정)
    envs = [
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='42'),
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    ]

    # ---- 1) Astra 카메라 (기존 astra.launch.py 포함)
    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astra.launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'sync_color_depth': 'true',
            'depth_registration': 'false',
            'color_fps': CFPS,
            'depth_fps': DFPS,
            'use_device_time': 'false',
            'width': W,
            'height': H,
            # QoS overrides (네 값 그대로)
            'qos_overrides./camera/color/image_raw': '2',
            'qos_overrides./camera/depth/image_raw': '2',
            'qos_overrides./camera/color/camera_info': '2',
        }.items()
    )

    # ---- 2) TF 고정 변환 두 개
    tf1 = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x','0.10','--y','0','--z','0.05','--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','base_link','--child-frame-id','camera_link'],
        respawn=True, respawn_delay=2.0
    )
    tf2 = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x','0','--y','0','--z','0','--roll','-1.5708','--pitch','0','--yaw','-1.5708',
                   '--frame-id','camera_link','--child-frame-id','camera_color_optical_frame'],
        respawn=True, respawn_delay=2.0
    )

    # ---- 3) rtabmap_sync/rgbd_sync (RGB+Depth+Info -> /rgbd_image)
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{
            'approx_sync': AS,
            'approx_sync_max_interval': AS_INT,
            'queue_size': QSIZE,
            'qos': QOS_NUM,
            'subscribe_compressed': SUB_COMP,
        }],
        remappings=[
            ('rgb/image',        [cam_ns, '/color/image_raw']),
            ('depth/image',      [cam_ns, '/depth/image_raw']),
            ('rgb/camera_info',  [cam_ns, '/color/camera_info']),
            ('rgbd_image',       '/rgbd_image'),
        ],
        respawn=True, respawn_delay=2.0
    )

    # ---- 4) TurtleBot3 bringup (옵션)
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
        ),
        condition=None,  # 그냥 항상 포함. 필요 없으면 실행 시 use_tb3_bringup:=false로 끄면 된다.
    )

    return LaunchDescription(
        [
            camera_ns, width, height, color_fps, depth_fps, use_device_time,
            approx_sync, approx_sync_max_interval, queue_size, qos_numeric,
            subscribe_compressed, use_tb3_bringup,
        ]
        + envs
        + [astra_launch, tf1, tf2, rgbd_sync, tb3_bringup]
    )
