import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================
    # Package paths
    # =========================================================
    # serial_test_share = get_package_share_directory('serial_test')
    realsense_share = get_package_share_directory('realsense2_camera')

    # =========================================================
    # RealSense launch path
    # =========================================================
    realsense_launch_path = os.path.join(
        realsense_share,
        'launch',
        'rs_launch.py'
    )

    # =========================================================
    # 2. RealSense D435 launch
    # Equivalent command:
    # ros2 launch realsense2_camera rs_launch.py \
    #   depth_module.depth_profile:=640x480x15 \
    #   pointcloud.enable:=true \
    #   align_depth.enable:=true
    # =========================================================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items()
    )

    # =========================================================
    # 3. YOLOv8 detection node
    # Equivalent command:
    # ros2 run camera_perception_pkg yolov8_node
    # =========================================================
    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen'
    )

    # =========================================================
    # 3-1. YOLOv8 bbox visualizer node
    # Equivalent command:
    # ros2 run camera_perception_pkg yolo_bbox_visualizer
    # =========================================================
    yolo_bbox_visualizer_node = Node(
        package='camera_perception_pkg',
        executable='yolo_bbox_visualizer',
        name='yolo_bbox_visualizer',
        output='screen',
        parameters=[
            {
                'image_topic': '/camera/camera/color/image_raw',
                'detections_topic': '/detections',
                'annotated_topic': '/yolov8/annotated_image',
                'min_score': 0.3,
            }
        ]
    )

    # =========================================================
    # 4. Object distance node
    # Equivalent command:
    # ros2 run camera_perception_pkg object_distance_node --ros-args \
    #   -p detections_topic:=/detections \
    #   -p depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    #   -p depth_camera_info_topic:=/camera/camera/color/camera_info \
    #   -p base_frame:=camera_link \
    #   -p target_class_name:=btn_2_deactive
    # =========================================================
    object_distance_node = Node(
        package='camera_perception_pkg',
        executable='object_distance_node',
        name='object_distance_node',
        output='screen',
        parameters=[
            {
                'detections_topic': '/detections',
                'depth_image_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'depth_camera_info_topic': '/camera/camera/color/camera_info',
                'base_frame': 'camera_link',
                'target_class_name': 'btn_under1_deactive',
            }
        ]
    )

    # =========================================================
    # 5. YOLOv8 debug node
    # Equivalent command:
    # ros2 run camera_perception_pkg yolov8_debug_node
    # =========================================================
    yolov8_debug_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_debug_node',
        name='yolov8_debug_node',
        output='screen'
    )

    # =========================================================
    # Launch order
    # =========================================================
    # 카메라 토픽이 생성된 뒤 perception node들이 붙도록 약간 지연 실행
    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=[
                realsense_launch
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                yolov8_node
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                yolo_bbox_visualizer_node
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                object_distance_node
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                yolov8_debug_node
            ]
        ),
    ])