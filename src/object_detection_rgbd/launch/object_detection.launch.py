from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments if needed
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Object Detection Node
    object_detection_node = Node(
        package="object_detection_rgbd",
        executable="object_detection_node",
        name="object_detection_node",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": "/camera/color/image_raw",
                "depth_image_topic": "/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "rgbd_topic": "/camera/rgbd",
                "use_rgbd": False,  # Whether to subscribe to rgbd topic instead of image and depth topics
                # Publishers
                "detected_objects_topic": "/detected_objects",
                "annotated_image_topic": "/annotated_image",
                "publish_markers_topic": "/detected_objects_3d",
                "output_pointcloud_topic": "/detected_objects_pointcloud",
                "detections_3d_topic": "/detections_3d",
                # Detection Model Parameters
                "model_type": "yolov5",
                "model_name": "yolov5s",
                # Path to fine-tuned YOLOv5 weights (best.pt). Empty = pretrained COCO model_name.
                # With custom weights, set filter_classes to that model's class names, e.g. ["plant"].
                "weights_path": "",
                "use_gpu": False,
                "confidence_threshold": 0.4,
                # Remove this line to keep all 80 COCO classes
                "filter_classes": ["person", "potted plant"],
                # Visualization Parameters
                "publish_annotated_image": True,
                "publish_3d": True,
                "publish_pointcloud": True,
                "pointcloud_color_mode": "rgb",  # 'rgb', 'class' or 'none'
            }
        ],
        output="screen",
    )

    return LaunchDescription([object_detection_node])
