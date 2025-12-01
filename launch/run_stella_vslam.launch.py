from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_vocab = os.path.join(get_package_share_directory("challenge_tools_ros"),
                                 "config", "hilti_stella_vslam", "orb_vocab.fbow")

    default_config = os.path.join(get_package_share_directory("challenge_tools_ros"),
                                  "config", "hilti_stella_vslam", "fisheye.yaml")

    vocab_arg = DeclareLaunchArgument("vocab", default_value=default_vocab)
    config_arg = DeclareLaunchArgument("config_path", default_value=default_config)
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    image_conversion_arg = DeclareLaunchArgument(
        "image_conversion",
        default_value="true",
        description="Enable compressed-to-image conversion node"
    )

    vocab = LaunchConfiguration("vocab")
    config_path = LaunchConfiguration("config_path")
    namespace = LaunchConfiguration("namespace")

    node_slam = Node(
        package="stella_vslam_ros",
        executable="run_slam",
        namespace=namespace,
        name="stella_vslam",
        output="screen",
        arguments=["-v", vocab, "-c", config_path],
        parameters=[{"publish_tf": False}],
    )

    node_transport = Node(
        package="challenge_tools_ros",
        executable="image_conversion_node.py",
        namespace=namespace,
        output="screen",
        arguments=[
            "/cam0/image_raw/compressed",
            "/camera/image_raw",
        ],
        condition=IfCondition(LaunchConfiguration("image_conversion"))
    )

    return LaunchDescription([
        vocab_arg,
        config_arg,
        namespace_arg,
        image_conversion_arg,
        node_slam,
        node_transport,
    ])
