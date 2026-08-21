import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("waji_description")
    urdf_path = os.path.join(package_dir, "urdf", "waji.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    root_frame = LaunchConfiguration("root_frame")
    tip_frame = LaunchConfiguration("tip_frame")
    pose_topic = LaunchConfiguration("pose_topic")

    # 启动三个节点
    return LaunchDescription(
        [
            DeclareLaunchArgument("root_frame", default_value="machine_root_ros"),
            DeclareLaunchArgument("tip_frame", default_value="bucket_tip"),
            DeclareLaunchArgument(
                "pose_topic", default_value="/bucket_tip_pose_machine_root_ros"
            ),
            # The measured URDF retains its native fk_root link name.  It is
            # a right-handed ROS frame (+X forward, +Y left, +Z up), not a
            # Unity frame.  This explicit identity adapter makes the sole
            # system root machine_root_ros without editing measured URDF data.

            # 节点1：连接系统根和URDF根(ros自带节点)
            # machine_root_ros ──静态TF──> fk_root
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="machine_root_ros_to_fk_root_adapter",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "machine_root_ros",
                    "--child-frame-id", "fk_root",
                ],
            ),
            # 节点2：根据关节角计算整个挖机的 TF(ros自带节点)
            # 输入: robot_description为urdf文件,此外还有/joint_states 中的四个关节角
            # 根据关节角度计算bucket tip的pose,并发布到/bucket_tip_pose_machine_root_ros
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # 节点3：查询整条TF的最终结果
            # machine_root_ros → bucket_tip
            # ↓
            # 发布 /bucket_tip_pose_machine_root_ros
            Node(
                package="waji_description",
                executable="bucket_tip_pose_publisher.py",
                name="bucket_tip_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "root_frame": root_frame,
                        "tip_frame": tip_frame,
                        "pose_topic": pose_topic,
                    }
                ],
            ),
        ]
    )
