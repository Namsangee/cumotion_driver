from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time      = LaunchConfiguration("use_sim_time")
    global_frame      = LaunchConfiguration("global_frame")
    voxel_size        = LaunchConfiguration("voxel_size")
    depth_topic       = LaunchConfiguration("depth_topic")
    depth_info_topic  = LaunchConfiguration("depth_info_topic")
    color_topic       = LaunchConfiguration("color_topic")
    color_info_topic  = LaunchConfiguration("color_info_topic")
    map_clear_frame   = LaunchConfiguration("map_clearing_frame_id")
    map_clear_radius  = LaunchConfiguration("map_clearing_radius_m")
    params_file       = LaunchConfiguration("params_file")

    nvblox_node = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="log",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "global_frame": global_frame,
                "esdf_mode": "3d",
                "voxel_size": voxel_size,
                "layer_visualization_exclusion_radius_m": 15.0,
                "layer_visualization_exclusion_height_m": 2.0,
                "map_clearing_frame_id": map_clear_frame,
                "map_clearing_radius_m": map_clear_radius,
                "use_tf_transforms": True,
            },
        ],
        remappings=[
            ("camera_0/depth/image",       depth_topic),
            ("camera_0/depth/camera_info", depth_info_topic),
            ("camera_0/color/image",       color_topic),
            ("camera_0/color/camera_info", color_info_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time",     default_value="true"),
        DeclareLaunchArgument("global_frame",     default_value="base_link"),
        DeclareLaunchArgument("voxel_size",       default_value="0.05"),
        DeclareLaunchArgument("depth_topic",      default_value="/depth"),
        DeclareLaunchArgument("depth_info_topic", default_value="/camera_info"),
        DeclareLaunchArgument("color_topic",      default_value="/rgb"),
        DeclareLaunchArgument("color_info_topic", default_value="/camera_info"),
        DeclareLaunchArgument("map_clearing_frame_id", default_value="base_link"),
        DeclareLaunchArgument("map_clearing_radius_m", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("dsr_cumotion"), "config", "nvblox_node.yaml"
            ])
        ),
        nvblox_node,
    ])
