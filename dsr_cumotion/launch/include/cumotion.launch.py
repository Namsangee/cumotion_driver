import os
from typing import List, Tuple
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import Action, Node, LaunchDescription

def get_realsense_depth_topics(num_cameras: int) -> Tuple[str, str]:
    depth_image_topics = []
    depth_camera_infos = []
    for i in range(num_cameras):
        depth_image_topics.append(f'/camera_{i+1}/aligned_depth_to_color/image_raw')
        depth_camera_infos.append(f'/camera_{i+1}/aligned_depth_to_color/camera_info')
    return depth_image_topics, depth_camera_infos

def get_isaac_sim_depth_topics() -> Tuple[str, str]:
    depth_image_topics: str = '["/depth"]'
    depth_camera_infos: str = '["/camera_info"]'
    return depth_image_topics, depth_camera_infos

def add_cumotion(args: lu.ArgumentContainer) -> List[Action]:
    camera_type = str(args.camera_type).lower()
    num_cameras = int(args.num_cameras)
    enable_object_attachment = lu.is_true(args.enable_object_attachment)
    workspace_bounds_name = str(args.workspace_bounds_name)
    actions = []

    if camera_type == "realsense":
        depth_image_topics, depth_camera_infos = get_realsense_depth_topics(num_cameras)
    elif camera_type == "isaac_sim":
        depth_image_topics, depth_camera_infos = get_isaac_sim_depth_topics()
    else:
        raise Exception(f"Camera type '{camera_type}' not recognized. Choose from ['realsense', 'isaac_sim'].")

    filter_speckles_in_robot_mask = False
    max_filtered_speckles_size = 0
    robot_mask_publish_topics = []
    world_depth_publish_topics = []
    for i in range(num_cameras):
        robot_mask_publish_topics.append(f'/cumotion/camera_{i+1}/robot_mask')
        world_depth_publish_topics.append(f'/cumotion/camera_{i+1}/world_depth')

    workspace_file_path = lu.get_path(
        'dsr_cumotion',
        f'config/{workspace_bounds_name}.yaml')
    if not os.path.exists(workspace_file_path):
        raise Exception(
            f'Workspace with name {workspace_bounds_name} does not exist. '
            'Launching cumotion or esdf visualizer without valid workspace is not allowed.')
    actions.append(
        lu.log_info([
            "Loading the '", workspace_bounds_name,
            "' workspace. Ignoring the grid_center_m / grid_size_m parameters of "
            "cumotion"
        ]))

    actions.append(
        lu.include(
            'isaac_ros_cumotion',
            'launch/isaac_ros_cumotion.launch.py',
            launch_arguments={
                'cumotion_planner.robot': args.robot_file_name,
                'cumotion_planner.urdf_path': args.urdf_file_path,
                'cumotion_planner.time_dilation_factor': args.time_dilation_factor,
                'cumotion_planner.max_attempts': args.max_attempts,
                'cumotion_planner.num_graph_seeds': args.num_graph_seeds,
                'cumotion_planner.num_trajopt_seeds': args.num_trajopt_seeds,
                'cumotion_planner.include_trajopt_retract_seed': 'True',
                'cumotion_planner.num_trajopt_time_steps': args.num_trajopt_time_steps,
                'cumotion_planner.interpolation_dt': '0.025',
                'cumotion_planner.joint_states_topic': args.joint_states_topic,
                'cumotion_planner.esdf_service_name': '/nvblox_node/get_esdf_and_gradient',
                'cumotion_planner.read_esdf_world': args.read_esdf_world,
                'cumotion_planner.update_esdf_on_request': args.update_esdf_on_request,
                'cumotion_planner.use_aabb_on_request': 'False',
                'cumotion_planner.collision_cache_cuboid': '20',
                'cumotion_planner.collision_cache_mesh': '20',
                'cumotion_planner.workspace_file_path': str(workspace_file_path),
                'cumotion_planner.grid_size_m': '[2.0, 2.0, 2.0]',
                'cumotion_planner.grid_center_m': '[0.0, 0.0, 0.0]',
                'cumotion_planner.voxel_size': '0.01',
                'cumotion_planner.publish_voxel_size': '0.01',
                'cumotion_planner.max_publish_voxels': '10000',
                'cumotion_planner.publish_curobo_world_as_voxels': args.publish_curobo_world_as_voxels,
                'cumotion_planner.add_ground_plane': 'True',
                'cumotion_planner.tool_frame': args.tool_frame,
                'cumotion_planner.override_moveit_scaling_factors': 'False',
                'cumotion_planner.update_link_sphere_server': args.update_link_sphere_server_planner,
                'cumotion_planner.enable_curobo_debug_mode': 'False',
            },
        ))

    if enable_object_attachment:
        actions.append(
            lu.include(
                'isaac_ros_cumotion_object_attachment',
                'launch/object_attachment.launch.py',
                launch_arguments={
                    'object_attachment.robot': args.robot_file_name,
                    'object_attachment.urdf_path': args.urdf_file_path,
                    'object_attachment.time_sync_slop': args.time_sync_slop,
                    'object_attachment.filter_depth_buffer_time': args.filter_depth_buffer_time,
                    'object_attachment.joint_states_topic': args.joint_states_topic,
                    'object_attachment.depth_image_topics': depth_image_topics,
                    'object_attachment.depth_camera_infos': depth_camera_infos,
                    'object_attachment.object_link_name': args.object_link_name,
                    'object_attachment.action_names': args.action_names,
                    'object_attachment.search_radius': args.search_radius,
                    'object_attachment.surface_sphere_radius': args.surface_sphere_radius,
                    'object_attachment.clustering_bypass_clustering': args.clustering_bypass,
                    'object_attachment.clustering_hdbscan_min_samples': args.clustering_hdbscan_min_samples,
                    'object_attachment.clustering_hdbscan_min_cluster_size': args.clustering_hdbscan_min_cluster_size,
                    'object_attachment.clustering_hdbscan_cluster_selection_epsilon': args.clustering_hdbscan_cluster_selection_epsilon,
                    'object_attachment.clustering_num_top_clusters_to_select': args.clustering_num_top_clusters_to_select,
                    'object_attachment.clustering_group_clusters': args.clustering_group_clusters,
                    'object_attachment.clustering_min_points': args.clustering_min_points,
                    'object_attachment.depth_qos': args.qos_setting,
                    'object_attachment.depth_info_qos': args.qos_setting,
                    'use_sim_time': args.use_sim_time,
                    'object_attachment.object_esdf_clearing_padding': args.object_esdf_clearing_padding,
                    'object_attachment.trigger_aabb_object_clearing': args.trigger_aabb_object_clearing
                }))
    return actions

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera_type')
    args.add_arg('enable_object_attachment', True)
    args.add_arg('num_cameras', 1)
    args.add_arg('workspace_bounds_name', '')
    args.add_arg('use_sim_time', False)

    args.add_arg('urdf_file_path', cli=True, default='')
    args.add_arg('robot_file_name', cli=True, default='')
    args.add_arg('time_dilation_factor', cli=True, default='0.25')
    args.add_arg('max_attempts', cli=True, default='20')
    args.add_arg('num_graph_seeds', cli=True, default='6')
    args.add_arg('num_trajopt_seeds', cli=True, default='6')
    args.add_arg('num_trajopt_time_steps', cli=True, default='64')
    args.add_arg('read_esdf_world', cli=True, default='False')
    args.add_arg('update_esdf_on_request', cli=True, default='False')
    args.add_arg('tool_frame', cli=True, default='grasp_frame')
    args.add_arg('update_link_sphere_server_planner', cli=True, default='planner_attach_object')
    args.add_arg('distance_threshold', cli=True, default='0.15')
    args.add_arg('time_sync_slop', cli=True, default='0.1')
    args.add_arg('filter_depth_buffer_time', cli=True, default='0.1')
    args.add_arg('joint_states_topic', cli=True, default='/joint_states')
    args.add_arg('trigger_aabb_object_clearing', cli=True, default='False')
    args.add_arg('object_link_name', cli=True, default='attached_object')
    args.add_arg('search_radius', cli=True, default='0.1')
    args.add_arg('update_link_sphere_server_segmenter', cli=True, default='segmenter_attach_object')
    args.add_arg('clustering_bypass', cli=True, default='True')
    args.add_arg('action_names', cli=True, default="['planner_attach_object']")
    args.add_arg('clustering_hdbscan_min_samples', cli=True, default='20')
    args.add_arg('clustering_hdbscan_min_cluster_size', cli=True, default='30')
    args.add_arg('clustering_hdbscan_cluster_selection_epsilon', cli=True, default='0.5')
    args.add_arg('clustering_num_top_clusters_to_select', cli=True, default='3')
    args.add_arg('clustering_group_clusters', cli=True, default='False')
    args.add_arg('clustering_min_points', cli=True, default='100')
    args.add_arg('publish_curobo_world_as_voxels', cli=True, default='False')
    args.add_arg('qos_setting', cli=True, default='SENSOR_DATA')
    args.add_arg('surface_sphere_radius', cli=True, default='0.01')
    args.add_arg('object_esdf_clearing_padding', cli=True, default='[0.025, 0.025, 0.025]')

    args.add_opaque_function(add_cumotion)
    return LaunchDescription(args.get_launch_actions())
