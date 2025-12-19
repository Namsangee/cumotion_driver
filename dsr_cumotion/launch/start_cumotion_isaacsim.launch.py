import os
import yaml

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from dsr_bringup2.utils import read_update_rate

def validate_gripper_mode(context):
    mode = str(LaunchConfiguration("mode").perform(context)).lower()
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()

    if mode == "real" and gripper == "2f85":
        raise RuntimeError(
            "[LAUNCH ERROR] Robotiq 2F-85 is only supported in VIRTUAL mode!"
        )
    return []

# Generate MoveIt2 + RViz node (CuMotion integrated)
def get_moveit_group_node(context):
    model = LaunchConfiguration("model").perform(context)
    use_sim_time = str(LaunchConfiguration("use_sim_time").perform(context)).lower()
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    pkg_share = get_package_share_directory("dsr_cumotion")
    enable_cumotion = (str(LaunchConfiguration("enable_cumotion").perform(context)).strip().lower()== "true")
    
    # File paths
    controller_file = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")
    urdf_path = os.path.join(pkg_share, "urdf", f"{model}.urdf.xacro")
    srdf_path = os.path.join(pkg_share, "srdf", f"{model}.srdf.xacro")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(model, package_name="dsr_cumotion")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "model": model,
                "color": LaunchConfiguration("color"),
                "gripper": gripper,
                "isaac_sim": LaunchConfiguration("isaac_sim"),
            },
        )
        .robot_description_semantic(
            file_path=srdf_path,
            mappings={
                "model": model,
                "gripper": gripper,
                "isaac_sim": LaunchConfiguration("isaac_sim")
            },
        )
        .robot_description_kinematics(file_path=kinematics_file)
        .trajectory_execution(file_path=controller_file)
        .to_moveit_configs()
    )

    ompl_path = os.path.join(pkg_share, "config", "ompl_planning.yaml")
    with open(ompl_path) as f:
        ompl_config = yaml.safe_load(f)

    pipelines = moveit_config.planning_pipelines.get("planning_pipelines", [])
    if isinstance(pipelines, list):
        pipelines.clear()
        pipelines.append("ompl")

    moveit_config.planning_pipelines["ompl"] = ompl_config
    moveit_config.planning_pipelines["default_planning_pipeline"] = "ompl"

    if enable_cumotion:
        cumotion_path = os.path.join(
            pkg_share, "config", "isaac_ros_cumotion_planning.yaml"
        )
        with open(cumotion_path) as f:
            cumotion_config = yaml.safe_load(f)

        pipelines.insert(0, "isaac_ros_cumotion")
        moveit_config.planning_pipelines["isaac_ros_cumotion"] = cumotion_config
        moveit_config.planning_pipelines["default_planning_pipeline"] = "isaac_ros_cumotion"

    if use_sim_time == "true":
        # moveit_config.trajectory_execution["trajectory_execution"]["allowed_start_tolerance"] = 0.1
        moveit_config.moveit_cpp.update({"use_sim_time": True})
    else:
        moveit_config.moveit_cpp.update({"use_sim_time": False})
    moveit_dict = moveit_config.to_dict()
    moveit_dict["capabilities"] = "move_group/ExecuteTaskSolutionCapability"

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_dict, {"use_sim_time": True}],
        remappings=[("/joint_states", "/isaac/joint_states"),],
    )
    nodes = [move_group_node]

    # Optionally include RViz2
    gui = str(LaunchConfiguration("gui").perform(context)).lower()
    if gui in ["true", "1", "yes"]:
        rviz_config = os.path.join(pkg_share, "config", "moveit_isaacsim.rviz")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            parameters=[moveit_dict],
        )
        nodes.append(rviz_node)
    return nodes

# Include CuMotion pipeline (Isaac ROS CuMotion)
def get_cumotion_node(context):
    model = LaunchConfiguration("model").perform(context)
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    enable_cumotion = str(LaunchConfiguration("enable_cumotion").perform(context)).lower()
    enable_attach = str(LaunchConfiguration("enable_attach").perform(context)).lower()

    pkg_share = get_package_share_directory("dsr_cumotion")
    nodes = []

    isaac_sim = str(LaunchConfiguration("isaac_sim").perform(context)).lower()

    if isaac_sim in ["true", "1", "yes"] and gripper == "2f85":
        urdf = f"{model}_isaac_sim.urdf"
        xrdf = f"{model}_isaac_sim.xrdf"

    elif gripper == "none":
        urdf = f"{model}_without_gripper.urdf"
        xrdf = f"{model}_without_gripper.xrdf"

    elif gripper == "vgc10":
        urdf = f"{model}_with_vgc10.urdf"
        xrdf = f"{model}_with_vgc10.xrdf"

    elif gripper == "2f85":
        urdf = f"{model}_with_2f85.urdf"
        xrdf = f"{model}_with_2f85.xrdf"

    else:
        raise RuntimeError(f"Invalid gripper type: {gripper}")

    urdf_path = os.path.join(pkg_share, "urdf", urdf)
    xrdf_path = os.path.join(pkg_share, "xrdf", xrdf)

    if enable_cumotion in ["true", "1", "yes"]:
        cumotion_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "include", "cumotion.launch.py")
            ),
            launch_arguments={
                "camera_type": "isaac_sim",
                "num_cameras": "1",
                "workspace_bounds_name": "workbound_test",
                "enable_object_attachment": enable_attach,
                "read_esdf_world": "true",
                "update_esdf_on_request": "true",
                "tool_frame": "grasp_frame",
                "joint_states_topic": "/isaac/joint_states",
                "urdf_file_path": urdf_path,
                "robot_file_name": xrdf_path,
            }.items(),
        )
        nodes.append(cumotion_launch)

    # if enable_attach in ["true", "1", "yes"]:
    #     static_depth_node = Node(
    #         package="dsr_cumotion",
    #         executable="camera_publisher.py",
    #         name="static_depth_camera_node",
    #         output="log",
    #     )
    #     nodes.append(static_depth_node)
    return nodes

def nvblox_node_fn(context):
    if LaunchConfiguration("enable_nvblox").perform(context).lower() not in ("true", "1", "yes"):
        return []

    pkg_share = get_package_share_directory("dsr_cumotion")
    params_file = os.path.join(pkg_share, "config", "nvblox_node.yaml")

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "include", "nvblox_node.launch.py")
        ),
        launch_arguments={
            "params_file": params_file
        }.items(),
    )
    return [nvblox_launch]


def control_node_fn(context):
    name = LaunchConfiguration("name")
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    robot_description_param = {"robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str)}
    pkg_share = get_package_share_directory('dsr_cumotion')
    pkg_share_dsr = get_package_share_directory("dsr_controller2")
    controller_yaml_path = os.path.join(pkg_share_dsr, "config", "dsr_controller2.yaml")

    params = [robot_description_param, controller_yaml_path]
    if gripper == "2f85":
        gripper_yaml = os.path.join(pkg_share, "config", "robotiq_controller.yaml")
        params.append(gripper_yaml)

    node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=name,
        parameters=params,
        output="both",
    )
    return [node]

def gripper_spawner_fn(context):
    if LaunchConfiguration("gripper").perform(context) != "2f85":
        return []
    return [
        Node(
            package="controller_manager",
            namespace=LaunchConfiguration("name"),
            executable="spawner",
            arguments=["gripper_position_controller", "-c", "controller_manager"],
            output="screen",
        )
    ]

def obstacle_manager_fn(context):
    obstacle = str(LaunchConfiguration("obstacle").perform(context)).lower()
    nodes=[]
    if obstacle in ["true", "1", "yes"]:

        pkg_share = get_package_share_directory("dsr_cumotion")
        yaml_path = os.path.join(pkg_share, "config", "obstacles.yaml")

        node=Node(
            package="dsr_cumotion",
            executable="obstacle_manager.py",
            name="obstacle_manager",
            output="screen",
            parameters=[{"config_file": yaml_path}],
        )
        nodes.append(node)
    return nodes
    

def pid_isaacsim_node_fn(context):
    isaac_sim = str(LaunchConfiguration("isaac_sim").perform(context)).lower()
    if isaac_sim not in ("true", "1", "yes"):
        return []

    node = Node(
        package="dsr_isaac_sim",        # PID 노드가 들어간 패키지
        executable="pid_isaacsim_node", # CMake에서 만든 executable
        name="pid_isaacsim_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return [node]

def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="", description="Namespace"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="Robot IP"),
        DeclareLaunchArgument("port", default_value="12345", description="Robot port"),
        DeclareLaunchArgument("mode", default_value="virtual", description="Mode"),
        DeclareLaunchArgument("model", default_value="m1013", description="Robot model"),
        DeclareLaunchArgument("color", default_value="white", description="Robot color"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("gz", default_value="false", description="Use Gazebo"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.100", description="RT IP"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time"),
        DeclareLaunchArgument("gripper", default_value="2f85", description="GRIPPER type"),
        DeclareLaunchArgument("obstacle", default_value="true", description="Obstacle using moveit planningscene"),
        DeclareLaunchArgument("enable_cumotion", default_value="true", description="Enable cumotion node"),
        DeclareLaunchArgument("enable_attach", default_value="true", description="Enable object_attach node"),
        DeclareLaunchArgument("enable_nvblox", default_value="true", description="Enable nvblox node"),
        DeclareLaunchArgument("isaac_sim", default_value="true", description="Enable Isaac Sim integration"),
    ]

    update_rate = str(read_update_rate()) # get update_rate from yaml

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("dsr_cumotion"),"urdf","m1013.urdf.xacro"]),
            " name:=",LaunchConfiguration("name"),
            " host:=",LaunchConfiguration("host"),
            " rt_host:=",LaunchConfiguration("rt_host"),
            " port:=",LaunchConfiguration("port"),
            " mode:=",LaunchConfiguration("mode"),
            " model:=",LaunchConfiguration("model"),
            " color:=",LaunchConfiguration("color"),
            " update_rate:=", update_rate,
            " gripper:=", LaunchConfiguration("gripper"),
            " isaac_sim:=", LaunchConfiguration("isaac_sim"),

        ]
    )
    set_robot_description = SetLaunchConfiguration("robot_description", robot_description_content)

    run_emulator = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"name":    LaunchConfiguration('name')  }, 
            {"rate":    100         },
            {"standby": 5000        },
            {"command": True        },
            {"host":    LaunchConfiguration('host')  },
            {"port":    LaunchConfiguration('port')  },
            {"mode":    LaunchConfiguration('mode')  },
            {"model":   LaunchConfiguration('model') },
            {"gripper": "none"      },
            {"mobile":  "none"      },
            {"rt_host":  LaunchConfiguration('rt_host')      },
            #parameters_file_path       # If a parameter is set in both the launch file and a YAML file, the value from the YAML file will be used.
        ],
        output="screen",
    )

    control_node = OpaqueFunction(function=control_node_fn)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        parameters=[
            {"robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str)}
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
        output="screen",
    )
    dsr_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["dsr_controller2", "-c", "controller_manager"],
        output="screen",
    )
    dsr_moveit_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
        output="screen",
    )

    motion_command = Node(
        package="dsr_cumotion_goal_interface",
        executable="move_command_node",
        name="move_command_node",
        output="screen",
        parameters = [{
            'planning_group': 'manipulator',
            'planner_pipeline': 'isaac_ros_cumotion',
            'planner_id': 'cuMotion',
            'base_frame': 'base_link',
            'tool_frame': 'grasp_frame',
            'allowed_planning_time': 5.0,
            'num_planning_attempts': 10,
            'max_vel_scale': 1.0,
            'max_acc_scale': 1.0,
            "retry_num" : 0,
        }]
    )

    pick_place_server = Node(
        package="dsr_cumotion",
        executable="pick_and_place_server.py",
        name="pick_and_place_server",
        output="screen",
    )

    cumotion = OpaqueFunction(function=get_cumotion_node)
    moveit_group = OpaqueFunction(function=get_moveit_group_node)
    obstacle = OpaqueFunction(function=obstacle_manager_fn)
    validation_guard = OpaqueFunction(function=validate_gripper_mode)
    nvblox_node = OpaqueFunction(function=nvblox_node_fn)
    pid_node = OpaqueFunction(function=pid_isaacsim_node_fn)

    delay_dsr_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg=">>  jsb active. Launching dsr_controller..."),
                dsr_controller
            ],
        )
    )

    delay_moveit_controller_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_controller,
            on_exit=[
                LogInfo(msg=">>  dsr_controller active. Launching moveit_controller..."),
                # dsr_moveit_controller, 
                OpaqueFunction(function=gripper_spawner_fn),
                dsr_moveit_controller],
        )
    )

    delay_moveit_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_controller,
            on_exit=[
                LogInfo(msg=">>  dsr_controller active. Launching moveit_controller..."),
                moveit_group
            ],
        )
    )

    delay_motion_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller,
            on_exit=[
                LogInfo(msg=">> moveit_controller active. Launching moveit node..."),
                motion_command
            ],
        )
    )

    delay_cumotion_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller,
            on_exit=[
                LogInfo(msg=">> moveit controller active. Launching cumotion node..."),
                cumotion
            ],
        )
    )

    delay_server_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller,
            on_exit=[
                LogInfo(msg=">> controller active. Launching pick_place_server node..."),
                pick_place_server, nvblox_node
            ],
        )
    )

    delay_obstacle_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller,
            on_exit=[
                LogInfo(msg=">> moveit_controller active. Launching obstacle node..."),
                obstacle
            ],
        )
    )
    
    delay_pid_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller,
            on_exit=[
                LogInfo(msg=">> moveit_controller active. Launching PID IsaacSim node..."),
                pid_node,
            ],
        )
    )
    return LaunchDescription(
        args
        + [
            validation_guard,
            set_robot_description,
            run_emulator,
            control_node, 
            # robot_state_publisher,
            joint_state_broadcaster,
            delay_dsr_controller_after_jsb,
            # dsr_controller,
            delay_moveit_controller_after_controller,
            delay_pid_after_moveit_controller,
            delay_motion_after_moveit_controller,
            # delay_obstacle_after_moveit_controller, 
            delay_server_after_moveit_controller,
            delay_cumotion_after_moveit_controller,
            delay_moveit_after_moveit_controller,
        ]
    )
