#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np

from isaacsim import SimulationApp

# -----------------------------------------------------------------------------
# SimulationApp config
# -----------------------------------------------------------------------------
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# -----------------------------------------------------------------------------
# Imports (Isaac Sim 4.5 namespace)
# -----------------------------------------------------------------------------
import carb
import omni.usd
import omni.graph.core as og

from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdPhysics, PhysxSchema

# Isaac Sim 4.5: core api/utils moved under isaacsim.*
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import nucleus, prims, rotations, stage, viewports, extensions
from isaacsim.core.utils.prims import set_targets


# Helper: carb settings
_settings = carb.settings.get_settings()

def _set(k, v):
    try:
        _settings.set(k, v)
    except Exception:
        pass

_set("/renderer/multiGpu/enabled", True)
_set("/renderer/textureStreaming/enabled", True)

_set("/rtx/enabled", True)
_set("/rtx/aa/enabled", True)
_set("/rtx/reflections/enabled", True)
_set("/rtx/indirectDiffuse/enabled", True)
_set("/rtx/ambientOcclusion/enabled", True)

# Paths / prim settings
ROBOT_MOUNT_PRIM = "/Root"
ROBOT_USD_PATH = "/ros2_ws/src/doosanrobotics_cumotion_driver/dsr_isaac_sim/usd/m1013_gripper.usd"  # docker

BACKGROUND_STAGE_PRIM = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"

GRAPH_PATH = "/ActionGraph"
ROS_VIEWPORT_NAME = "ros_camera_viewport"

KLT_STAGE_PATH = "/background/small_KLT"
KLT_USD_PATH = "/Isaac/Props/KLT_Bin/small_KLT.usd"


# Extensions (Isaac Sim 4.5)
#   - ROS2 nodes live in: isaacsim.ros2.bridge :contentReference[oaicite:1]{index=1}
#   - Core OmniGraph nodes live in: isaacsim.core.nodes :contentReference[oaicite:2]{index=2}
def _enable_ext(ext_name: str):
    try:
        extensions.enable_extension(ext_name)
        return True
    except Exception as e:
        carb.log_warn(f"Failed to enable extension '{ext_name}': {e}")
        return False

_enable_ext("isaacsim.core.nodes")
_enable_ext("isaacsim.ros2.bridge")
_enable_ext("omni.graph.action")

for _ in range(5):
    simulation_app.update()

# SimulationContext
simulation_context = SimulationContext(stage_units_in_meters=1.0)

for _ in range(3):
    simulation_app.update()

# Scene setup
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit(1)

# viewport camera
viewports.set_camera_view(
    eye=np.array([1.2, 1.2, 0.8]),
    target=np.array([0.0, 0.0, 0.5])
)

# background
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PRIM)

# (optional) KLT
prims.create_prim(KLT_STAGE_PATH, "Xform", position=np.array([-0.27, 0.14, 0.08]),
                  orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0)))
stage.add_reference_to_stage(assets_root_path + KLT_USD_PATH, KLT_STAGE_PATH)

# robot mount + reference
if not os.path.exists(ROBOT_USD_PATH):
    raise FileNotFoundError(f"Robot USD not found: {ROBOT_USD_PATH}")

stg = omni.usd.get_context().get_stage()

if not stg.GetPrimAtPath("/Root/m1013").IsValid():
    prims.create_prim(
        "/Root/m1013", "Xform",
        position=np.array([0.0, -0.64, 0.0]),
        orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    )

stage.add_reference_to_stage(ROBOT_USD_PATH, "/Root/m1013")

# stage stabilization
for _ in range(10):
    simulation_app.update()
    time.sleep(0.02)

stg = omni.usd.get_context().get_stage()


# Add objects
prims.create_prim(
    "/Wall",
    "Cube",
    position=np.array([-0.04694, 0.33183, 0.10395]),
    scale=np.array([0.04, 0.4, 0.10])
)

# Blue cube
blue_cube_prim = prims.create_prim("/blue_cube", "Xform", position=np.array([0.2, -0.08, 0.04]))
blue_cube_geom_prim = prims.create_prim("/blue_cube/cube", "Cube")
UsdGeom.Cube(blue_cube_geom_prim).GetSizeAttr().Set(0.04)

material_prim = prims.create_prim("/blue_cube/material", "Material")
material = UsdShade.Material(material_prim)
shader = UsdShade.Shader.Define(stg, "/blue_cube/material/shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0, 0, 230/255.0))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(blue_cube_geom_prim).Bind(material)

UsdPhysics.RigidBodyAPI.Apply(blue_cube_prim)
UsdPhysics.CollisionAPI.Apply(blue_cube_prim)
UsdPhysics.MassAPI.Apply(blue_cube_prim).GetMassAttr().Set(0.05)

# Red cube
red_cube_prim = prims.create_prim("/red_cube", "Xform", position=np.array([0.2, 0.12, 0.04]))
red_cube_geom_prim = prims.create_prim("/red_cube/cube", "Cube")
UsdGeom.Cube(red_cube_geom_prim).GetSizeAttr().Set(0.04)

red_material_prim = prims.create_prim("/red_cube/material", "Material")
red_material = UsdShade.Material(red_material_prim)
red_shader = UsdShade.Shader.Define(stg, "/red_cube/material/shader")
red_shader.CreateIdAttr("UsdPreviewSurface")
red_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((1.0, 0.0, 0.0))
red_material.CreateSurfaceOutput().ConnectToSource(red_shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(red_cube_geom_prim).Bind(red_material)

UsdPhysics.RigidBodyAPI.Apply(red_cube_prim)
UsdPhysics.CollisionAPI.Apply(red_cube_prim)
UsdPhysics.MassAPI.Apply(red_cube_prim).GetMassAttr().Set(0.05)

# def _get_or_add_scale_op(xform_prim):
#     xform = UsdGeom.Xformable(xform_prim)
#     for op in xform.GetOrderedXformOps():
#         if op.GetOpType() == UsdGeom.XformOp.TypeScale:
#             return op
#     return xform.AddXformOp(UsdGeom.XformOp.TypeScale)

# def make_goal_box(name: str, position=(0.0, 0.0, 0.5), size=(0.04, 0.04, 0.04), mass=0.05):
#     base_path = f"/{name}"
#     cube_path = f"{base_path}/cube"
#     mat_path = f"{base_path}/material"
#     shader_path = f"{mat_path}/shader"

#     xform_prim = prims.create_prim(base_path, "Xform", position=np.array(position))
#     cube_prim = prims.create_prim(cube_path, "Cube")
#     UsdGeom.Cube(cube_prim).GetSizeAttr().Set(1.0)

#     sx, sy, sz = map(float, size)
#     scale_op = _get_or_add_scale_op(xform_prim)
#     scale_op.Set(Gf.Vec3f(sx, sy, sz))

#     material = UsdShade.Material.Define(stg, mat_path)
#     shader = UsdShade.Shader.Define(stg, shader_path)
#     shader.CreateIdAttr("UsdPreviewSurface")
#     shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))
#     shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
#     shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
#     material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
#     UsdShade.MaterialBindingAPI(cube_prim).Bind(material)

#     UsdPhysics.RigidBodyAPI.Apply(cube_prim)
#     UsdPhysics.CollisionAPI.Apply(cube_prim)
#     UsdPhysics.MassAPI.Apply(cube_prim).GetMassAttr().Set(float(mass))

#     return base_path

# # make_goal_box("goal_cube1", position=(-0.22, 0.08, 0.02), size=(0.08, 0.08, 0.01), mass=0.05)
# make_goal_box("goal_cube", position=(-0.22, 0.18, 0.02), size=(0.08, 0.08, 0.01), mass=0.05)

simulation_app.update()

# ROS DOMAIN
try:
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
except ValueError:
    ros_domain_id = 10
print("Using ROS_DOMAIN_ID:", ros_domain_id)

# robot/camera frame paths
robot_root = "/Root/m1013/m1013" if stg.GetPrimAtPath("/Root/m1013/m1013").IsValid() else "/Root/m1013"
camera_path = f"{robot_root}/d435i_camera/realsense_camera"
grasp_frame_path = f"{robot_root}/tool0/gripper_frame/grasp_frame"

print("[INFO] robot_root:", robot_root)
print("[INFO] camera_path:", camera_path)
print("[INFO] grasp_frame_path:", grasp_frame_path)

# target prims for TF tree publish
target_list = [
    Sdf.Path(robot_root),
    Sdf.Path(camera_path),
    Sdf.Path(grasp_frame_path),
    # Sdf.Path("/goal_cube1/cube"),
    # Sdf.Path("/goal_cube/cube"),
    Sdf.Path("/blue_cube"),
    Sdf.Path("/red_cube"),
    Sdf.Path(KLT_STAGE_PATH),
]

# Graph 1: Robot <-> ROS
#   - Node types updated to Isaac Sim 4.5 namespaces
#   - ROS2Context lives in isaacsim.ros2.bridge :contentReference[oaicite:3]{index=3}
#   - Time/Articulation nodes live in isaacsim.core.nodes :contentReference[oaicite:4]{index=4}
try:
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": f"{GRAPH_PATH}/Robot", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),

                ("Context", "isaacsim.ros2.bridge.ROS2Context"),

                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),

                ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),

                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishTransformTree.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),

                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishTransformTree.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),

                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),

                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(ros_domain_id)),
                ("Context.inputs:useDomainIDEnvVar", True),

                ("ArticulationController.inputs:robotPath", robot_root),

                ("PublishJointState.inputs:topicName", "/isaac/joint_states"),

                ("SubscribeJointState.inputs:topicName", "/joint_states_to_isaac"),
                # ("SubscribeJointState.inputs:topicName", "/joint_states"),

                ("PublishTransformTree.inputs:targetPrims", target_list),

                ("PublishClock.inputs:topicName", "/clock"),
            ],
        },
    )

    set_targets(
        prim=stg.GetPrimAtPath(f"{GRAPH_PATH}/Robot/PublishJointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[robot_root],
    )

except Exception as e:
    print(f"[Robot Graph] Error: {e}")


# Graph 2: Camera -> ROS (RGB/Depth/CameraInfo)
#   - Viewport nodes are in isaacsim.core.nodes :contentReference[oaicite:7]{index=7}
try:
    camera_frame = camera_path.split("/")[-1]

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": f"{GRAPH_PATH}/Camera",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),

                ("CreateViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("GetRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("SetCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),

                ("Context", "isaacsim.ros2.bridge.ROS2Context"),

                ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateViewport.inputs:execIn"),
                ("CreateViewport.outputs:execOut", "GetRenderProduct.inputs:execIn"),
                ("CreateViewport.outputs:viewport", "GetRenderProduct.inputs:viewport"),

                ("GetRenderProduct.outputs:execOut", "SetCamera.inputs:execIn"),
                ("GetRenderProduct.outputs:renderProductPath", "SetCamera.inputs:renderProductPath"),

                ("SetCamera.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
                ("SetCamera.outputs:execOut", "CameraHelperInfo.inputs:execIn"),
                ("SetCamera.outputs:execOut", "CameraHelperDepth.inputs:execIn"),

                ("GetRenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                ("GetRenderProduct.outputs:renderProductPath", "CameraHelperInfo.inputs:renderProductPath"),
                ("GetRenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),

                ("Context.outputs:context", "CameraHelperRgb.inputs:context"),
                ("Context.outputs:context", "CameraHelperInfo.inputs:context"),
                ("Context.outputs:context", "CameraHelperDepth.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(ros_domain_id)),
                ("Context.inputs:useDomainIDEnvVar", True),

                ("CreateViewport.inputs:name", ROS_VIEWPORT_NAME),
                ("CreateViewport.inputs:viewportId", 2),

                ("CameraHelperRgb.inputs:frameId", camera_frame),
                ("CameraHelperInfo.inputs:frameId", camera_frame),
                ("CameraHelperDepth.inputs:frameId", camera_frame),

                ("CameraHelperRgb.inputs:topicName", "rgb"),
                ("CameraHelperRgb.inputs:type", "rgb"),

                ("CameraHelperInfo.inputs:topicName", "camera_info"),

                ("CameraHelperDepth.inputs:topicName", "depth"),
                ("CameraHelperDepth.inputs:type", "depth"),

                ("SetCamera.inputs:cameraPrim", [Sdf.Path(camera_path)]),
            ],
        },
    )

    og.Controller.evaluate_sync(ros_camera_graph)

except Exception as e:
    print(f"[Camera Graph] Error: {e}")


# Camera intrinsics
cam_prim = stg.GetPrimAtPath(camera_path)
if cam_prim.IsValid():
    cam = UsdGeom.Camera(cam_prim)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(15.7)
    cam.GetFocalLengthAttr().Set(18.8)
    cam.GetFocusDistanceAttr().Set(400.0)


# Run loop
simulation_context.play()

while simulation_app.is_running():
    simulation_context.step(render=True)

    # Impulse trigger for Robot graph
    try:
        og.Controller.set(
            og.Controller.attribute(f"{GRAPH_PATH}/Robot/OnImpulseEvent.state:enableImpulse"),
            True
        )
    except Exception:
        pass

simulation_context.stop()
simulation_app.close()
