#!/usr/bin/env bash

# X11 권한 허용 (1회만 필요)
xhost +si:localuser:root

docker run --rm -it \
  --name isaac-sim \
  --gpus all \
  --network host \
  --ipc=host \
  --user $(id -u):$(id -g) \
  --entrypoint /bin/bash \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -e ROS2_WS=/ros2_ws/src/doosanrobotics_cumotion_driver \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMENT_PREFIX_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \
  -e PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v "$HOME/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev:rw" \
  -v "$HOME/ros2_ws/src/doosan-robot2:/ros2_ws/src/doosan-robot2:ro" \
  -v "$HOME/ros2_ws/src/doosanrobotics_cumotion_driver:/ros2_ws/src/doosanrobotics_cumotion_driver:ro" \
  -v "$HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$HOME/docker/isaac-sim/cache/ov:/root/.cache/ov:rw" \
  -v "$HOME/docker/isaac-sim/cache/pip:/root/.cache/pip:rw" \
  -v "$HOME/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw" \
  -v "$HOME/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw" \
  -v "$HOME/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw" \
  -v "$HOME/docker/isaac-sim/data:/root/.local/share/ov/data:rw" \
  -v "$HOME/docker/isaac-sim/documents:/root/Documents:rw" \
  nvcr.io/nvidia/isaac-sim:4.5.0
