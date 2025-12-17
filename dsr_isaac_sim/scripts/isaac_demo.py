#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from action_msgs.msg import GoalStatusArray
from control_msgs.action import GripperCommand

from dsr_cumotion_msgs.msg import TargetPose, TargetJoint
from dsr_cumotion_goal_interface.utils.math_utils import (
    euler_to_quaternion,
    quaternion_multiply,
)
from dsr_cumotion_msgs.srv import PickPlace


class CubeToTargetPoseAndGripOnce(Node):
    """
    Finite State Machine (FSM) for a single pick-and-place operation.

    Flow:
        HOME → OPEN_GRIPPER → PICK → CLOSE_GRIPPER → PLACE → OPEN_GRIPPER → END

    Recovery:
        If MoveIt planning fails (ABORTED),
        retry PICK motion with yaw rotated by +90° up to 3 times.
        If all retries fail, return HOME and shut down.
    """

    def __init__(self):
        super().__init__("cube_to_target_pose_and_grip_once")

        # ---------------- ROS Parameters ----------------
        self.declare_parameter("pick_cube_frame", "red_cube")     # TF frame of pick target
        self.declare_parameter("place_cube_frame", "small_KLT")         # TF frame of place target
        self.declare_parameter("base_frame", "base_link")              # Base TF frame

        self.declare_parameter("pick_z_offset", 0.0)                # Z offset for pick pose (m)
        self.declare_parameter("place_z_offset", 0.1)                  # Z offset for place pose (m)

        self.declare_parameter("vel_scale", 0.5)                        # Velocity scaling (0–1)
        self.declare_parameter("acc_scale", 0.2)                        # Acceleration scaling (0–1)

        self.declare_parameter("gripper_open_position", 0.0)           # Gripper open position
        self.declare_parameter("gripper_close_position", 0.8)          # Gripper close position
        self.declare_parameter("gripper_effort", 50.0)                 # Max gripper effort

        # ---------------- Parameter Values ----------------
        self.pick_cube = self.get_parameter("pick_cube_frame").value
        self.place_cube = self.get_parameter("place_cube_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.pick_z = self.get_parameter("pick_z_offset").value
        self.place_z = self.get_parameter("place_z_offset").value

        self.vel_scale = self.get_parameter("vel_scale").value
        self.acc_scale = self.get_parameter("acc_scale").value

        self.gripper_open = self.get_parameter("gripper_open_position").value
        self.gripper_close = self.get_parameter("gripper_close_position").value
        self.gripper_effort = self.get_parameter("gripper_effort").value

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Publishers ----------------
        self.target_pose_pub = self.create_publisher(TargetPose, "/target_pose", 10)
        self.target_joint_pub = self.create_publisher(TargetJoint, "/target_joint", 10)

        # ---------------- MoveIt Status ----------------
        self.move_status_sub = self.create_subscription(
            GoalStatusArray,
            "/move_action/_action/status",
            self.move_status_cb,
            10,
        )

        # ---------------- Gripper Action ----------------
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_position_controller/gripper_cmd",
        )

        self.pickplace_client = self.create_client(
            PickPlace,
            "/attach_detach_command",
        )
        
        # ---------------- FSM State ----------------
        self.state = "MOVE_HOME"
        self.waiting_move = False

        # ---------------- MoveIt UUID Tracking ----------------
        self.last_uuid_seen = None
        self.armed_prev_uuid = None
        self.active_goal_uuid = None

        # ---------------- Retry Logic ----------------
        self.retry_count = 0
        self.max_retry = 3
        self.retry_yaw_step = math.pi / 2
        self.current_retry_yaw = 0.0

        self.exit_after_home = False

        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("Pick & Place FSM started")

    def timer_cb(self):
        """
        FSM tick function.
        Sends exactly one command per state and transitions to a waiting state.
        """
        if self.state == "MOVE_HOME":
            self._arm_for_new_move()
            self.send_home_joint()
            time.sleep(2.0) # for nvblox stabilization
            self.waiting_move = True
            self.state = "WAIT_MOVE_HOME"

        elif self.state == "OPEN_GRIPPER":
            self.send_gripper_goal(self.gripper_open)
            self.state = "WAIT_GRIPPER_OPEN"

        elif self.state == "MOVE_PICK":
            ok = self.send_target_pose(
                cube_frame=self.pick_cube,
                z_offset=self.pick_z,
                extra_yaw=self.current_retry_yaw,
            )
            if ok:
                self._arm_for_new_move()
                self.waiting_move = True
                self.state = "WAIT_MOVE_PICK"

        elif self.state == "MOVE_PLACE":
            ok = self.send_target_pose(
                cube_frame=self.place_cube,
                z_offset=self.place_z,
                extra_yaw=0.0,
            )
            if ok:
                self._arm_for_new_move()
                self.waiting_move = True
                self.state = "WAIT_MOVE_PLACE"

    def send_target_pose(self, cube_frame: str, z_offset: float, extra_yaw: float = 0.0) -> bool:
        """
        Publishes a TargetPose based on TF lookup.

        Args:
            cube_frame: Target TF frame name
            z_offset: Z-axis offset in meters
            extra_yaw: Additional yaw rotation in radians (used for retry)

        Returns:
            True if TF lookup and publish succeed, False otherwise
        """
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                cube_frame,
                rclpy.time.Time(),
            )
        except Exception:
            self.get_logger().warn(f"TF not ready: {cube_frame}")
            return False

        q_cube = [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]

        q_offset = euler_to_quaternion(math.pi, 0.0, 0.0)
        q_retry = euler_to_quaternion(0.0, 0.0, extra_yaw)

        q = quaternion_multiply(q_cube, q_offset)
        q = quaternion_multiply(q, q_retry)

        msg = TargetPose()
        msg.x = tf.transform.translation.x
        msg.y = tf.transform.translation.y
        msg.z = tf.transform.translation.z + z_offset
        msg.qx, msg.qy, msg.qz, msg.qw = q
        msg.rx = msg.ry = msg.rz = 0.0
        msg.max_vel_scale = self.vel_scale
        msg.max_acc_scale = self.acc_scale

        self.target_pose_pub.publish(msg)
        self.get_logger().info(
            f"TargetPose sent for {cube_frame} (yaw={math.degrees(extra_yaw):.1f} deg)"
        )
        return True

    def send_home_joint(self) -> None:
        """
        Publishes a TargetJoint command to move the robot to HOME position.
        """
        msg = TargetJoint()
        msg.joints = [0.0, 0.0, 60.0, 0.0, 115.0, 0.0]
        msg.max_vel_scale = self.vel_scale
        msg.max_acc_scale = self.acc_scale
        self.target_joint_pub.publish(msg)
        self.get_logger().info("Home joint command sent")

    def call_pickplace_service(self, motion_type: int) -> None:
        """
        Calls attach/detach service before gripper action.

        Args:
            motion_type: 0 = attach (pick), 1 = detach (place)
        """
        if not self.pickplace_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("PickPlace service not available")
            return

        req = PickPlace.Request()
        req.motion_type = motion_type

        future = self.pickplace_client.call_async(req)
        future.add_done_callback(
            lambda _: self.get_logger().info(
                f"PickPlace service called (motion_type={motion_type})"
            )
        )

    def _uuid_to_tuple(self, uuid_msg) -> tuple:
        """Converts a uint8[16] UUID to a tuple for comparison."""
        return tuple(uuid_msg)

    def _arm_for_new_move(self) -> None:
        """Prepares to detect a new MoveIt goal UUID."""
        self.armed_prev_uuid = self.last_uuid_seen
        self.active_goal_uuid = None

    def _reset_move_tracking(self) -> None:
        """Resets MoveIt tracking state."""
        self.waiting_move = False
        self.active_goal_uuid = None
        self.armed_prev_uuid = None

    def handle_move_failure(self) -> None:
        """
        Recovery handler for MoveIt planning failure (ABORTED).
        Retries PICK with yaw +90° up to 3 times, otherwise returns HOME and exits.
        """
        self._reset_move_tracking()

        if self.retry_count < self.max_retry:
            self.retry_count += 1
            self.current_retry_yaw += self.retry_yaw_step
            self.get_logger().warn(
                f"Planning failed. Retry {self.retry_count}/{self.max_retry}, "
                f"yaw={math.degrees(self.current_retry_yaw):.1f} deg"
            )
            self.state = "MOVE_PICK"
            return

        self.get_logger().error("Planning failed after all retries → HOME and shutdown")
        self.retry_count = 0
        self.current_retry_yaw = 0.0
        self.exit_after_home = True
        self.state = "MOVE_HOME"

    def move_status_cb(self, msg: GoalStatusArray):
        """
        Monitors MoveIt action status and drives FSM transitions.
        """
        if not msg.status_list:
            return

        latest = msg.status_list[-1]
        latest_uuid = self._uuid_to_tuple(latest.goal_info.goal_id.uuid)
        self.last_uuid_seen = latest_uuid

        if not self.waiting_move:
            return

        if self.active_goal_uuid is None:
            if self.armed_prev_uuid is not None and latest_uuid == self.armed_prev_uuid:
                return
            self.active_goal_uuid = latest_uuid
            self.get_logger().info("Captured new MoveIt goal UUID")

        target_status = None
        for s in msg.status_list:
            if self._uuid_to_tuple(s.goal_info.goal_id.uuid) == self.active_goal_uuid:
                target_status = s.status
                break

        if target_status is None:
            return

        if target_status == 6:  # ABORTED
            if self.state == "WAIT_MOVE_HOME":
                self.get_logger().error("Home move failed. Forcing shutdown.")
                self.shutdown()
                return

            if self.exit_after_home:
                self.get_logger().error("Abort during exit sequence. Shutting down.")
                self.shutdown()
                return

            self.handle_move_failure()
            return

        if target_status != 4:  # SUCCESS only
            return

        self._reset_move_tracking()

        if self.state == "WAIT_MOVE_HOME":
            if self.exit_after_home:
                self.get_logger().info("Home reached. Shutting down.")
                self.shutdown()
                return
            self.state = "OPEN_GRIPPER"
            return

        if self.state == "WAIT_MOVE_PICK":
            self.retry_count = 0
            self.current_retry_yaw = 0.0
            self.call_pickplace_service(0)  # Attach
            time.sleep(2.0)
            self.send_gripper_goal(self.gripper_close)
            self.state = "WAIT_GRIPPER_CLOSE"
            return

        if self.state == "WAIT_MOVE_PLACE":
            self.call_pickplace_service(1)  # Detach
            time.sleep(2.0)
            self.send_gripper_goal(self.gripper_open)
            self.state = "WAIT_GRIPPER_PLACE"
            return

    def send_gripper_goal(self, position: float) -> None:
        """
        Sends a gripper action goal.

        Args:
            position: Target gripper position
        """
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.shutdown()
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_effort

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_cb)

    def gripper_goal_cb(self, future) -> None:
        """Checks gripper goal acceptance."""
        handle = future.result()
        if not handle.accepted:
            self.shutdown()
            return
        handle.get_result_async().add_done_callback(self.gripper_result_cb)

    def gripper_result_cb(self, future) -> None:
        """Handles gripper result and advances FSM."""
        if self.state == "WAIT_GRIPPER_OPEN":
            self.state = "MOVE_PICK"
            return
        if self.state == "WAIT_GRIPPER_CLOSE":
            self.state = "MOVE_PLACE"
            return
        if self.state == "WAIT_GRIPPER_PLACE":
            self.get_logger().info("Place done → return HOME")

            # HOME으로 이동 후 종료하도록 플래그 설정
            self.exit_after_home = True

            # HOME 이동 명령 전송
            self._arm_for_new_move()
            self.send_home_joint()
            time.sleep(2.0)

            self.waiting_move = True
            self.state = "WAIT_MOVE_HOME"
            return

    def shutdown(self) -> None:
        """Shuts down the node cleanly."""
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CubeToTargetPoseAndGripOnce()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
