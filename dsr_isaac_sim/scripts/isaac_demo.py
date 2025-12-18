#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from action_msgs.msg import GoalStatusArray
from control_msgs.action import GripperCommand

from dsr_cumotion_msgs.msg import TargetPose, TargetJoint
from dsr_cumotion_goal_interface.utils.math_utils import euler_to_quaternion,quaternion_multiply
from dsr_cumotion_msgs.srv import PickPlace

class IsaacsimTest(Node):
    def __init__(self):
        super().__init__("cube_to_target_pose_and_grip_once")

        # Parameters
        self.declare_parameter("pick_cube_frames", ["red_cube", "blue_cube"])
        self.declare_parameter("place_cube_frame", "small_KLT")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("pick_z_offset", 0.0)
        self.declare_parameter("place_z_offset", 0.1)

        self.declare_parameter("vel_scale", 0.5)
        self.declare_parameter("acc_scale", 0.2)

        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 0.8)
        self.declare_parameter("gripper_effort", 50.0)

        self.declare_parameter("home_joints", [0.0, 0.0, 45.0, 0.0, 115.0, 0.0])
        self.declare_parameter("drop_joints", [40.0, 20.0, 115.0, 0.0, 50.0, 40.0])

        self.declare_parameter("command_cooldown_sec", 2.0)

        # Values
        self.pick_cubes = list(self.get_parameter("pick_cube_frames").value)
        self.place_cube = self.get_parameter("place_cube_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.pick_z = float(self.get_parameter("pick_z_offset").value)
        self.place_z = float(self.get_parameter("place_z_offset").value)

        self.vel_scale = float(self.get_parameter("vel_scale").value)
        self.acc_scale = float(self.get_parameter("acc_scale").value)

        self.gripper_open = float(self.get_parameter("gripper_open_position").value)
        self.gripper_close = float(self.get_parameter("gripper_close_position").value)
        self.gripper_effort = float(self.get_parameter("gripper_effort").value)

        self.home_joints = list(self.get_parameter("home_joints").value)
        self.drop_joints = list(self.get_parameter("drop_joints").value)

        self.cooldown_sec = float(self.get_parameter("command_cooldown_sec").value)
        self.next_command_time = self.get_clock().now()

        # ROS Interfaces
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pose_pub = self.create_publisher(TargetPose, "/target_pose", 10)
        self.target_joint_pub = self.create_publisher(TargetJoint, "/target_joint", 10)

        self.move_status_sub = self.create_subscription(
            GoalStatusArray,
            "/move_action/_action/status",
            self.move_status_cb,
            10,
        )

        self.gripper_client = ActionClient(
            self, GripperCommand, "/gripper_position_controller/gripper_cmd"
        )

        self.pickplace_client = self.create_client(
            PickPlace, "/attach_detach_command"
        )

        # FSM State
        self.state = "MOVE_HOME"
        self.waiting_move = False

        self.cube_idx = 0
        self.current_cube = self.pick_cubes[0]

        self.exit_after_home = False

        # MoveIt goal tracking
        self.last_uuid_seen = None
        self.armed_prev_uuid = None
        self.active_goal_uuid = None

        # Retry (pick yaw)
        self.retry_count = 0
        self.max_retry = 4
        self.retry_yaw_step = math.pi / 2
        self.current_retry_yaw = 0.0

        self.timer = self.create_timer(0.2, self.timer_cb)

        self.get_logger().info(
            f"FSM started | cubes={self.pick_cubes} | place={self.place_cube}"
        )

    # Helper
    def _can_send(self):
        return self.get_clock().now() >= self.next_command_time

    def _cooldown(self, sec=None):
        if sec is None:
            sec = self.cooldown_sec
        self.next_command_time = self.get_clock().now() + Duration(seconds=sec)

    def _arm_move(self):
        self.armed_prev_uuid = self.last_uuid_seen
        self.active_goal_uuid = None
        self.waiting_move = True

    def _reset_move(self):
        self.waiting_move = False
        self.active_goal_uuid = None
        self.armed_prev_uuid = None

    # FSM Timer
    def timer_cb(self):
        if not self._can_send():
            return

        self.get_logger().debug(
            f"[FSM] state={self.state} "
            f"cube={self.current_cube} "
            f"idx={self.cube_idx}/{len(self.pick_cubes)}"
        )

        if self.state == "MOVE_HOME":
            self._arm_move()
            self.send_joint(self.home_joints)
            self._cooldown()
            self.state = "WAIT_MOVE_HOME"

        elif self.state == "OPEN_GRIPPER":
            self.send_gripper(self.gripper_open)
            self._cooldown(1.0)
            self.state = "WAIT_GRIPPER_OPEN"

        elif self.state == "MOVE_PICK":
            if self.send_pose(self.current_cube, self.pick_z, self.current_retry_yaw):
                self._arm_move()
                self._cooldown()
                self.state = "WAIT_MOVE_PICK"

        elif self.state == "MOVE_PLACE":
            if self.send_pose(self.place_cube, self.place_z, 0.0):
                self._arm_move()
                self._cooldown()
                self.state = "WAIT_MOVE_PLACE"

        elif self.state == "MOVE_DROP":
            self._arm_move()
            self.send_joint(self.drop_joints)
            self._cooldown()
            self.state = "WAIT_MOVE_DROP"

        elif self.state == "DO_CLOSE_GRIPPER":
            self.send_gripper(self.gripper_close)
            self._cooldown(1.0)
            self.state = "WAIT_GRIPPER_CLOSE"

        elif self.state == "DO_OPEN_GRIPPER_PLACE":
            self.send_gripper(self.gripper_open)
            self._cooldown(1.0)
            self.state = "WAIT_GRIPPER_PLACE"

        elif self.state == "DO_OPEN_GRIPPER_DROP":
            self.send_gripper(self.gripper_open)
            self._cooldown(1.0)
            self.state = "WAIT_GRIPPER_DROP"

    # Motion
    def send_pose(self, frame, z_offset, yaw):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time()
            )
        except Exception:
            self.get_logger().warn(f"TF not ready: {frame}")
            return False

        q = quaternion_multiply(
            quaternion_multiply(
                [
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ],
                euler_to_quaternion(math.pi, 0, 0),
            ),
            euler_to_quaternion(0, 0, yaw),
        )

        msg = TargetPose()
        msg.x = tf.transform.translation.x
        msg.y = tf.transform.translation.y
        msg.z = tf.transform.translation.z + z_offset
        msg.qx, msg.qy, msg.qz, msg.qw = q
        msg.max_vel_scale = self.vel_scale
        msg.max_acc_scale = self.acc_scale

        self.target_pose_pub.publish(msg)
        return True

    def send_joint(self, joints):
        msg = TargetJoint()
        msg.joints = joints
        msg.max_vel_scale = self.vel_scale
        msg.max_acc_scale = self.acc_scale
        self.target_joint_pub.publish(msg)

    # Gripper
    def send_gripper(self, pos):
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper server not available")
            self.shutdown()
            return

        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = self.gripper_effort

        self.gripper_client.send_goal_async(goal).add_done_callback(
            self._gripper_goal_cb
        )

    def _gripper_goal_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            self.shutdown()
            return
        handle.get_result_async().add_done_callback(self.gripper_result_cb)

    def gripper_result_cb(self, _):
        if self.state == "WAIT_GRIPPER_OPEN":
            self.state = "MOVE_PICK"

        elif self.state == "WAIT_GRIPPER_CLOSE":
            self.state = "MOVE_PLACE"

        elif self.state == "WAIT_GRIPPER_PLACE":
            self._advance_cube()
            self.state = "MOVE_HOME"

        elif self.state == "WAIT_GRIPPER_DROP":
            self._advance_cube()
            self.state = "MOVE_HOME"

    # MoveIt status
    def move_status_cb(self, msg: GoalStatusArray):
        if not msg.status_list or not self.waiting_move:
            return

        latest = msg.status_list[-1]
        self.last_uuid_seen = tuple(latest.goal_info.goal_id.uuid)

        if self.active_goal_uuid is None:
            if self.last_uuid_seen == self.armed_prev_uuid:
                return
            self.active_goal_uuid = self.last_uuid_seen

        status = next(
            (
                s.status
                for s in msg.status_list
                if tuple(s.goal_info.goal_id.uuid) == self.active_goal_uuid
            ),
            None,
        )

        if status != 4:
            return

        self._reset_move()

        if self.state == "WAIT_MOVE_HOME":
            if self.exit_after_home:
                self.shutdown()
            else:
                self.state = "OPEN_GRIPPER"

        elif self.state == "WAIT_MOVE_PICK":
            self.retry_count = 0
            self.call_pickplace(0)
            self._cooldown(3.0)
            self.state = "DO_CLOSE_GRIPPER"

        elif self.state == "WAIT_MOVE_PLACE":
            self.call_pickplace(1)
            self._cooldown(3.0)
            self.state = "DO_OPEN_GRIPPER_PLACE"

        elif self.state == "WAIT_MOVE_DROP":
            self.call_pickplace(1)
            self._cooldown(3.0)
            self.state = "DO_OPEN_GRIPPER_DROP"

    # Pick / Place service
    def call_pickplace(self, motion_type: int):
        if not self.pickplace_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("PickPlace service not available")
            return
        req = PickPlace.Request()
        req.motion_type = motion_type
        self.pickplace_client.call_async(req)

    # Cube advance & shutdown
    def _advance_cube(self):
        self.cube_idx += 1
        if self.cube_idx >= len(self.pick_cubes):
            self.exit_after_home = True
        else:
            self.current_cube = self.pick_cubes[self.cube_idx]

    def shutdown(self):
        self.get_logger().info("FSM finished. Shutting down.")
        self.destroy_node()
        rclpy.shutdown()


# main
def main():
    rclpy.init()
    rclpy.spin(IsaacsimTest())


if __name__ == "__main__":
    main()
