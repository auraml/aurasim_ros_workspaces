#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive


class MoveToPoseNode(Node):
    """
    Simple node that subscribes to /target_pose and moves the robot arm to that pose
    """

    def __init__(self):
        super().__init__('move_to_pose_node')

        self.get_logger().info('Initializing MoveToPose node...')

        # MoveGroup action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup action server!')

        # Subscribe to target pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        # Track current target and execution state
        self.current_target_pose = None
        self.is_executing = False

        self.get_logger().info('Ready to receive target poses on /target_pose')
        self.get_logger().info('Publish PoseStamped messages to move the arm')

    def _poses_equal(self, pose1: PoseStamped, pose2: PoseStamped, tolerance=0.001):
        """Check if two poses are equal within tolerance"""
        if pose1 is None or pose2 is None:
            return False

        # Check position
        pos_diff = (
            abs(pose1.pose.position.x - pose2.pose.position.x) +
            abs(pose1.pose.position.y - pose2.pose.position.y) +
            abs(pose1.pose.position.z - pose2.pose.position.z)
        )

        # Check orientation
        ori_diff = (
            abs(pose1.pose.orientation.x - pose2.pose.orientation.x) +
            abs(pose1.pose.orientation.y - pose2.pose.orientation.y) +
            abs(pose1.pose.orientation.z - pose2.pose.orientation.z) +
            abs(pose1.pose.orientation.w - pose2.pose.orientation.w)
        )

        return pos_diff < tolerance and ori_diff < tolerance

    def pose_callback(self, msg: PoseStamped):
        """Callback when new target pose is received"""
        # Check if already executing
        if self.is_executing:
            self.get_logger().warn('Still executing previous goal, ignoring new pose')
            return

        # Check if same as current target
        if self._poses_equal(msg, self.current_target_pose):
            self.get_logger().info('Target pose unchanged, ignoring')
            return

        pos = msg.pose.position
        self.get_logger().info(
            f'Received target pose: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] in world frame'
        )

        # Store current target
        self.current_target_pose = msg
        self.is_executing = True

        # Create MoveGroup goal
        goal = self._create_pose_goal(msg)

        # Send goal
        self.get_logger().info('Sending goal to MoveGroup...')
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by MoveGroup')
            self.is_executing = False
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle execution result"""
        result = future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Movement completed successfully!')
        else:
            self.get_logger().error(f'Movement failed with error code: {result.error_code.val}')

        # Reset execution flag
        self.is_executing = False

    def _create_pose_goal(self, target_pose: PoseStamped):
        """Create MoveGroup goal for pose target"""
        goal = MoveGroup.Goal()

        # Request settings
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -2.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0

        goal.request.start_state.is_diff = True
        goal.request.group_name = 'panda_arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        # Always use world frame
        frame_id = 'world'

        # Goal constraints (position + orientation)
        constraints = Constraints()

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.link_name = 'panda_hand'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Define constraint region as small sphere around target
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1mm tolerance
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = 'panda_hand'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
