#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
import time


class PickAndPlaceNode(Node):
    # Planning parameters
    ARM_GROUP = 'panda_arm'
    GRIPPER_GROUP = 'hand'
    END_EFFECTOR_LINK = 'panda_hand'
    BASE_FRAME = 'panda_link0'

    # Motion parameters
    PLANNING_ATTEMPTS = 10
    PLANNING_TIME = 5.0
    VELOCITY_SCALING = 0.1
    ACCELERATION_SCALING = 0.1

    # Gripper positions
    GRIPPER_OPEN = 0.035
    GRIPPER_CLOSED = 0.015

    # Home position joint values
    HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def __init__(self):
        super().__init__('pick_and_place_node')
        self.get_logger().info('Initializing Pick and Place node...')

        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup action server!')

        # Cartesian path service
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        self.get_logger().info('Waiting for Cartesian path service...')
        self.cartesian_path_client.wait_for_service()
        self.get_logger().info('Connected to Cartesian path service!')

        # Execute trajectory action client
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        self.get_logger().info('Waiting for ExecuteTrajectory action server...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('Connected to ExecuteTrajectory action server!')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_state = None

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def _execute_move_group_goal(self, goal_msg, goal_description="move"):
        """
        Common method to execute MoveGroup goal and wait for result
        Returns True if successful, False otherwise
        """
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{goal_description}: Goal rejected!')
            return False

        self.get_logger().info(f'{goal_description}: Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info(f'{goal_description}: Success!')
            return True
        else:
            self.get_logger().error(f'{goal_description}: Failed with error code {result.error_code.val}')
            return False

    def _create_pose_goal(self, group_name, pose_stamped):
        """Create a MoveGroup goal with pose target using constraints (Humble compatible)"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = self.PLANNING_ATTEMPTS
        goal_msg.request.allowed_planning_time = self.PLANNING_TIME
        goal_msg.request.max_velocity_scaling_factor = self.VELOCITY_SCALING
        goal_msg.request.max_acceleration_scaling_factor = self.ACCELERATION_SCALING

        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose_stamped.header.frame_id
        position_constraint.link_name = self.END_EFFECTOR_LINK

        # Use sphere primitive for position tolerance
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm tolerance

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [primitive]
        bounding_volume.primitive_poses = [pose_stamped.pose]
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = pose_stamped.header.frame_id
        orientation_constraint.link_name = self.END_EFFECTOR_LINK
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # Combine constraints
        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False

        return goal_msg

    def _create_joint_goal(self, group_name, joint_names, joint_positions):
        """Create a MoveGroup goal with joint target"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = self.PLANNING_ATTEMPTS
        goal_msg.request.allowed_planning_time = self.PLANNING_TIME
        goal_msg.request.max_velocity_scaling_factor = self.VELOCITY_SCALING
        goal_msg.request.max_acceleration_scaling_factor = self.ACCELERATION_SCALING

        joint_constraints = []
        for name, pos in zip(joint_names, joint_positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = pos
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            joint_constraints.append(constraint)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False
        return goal_msg

    def move_to_pose(self, x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, cartesian=False):
        """Move end effector to specified pose"""
        self.get_logger().info(f'Moving to pose: x={x}, y={y}, z={z}, cartesian={cartesian}')

        if cartesian:
            return self.move_cartesian(x, y, z, qx, qy, qz, qw)

        # Create pose stamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.BASE_FRAME
        pose_stamped.pose.position = Point(x=x, y=y, z=z)
        pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Create and execute goal
        goal_msg = self._create_pose_goal(self.ARM_GROUP, pose_stamped)
        return self._execute_move_group_goal(goal_msg, f"Move to [{x:.3f}, {y:.3f}, {z:.3f}]")

    def move_cartesian(self, x, y, z, qx, qy, qz, qw):
        """Move end effector along Cartesian path"""
        self.get_logger().info(f'Moving via Cartesian path to: x={x}, y={y}, z={z}')

        if self.current_joint_state is None:
            self.get_logger().error('No joint state received yet!')
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = 'panda_link0'
        req.group_name = 'panda_arm'
        req.link_name = 'panda_hand'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        req.start_state = robot_state

        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        req.waypoints = [target_pose]

        future = self.cartesian_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.fraction < 0.99:
            self.get_logger().error(f'Cartesian path planning failed: fraction={response.fraction}')
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = response.solution

        future = self.execute_trajectory_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Cartesian execution goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Cartesian motion successful!')
            return True
        else:
            self.get_logger().error(f'Cartesian execution failed: {result.error_code.val}')
            return False

    def move_gripper(self, position):
        """
        Move gripper to specified position
        position: 0.0 = closed, 0.035 = open
        """
        state_name = 'closed' if position < 0.02 else 'open'
        self.get_logger().info(f'Moving gripper: {state_name} (position={position})')

        joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        joint_positions = [position, position]

        goal_msg = self._create_joint_goal(self.GRIPPER_GROUP, joint_names, joint_positions)
        return self._execute_move_group_goal(goal_msg, f"Gripper {state_name}")

    def open_gripper(self):
        """Open gripper"""
        return self.move_gripper(self.GRIPPER_OPEN)

    def close_gripper(self):
        """Close gripper"""
        return self.move_gripper(self.GRIPPER_CLOSED)

    def go_home(self):
        """Move to home (ready) position"""
        self.get_logger().info('Moving to home position...')

        joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        goal_msg = self._create_joint_goal(self.ARM_GROUP, joint_names, self.HOME_JOINTS)
        return self._execute_move_group_goal(goal_msg, "Move to home")

    def execute_pick_and_place(self, pick_pos=(0.273, 0.433, 0.127),
                                place_pos=(0.070, -0.649, 0.4),
                                approach_height=0.2):
        """
        Execute complete pick and place sequence

        Args:
            pick_pos: (x, y, z) tuple for pick location
            place_pos: (x, y, z) tuple for place location
            approach_height: z height for approach/retreat moves
        """
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting Pick and Place sequence')
        self.get_logger().info('=' * 50)

        # Define sequence steps
        steps = [
            ('Moving to pre-pick position',
             lambda: self.move_to_pose(pick_pos[0], pick_pos[1], approach_height)),

            ('Opening gripper', self.open_gripper),

            ('Moving down to grasp object',
             lambda: self.move_to_pose(*pick_pos, cartesian=True)),

            ('Closing gripper', self.close_gripper),

            ('Lifting object',
             lambda: self.move_to_pose(pick_pos[0], pick_pos[1], approach_height, cartesian=True)),

            ('Moving to place position',
             lambda: self.move_to_pose(*place_pos)),

            ('Releasing object', self.open_gripper),

            ('Returning home', self.go_home),
        ]

        # Execute steps
        for i, (description, action) in enumerate(steps, 1):
            self.get_logger().info(f'Step {i}/{len(steps)}: {description}...')
            if not action():
                self.get_logger().error(f'Failed at step {i}: {description}')
                return False
            time.sleep(0.5)  # Brief pause between steps

        self.get_logger().info('=' * 50)
        self.get_logger().info('Pick and Place completed successfully!')
        self.get_logger().info('=' * 50)
        return True


def main(args=None):
    rclpy.init(args=args)

    node = PickAndPlaceNode()

    try:
        success = node.execute_pick_and_place()

        if success:
            node.get_logger().info('Task completed successfully!')
        else:
            node.get_logger().error('Task failed!')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
