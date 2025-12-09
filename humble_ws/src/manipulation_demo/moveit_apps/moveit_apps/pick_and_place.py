#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
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
from std_msgs.msg import Header
import time


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        self.get_logger().info('Initializing Pick and Place node...')

        # MoveGroup action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup action server!')

        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        self.get_logger().info('Waiting for Cartesian path service...')
        self.cartesian_path_client.wait_for_service()
        self.get_logger().info('Connected to Cartesian path service!')

        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        self.get_logger().info('Waiting for ExecuteTrajectory action server...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('Connected to ExecuteTrajectory action server!')

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

    def move_to_pose(self, x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, cartesian=False):
        """Move end effector to specified pose"""
        self.get_logger().info(f'Moving to pose: x={x}, y={y}, z={z}, cartesian={cartesian}')

        if cartesian:
            return self.move_cartesian(x, y, z, qx, qy, qz, qw)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'panda_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Set pose constraint
        pose_constraint = PositionConstraint()
        pose_constraint.header.frame_id = 'panda_link0'
        pose_constraint.link_name = 'panda_hand'

        # Position constraint
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [primitive]
        bounding_volume.primitive_poses = [pose]

        pose_constraint.constraint_region = bounding_volume
        pose_constraint.weight = 1.0

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'panda_link0'
        orientation_constraint.link_name = 'panda_hand'
        orientation_constraint.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pose_constraint]
        constraints.orientation_constraints = [orientation_constraint]

        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False

        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Successfully moved to pose!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
            return False

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
        state_name = 'close1' if position < 0.01 else ('open' if position > 0.03 else 'close2')
        self.get_logger().info(f'Moving gripper to: {state_name} (position={position})')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'hand'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Set joint constraints for gripper
        joint_constraint_1 = JointConstraint()
        joint_constraint_1.joint_name = 'panda_finger_joint1'
        joint_constraint_1.position = position
        joint_constraint_1.tolerance_above = 0.001
        joint_constraint_1.tolerance_below = 0.001
        joint_constraint_1.weight = 1.0

        joint_constraint_2 = JointConstraint()
        joint_constraint_2.joint_name = 'panda_finger_joint2'
        joint_constraint_2.position = position
        joint_constraint_2.tolerance_above = 0.001
        joint_constraint_2.tolerance_below = 0.001
        joint_constraint_2.weight = 1.0

        constraints = Constraints()
        constraints.joint_constraints = [joint_constraint_1, joint_constraint_2]

        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False

        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False

        self.get_logger().info('Gripper goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Gripper moved successfully!')
            return True
        else:
            self.get_logger().error(f'Gripper motion failed with error code: {result.error_code.val}')
            return False

    def go_home(self):
        """Move to home (ready) position"""
        self.get_logger().info('Moving to home position...')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'panda_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Use "ready" named state
        joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        ready_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        joint_constraints = []
        for name, pos in zip(joint_names, ready_positions):
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

        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Home goal rejected!')
            return False

        self.get_logger().info('Home goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Successfully moved to home!')
            return True
        else:
            self.get_logger().error(f'Home motion failed with error code: {result.error_code.val}')
            return False

    def execute_pick_and_place(self):
        """Execute complete pick and place sequence"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting Pick and Place sequence')
        self.get_logger().info('=' * 50)

        self.get_logger().info('Step 1: Moving to pre-pick position...')
        if not self.move_to_pose(x=0.273, y=0.433, z=0.2):
            self.get_logger().error('Failed to move to pre-pick position')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 2: Opening gripper...')
        if not self.move_gripper(0.035):
            self.get_logger().error('Failed to open gripper')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 3: Moving down to grasp cube...')
        if not self.move_to_pose(x=0.273, y=0.433, z=0.127, cartesian=True):
            self.get_logger().error('Failed to move down')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 4: Closing gripper...')
        if not self.move_gripper(0.015):
            self.get_logger().error('Failed to close gripper')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 5: Lifting to pre-grasp height...')
        if not self.move_to_pose(x=0.273, y=0.433, z=0.2, cartesian=True):
            self.get_logger().error('Failed to lift to pre-grasp')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 6: Lifting cube higher...')
        if not self.move_to_pose(x=0.273, y=0.433, z=0.4):
            self.get_logger().error('Failed to lift')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 7: Moving to drop position...')
        if not self.move_to_pose(x=0.070, y=-0.649, z=0.4):
            self.get_logger().error('Failed to move to drop position')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 8: Opening gripper (dropping cube)...')
        if not self.move_gripper(0.035):
            self.get_logger().error('Failed to open gripper')
            return False
        time.sleep(1.0)

        self.get_logger().info('Step 9: Returning home...')
        if not self.go_home():
            self.get_logger().error('Failed to go home')
            return False

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
