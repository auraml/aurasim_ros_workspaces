import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose(x, y, yaw_z, yaw_w, navigator):
    """Create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = yaw_z
    pose.pose.orientation.w = yaw_w
    return pose


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()

    # Wait for Nav2 to be active
    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()
    navigator.get_logger().info('Nav2 is active!')

    # Set initial pose (where the robot starts)
    initial_pose = create_pose(0.0, 0.0, 0.0, 1.0, navigator)
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info('Initial pose set')

    # Define 4 waypoints for the route
    waypoints = [
        {'x': 2.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'name': 'Point 1'},
        {'x': 2.0, 'y': 2.0, 'z': 0.7, 'w': 0.7, 'name': 'Point 2'},
        {'x': 0.0, 'y': 2.0, 'z': 1.0, 'w': 0.0, 'name': 'Point 3'},
        {'x': 0.0, 'y': 0.0, 'z': -0.7, 'w': 0.7, 'name': 'Point 4 (Home)'},
    ]

    # Navigate through each waypoint sequentially
    for i, wp in enumerate(waypoints):
        navigator.get_logger().info(f'Navigating to {wp["name"]} ({wp["x"]}, {wp["y"]})')

        goal_pose = create_pose(wp['x'], wp['y'], wp['z'], wp['w'], navigator)
        navigator.goToPose(goal_pose)

        # Wait for task to complete
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                distance = feedback.distance_remaining
                navigator.get_logger().info(f'Distance remaining: {distance:.2f}m')
            rclpy.spin_once(navigator, timeout_sec=0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'Reached {wp["name"]} successfully!')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'Navigation to {wp["name"]} was canceled')
            break
        elif result == TaskResult.FAILED:
            navigator.get_logger().error(f'Failed to reach {wp["name"]}')
            break

    navigator.get_logger().info('Navigation test completed!')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
