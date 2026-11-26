import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    # Four in-bounds patrol coordinates around the TurtleBot3 arena
    # Four indoor coordinates from /clicked_point
    waypoints = [
    (-2.20, 2.66),
    (-2.06, -3.38),
    (3.56, -1.14),
    (4.38, -2.80),
]

    state = {'target_detected': False}

    def target_callback(msg):
        state['target_detected'] = msg.data

    nav.create_subscription(Bool, '/target_detected', target_callback, 10)

    nav.get_logger().info('Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 ready! Starting continuous patrol loop.')

    waypoint_idx = 0
    goal_active = False
    paused = False

    try:
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.1)

            # 1. Pause immediately if YOLOv8 spots a person
            if state['target_detected']:
                if goal_active:
                    nav.get_logger().info('PERSON DETECTED! Pausing patrol...')
                    nav.cancelTask()
                    goal_active = False
                paused = True
                continue

            # 2. Resume the interrupted waypoint once the path clears
            if paused:
                nav.get_logger().info('Path cleared! Resuming patrol...')
                paused = False

            # 3. Send the current waypoint if no goal is running
            if not goal_active:
                wx, wy = waypoints[waypoint_idx]
                nav.get_logger().info(
                    f'Heading to waypoint {waypoint_idx + 1}/{len(waypoints)}: ({wx}, {wy})'
                )
                nav.goToPose(make_pose(nav, wx, wy))
                goal_active = True
                continue

            # 4. Advance to the next waypoint when the current one finishes
            if nav.isTaskComplete():
                goal_active = False
                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    nav.get_logger().info(f'Reached waypoint {waypoint_idx + 1}!')
                else:
                    nav.get_logger().warn(
                        f'Waypoint {waypoint_idx + 1} ended with status: {result}'
                    )
                waypoint_idx = (waypoint_idx + 1) % len(waypoints)

    except KeyboardInterrupt:
        nav.get_logger().info('Patrol interrupted by user. Stopping robot...')
        nav.cancelTask()
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()