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

    nav.declare_parameter('goal_x', 4.79)
    nav.declare_parameter('goal_y', 2.61)
    x = nav.get_parameter('goal_x').value
    y = nav.get_parameter('goal_y').value

    # Listen to vision_node's /target_detected topic
    target_seen = False

    def target_callback(msg):
        nonlocal target_seen
        target_seen = msg.data

    nav.create_subscription(Bool, '/target_detected', target_callback, 10)

    nav.waitUntilNav2Active()

    goal = make_pose(nav, x, y)
    print(f'Heading to ({x}, {y})... watching for RED target.')
    nav.goToPose(goal)

    stopped_by_vision = False
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)

        if target_seen:
            print('\nRED TARGET SPOTTED! Stopping robot...')
            nav.cancelTask()
            stopped_by_vision = True
            break

        feedback = nav.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} m', end='\r')

    if stopped_by_vision:
        print('Mission paused: stopped in front of target.')
    else:
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'\nReached ({x}, {y})!')
        else:
            print(f'\nFailed to reach ({x}, {y}).')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
