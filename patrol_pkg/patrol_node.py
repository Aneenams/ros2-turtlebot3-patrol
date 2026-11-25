import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def make_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0  # Face forward (0 degrees)
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Declare parameters with default values
    nav.declare_parameter('goal_x', 4.79)
    nav.declare_parameter('goal_y', 2.61)

    x = nav.get_parameter('goal_x').value
    y = nav.get_parameter('goal_y').value

    nav.waitUntilNav2Active()

    goal = make_pose(nav, x, y)
    print(f'Heading to ({x}, {y})...')
    nav.goToPose(goal)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} m', end='\r')

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'\nReached ({x}, {y})!')
    else:
        print(f'\nFailed to reach ({x}, {y}).')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
