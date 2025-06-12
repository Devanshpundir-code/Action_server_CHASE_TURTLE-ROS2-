
#formatted by gemini

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.action import Chase
import math


class ChaseServer(Node):
    def __init__(self):
        super().__init__('chase_server')

        self.action_server = ActionServer(
            self,
            Chase,
            'chaser',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.turtle1_pose = None
        self.turtle2_pose = None

        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)

    def goal_callback(self, goal_request):
        if goal_request.command == 1:
            self.get_logger().info("Goal accepted: Start chasing.")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info("Invalid command. Goal rejected.")
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal canceled.")
        return CancelResponse.ACCEPT

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Chasing started...")

        feedback_msg = Chase.Feedback()
        result = Chase.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Chase cancelled.")
                return Chase.Result(result_msg="Chase cancelled")

            if self.turtle1_pose is None or self.turtle2_pose is None:
                continue

            dx = self.turtle1_pose.x - self.turtle2_pose.x
            dy = self.turtle1_pose.y - self.turtle2_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.5:
                goal_handle.succeed()
                self.cmd_vel_pub.publish(Twist())  # stop
                result.result_msg = "Chase complete"
                return result

            angle_to_goal = math.atan2(dy, dx)
            angle_diff = angle_to_goal - self.turtle2_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            cmd = Twist()
            cmd.linear.x = 2.0 * distance
            cmd.angular.z = 4.0 * angle_diff
            self.cmd_vel_pub.publish(cmd)

            feedback_msg.x = self.turtle2_pose.x
            feedback_msg.y = self.turtle2_pose.y
            goal_handle.publish_feedback(feedback_msg)

            await rclpy.sleep(0.1)

        goal_handle.abort()
        result.result_msg = "Aborted due to unknown issue"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ChaseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
