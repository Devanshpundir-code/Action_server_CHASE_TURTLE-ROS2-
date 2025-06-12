import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import Chase


class ChaseClient(Node):
    def __init__(self):
        super().__init__("chase_client_node")
        self.client = ActionClient(self, Chase, "chaser")

    def send_goal(self, command_value):
        # Wait for server to be ready
        self.client.wait_for_server()

        # Create goal message properly
        goal_msg = Chase.Goal()
        goal_msg.command = command_value

        self.get_logger().info("Sending goal to start chasing...")

        # Send goal and register callbacks
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Chasing request was rejected.")
            return

        self.get_logger().info("Chasing request accepted.")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result.status}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback received: x={feedback.x}, y={feedback.y}")


def main(args=None):
    rclpy.init(args=args)
    node = ChaseClient()
    node.send_goal(1)  # 1 = command to start chasing
    rclpy.spin(node)
    rclpy.shutdown()
