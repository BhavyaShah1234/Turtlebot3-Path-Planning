import math
import rclpy as r
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class MissionExecutor(Node):
    def __init__(self):
        super(MissionExecutor, self).__init__(node_name='mission_executor')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)

    def callback(self, location_message):
        self.get_logger().info(f'Current Location: {location_message}')

    def rpy_to_quatornions(self, roll, pitch, yaw):
        sin_roll = math.sin(roll / 2)
        cos_roll = math.cos(roll / 2)
        sin_pitch = math.sin(pitch / 2)
        cos_pitch = math.cos(pitch / 2)
        sin_yaw = math.sin(yaw / 2)
        cos_yaw = math.cos(yaw / 2)
        x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
        y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
        z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
        w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw
        return x, y, z, w

    def send_goal(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        destination_message = PoseStamped()
        destination_message.header.frame_id = 'map'
        destination_message.header.stamp = self.get_clock().now().to_msg()
        destination_message.pose.position.x = x
        destination_message.pose.position.y = y
        destination_message.pose.position.z = z
        x, y, z, w = self.rpy_to_quatornions(roll, pitch, yaw)
        destination_message.pose.orientation.x = x
        destination_message.pose.orientation.y = y
        destination_message.pose.orientation.z = z
        destination_message.pose.orientation.w = w
        goal_message = NavigateToPose.Goal()
        goal_message.pose = destination_message
        while not self.action_client.wait_for_server(10.0):
            self.get_logger().info("Waiting for server....")
        promise = self.action_client.send_goal_async(goal_message, self.feedback_callback)
        promise.add_done_callback(self.final_callback)
        return promise

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback.feedback}")

    def final_callback(self, promise):
        goal_message = promise.result()
        if not goal_message.accepted:
            self.get_logger().info("Goal rejected")
        self.get_logger().info("Goal accepted")
        goal_message.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, promise):
        result = promise.result()
        if result.status == 4:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Error: {result.status}')
        self.get_logger().info(f'Result: {result}')
        return result

def main(args=None):
    r.init(args=args)
    node = MissionExecutor()
    node.send_goal(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
