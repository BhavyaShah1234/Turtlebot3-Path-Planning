import rclpy as r
from rclpy.node import Node
from path_planner_msg.srv import Mission
from mission_executor import MissionExecutor

class MissionPlanner(Node):
    def __init__(self):
        super(MissionPlanner, self).__init__(node_name='mission_planner')
        self.server = self.create_service(Mission, '/mission_planner_service', self.callback)
        self.mission_executor = MissionExecutor()

    def callback(self, request, response):
        x, y, z, roll, pitch, yaw = request.destination_point
        self.mission_executor.send_goal(x, y, z, roll, pitch, yaw)
