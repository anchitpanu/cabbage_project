import rclpy
from rclpy.node import Node
from enum import Enum


from cabbagerobot_interfaces.msg import PlantingOrder, DetectionResult
from cabbagerobot_interfaces.action import MoveDistance
from rclpy.action import ActionClient


class State(Enum):
    INIT = 0
    PLANTING = 1
    DETECTION = 2
    DONE = 3


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.state = State.INIT
        self.plan = []
        self.index = 0


        self.move_client = ActionClient(self, MoveDistance, 'move_distance')
        self.create_subscription(PlantingOrder, 'planting_order', self.order_cb, 10)
        self.create_subscription(DetectionResult, 'detection_result', self.detect_cb, 10)


    def order_cb(self, msg):
        self.plan = msg.distances_cm
        self.state = State.PLANTING
        self.index = 0
        self.execute_next()


    def execute_next(self):
        if self.index >= len(self.plan):
            self.state = State.DETECTION
            self.get_logger().info('Switching to detection zone')
            return


        goal = MoveDistance.Goal()
        goal.distance_cm = self.plan[self.index]
        self.move_client.send_goal_async(goal)
        self.index += 1


    def detect_cb(self, msg):
        self.get_logger().info(f'Size={msg.size_cm} ready={msg.ready_to_harvest}')




def main():
    rclpy.init()
    node = TaskManager()
    rclpy.spin(node)
    rclpy.shutdown()
