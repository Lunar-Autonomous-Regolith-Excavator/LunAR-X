#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lx_msgs.action import PlanTask
from lx_msgs.msg import PlannedTask
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
import yaml
from numpy import arctan2, rad2deg
import cv2

from ament_index_python.packages import get_package_share_directory

class PlanTaskNode(Node):
    def __init__(self):
        super().__init__('plan_task_node')
        self._action_client = ActionClient(self, PlanTask, 'plan_task')
        
        self.get_logger().info('Waiting for action server...')
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        self.get_logger().info('Action server is available')

        self.load_parameters()

    def load_parameters(self):
        # Read parameters from YAML file
        package_directory = get_package_share_directory("lx_planning")

        yaml_file = package_directory + '/maps/test.yaml'
        with open(yaml_file, 'r') as file:
            params = yaml.safe_load(file)

        # Extract parameters
        self.berm_input = params['berm_input']
        self.excavation_input = params['excavation_input']
        self.berm_height = params['berm_height']
        self.section_length = params['section_length']

        self.output_file = params['output_file']

        # Load the map image from lx_planning/maps/mapstring.png
        map_string = params['map_image']
        map_string= package_directory + "/maps/" + map_string
        map_image = cv2.imread(map_string, cv2.IMREAD_GRAYSCALE) # 0 to 255
        
        # put into np int array and shift to -128 to 127
        map_image = map_image.astype(int)
        map_image = map_image - 128

        # Create OccupancyGrid message
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.resolution = 0.05  # Update with your map resolution
        self.occupancy_grid.info.width = map_image.shape[1]
        self.occupancy_grid.info.height = map_image.shape[0]
        self.occupancy_grid.info.origin.position.x = -((map_image.shape[1] - 1) / 2) * self.occupancy_grid.info.resolution
        self.occupancy_grid.info.origin.position.y = -((map_image.shape[0] - 1) / 2) * self.occupancy_grid.info.resolution
        self.occupancy_grid.data = map_image.flatten().tolist()
        
        # convert berm_input to a list of Points
        self.berm_input = [Point(x=point['x'], y=point['y']) for point in self.berm_input]
        self.excavation_input = [Point(x=point['x'], y=point['y'], z=point['z']) for point in self.excavation_input]

    def call_plan_task_action(self):
        goal_msg = PlanTask.Goal(
            berm_input=self.berm_input,
            excavation_input=self.excavation_input,
            berm_height=self.berm_height,
            section_length=self.section_length,
            map=self.occupancy_grid
        )

        self.get_logger().info('Sending goal to action server...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self.get_logger().info('Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        plan = future.result().result.plan
        self.get_logger().info('Result received! Plan length: {0}'.format(len(plan)))

        # Write plan to csv file
        self.get_logger().info('Writing plan to file: {0}'.format(self.output_file))
        with open(self.output_file, mode='w') as plan_file:
            for i in range(len(plan)):
                quat = plan[i].pose.orientation
                yaw = rad2deg(arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)))
                line = '{0}, {1}, {2}, {3}'.format(plan[i].task_type, plan[i].pose.position.x, plan[i].pose.position.y, yaw)
                plan_file.write(line + '\n')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = PlanTaskNode()
    action_client.call_plan_task_action()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
