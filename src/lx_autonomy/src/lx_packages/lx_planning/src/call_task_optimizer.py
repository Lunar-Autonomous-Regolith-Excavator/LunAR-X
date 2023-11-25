#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lx_msgs.action import PlanTask
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
import cv2
import yaml

class PlanTaskNode(Node):
    def __init__(self):
        super().__init__('plan_task_node')
        self.action_client = ActionClient(self, PlanTask, 'plan_task')
        self.get_logger().info('Waiting for action server...')

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        self.get_logger().info('Action server is available')

        self.load_parameters()

    def load_parameters(self):
        # Read parameters from YAML file
        yaml_file = '/home/lx_autonomy/lx_autonomy_ws/src/lx_packages/lx_planning/maps/moonyard.yaml'
        with open(yaml_file, 'r') as file:
            params = yaml.safe_load(file)

        # Extract parameters
        berm_input = params['berm_input']
        excavation_input = params['excavation_input']
        berm_height = params['berm_height']
        section_length = params['section_length']
        map_string = params['map_image']

        # Load the map image from lx_planning/maps/mapstring.png
        map_string= "/home/lx_autonomy/lx_autonomy_ws/src/lx_packages/lx_planning/maps/"+map_string
        map_image = cv2.imread(map_string, cv2.IMREAD_GRAYSCALE) # 0 to 255
        
        # put into np int array and shift to -128 to 127
        map_image = map_image.astype(int)
        map_image = map_image - 128
        

        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = 0.05  # Update with your map resolution
        occupancy_grid.info.width = map_image.shape[1]
        occupancy_grid.info.height = map_image.shape[0]
        occupancy_grid.info.origin.position.x = -((map_image.shape[1] - 1) / 2) * occupancy_grid.info.resolution
        occupancy_grid.info.origin.position.y = -((map_image.shape[0] - 1) / 2) * occupancy_grid.info.resolution
        occupancy_grid.data = map_image.flatten().tolist()
        
        # convert berm_input to a list of Points
        berm_input = [Point(x=point['x'], y=point['y'], z=point['z']) for point in berm_input]
        excavation_input = [Point(x=point['x'], y=point['y'], z=point['z']) for point in excavation_input]
        
        # Call the PlanTask action
        self.call_plan_task_action(berm_input, excavation_input, berm_height, section_length, occupancy_grid)

    def call_plan_task_action(self, berm_input, excavation_input, berm_height, section_length, map_data):
        goal_msg = PlanTask.Goal(
            berm_input=berm_input,
            excavation_input=excavation_input,
            berm_height=berm_height,
            section_length=section_length,
            map=map_data
        )

        self.get_logger().info('Sending goal to action server...')
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlanTaskNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
