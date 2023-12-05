import numpy as np
from scipy.ndimage import gaussian_filter, rotate
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter  # For gif

from ament_index_python.packages import get_package_share_directory
import yaml
import cv2
import sys

# visualization
fig = plt.figure()
ax = plt.gca()
ax.set_aspect('equal')

height_grid_arr = []
corners_arr = []
corners_tool_arr = []
robot_pose_arr = []
path_arr = []
berm_pts_arr = []
task_arr = []
excavation_arr = []

# Task types
task_types = ['NAVIGATION', 'EXCAVATION', 'DUMP']

# Global variables
dt = 1e-1
angle_of_repose = 30 # degrees

# class for Robot
class Robot:
    # Constants
    WIDTH = 70
    LENGTH = 100
    
    LENGTH_TOOL = 35
    TOOL_POS = LENGTH/2 + LENGTH_TOOL
    TOOL_RAD = 10
    TOOL_WIDTH = 42
    MAX_TOOL_HEIGHT = 45
    MIN_TOOL_HEIGHT = 0
    
    MAX_VEL = 50
    TOOL_VEL = 20

    WHEEL_WIDTH = 13
    WHEEL_DELTA_X = 54
    WHEEL_DELTA_Y = 54

    corners_base = np.zeros((4, 2))
    corners_base[0, :] = [-LENGTH/2, -WIDTH/2]
    corners_base[1, :] = [LENGTH/2, -WIDTH/2]
    corners_base[2, :] = [LENGTH/2, WIDTH/2]
    corners_base[3, :] = [-LENGTH/2, WIDTH/2]

    corners_wt = np.zeros((4, 2))
    corners_wt[0, :] = [-LENGTH/2, -WIDTH/2]
    corners_wt[1, :] = [TOOL_POS, -WIDTH/2]
    corners_wt[2, :] = [TOOL_POS, WIDTH/2]
    corners_wt[3, :] = [-LENGTH/2, WIDTH/2]

    corners_tool = np.zeros((4, 2))
    corners_tool[0, :] = [-TOOL_RAD, -TOOL_WIDTH/2]
    corners_tool[1, :] = [TOOL_RAD, -TOOL_WIDTH/2]
    corners_tool[2, :] = [TOOL_RAD, TOOL_WIDTH/2]
    corners_tool[3, :] = [-TOOL_RAD, TOOL_WIDTH/2]

    corners_wheel = np.zeros((4, 2))
    corners_wheel[0, :] = [-WHEEL_DELTA_X/2, -WHEEL_DELTA_Y/2]
    corners_wheel[1, :] = [WHEEL_DELTA_X/2, -WHEEL_DELTA_Y/2]
    corners_wheel[2, :] = [WHEEL_DELTA_X/2, WHEEL_DELTA_Y/2]
    corners_wheel[3, :] = [-WHEEL_DELTA_X/2, WHEEL_DELTA_Y/2]

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.rot_mat = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        self.tool_x = self.x + Robot.TOOL_POS * np.cos(self.theta)
        self.tool_y = self.y + Robot.TOOL_POS * np.sin(self.theta)
        self.tool_height = Robot.MIN_TOOL_HEIGHT

    def get_corners(self):
        return np.matmul(self.rot_mat, Robot.corners_base.T).T + np.array([[self.x, self.y]])

    def get_corners_wt(self):
        return np.matmul(self.rot_mat, Robot.corners_wt.T).T + np.array([[self.x, self.y]])

    def get_corners_tool(self):
        return np.matmul(self.rot_mat, Robot.corners_tool.T).T + np.array([[self.tool_x, self.tool_y]])
    
    def get_wheel_pos(self):
        return np.matmul(self.rot_mat, Robot.corners_wheel.T).T + np.array([[self.x, self.y]])
    
    def move(self, v, w):
        global dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        self.rot_mat = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        self.tool_x = self.x + Robot.TOOL_POS * np.cos(self.theta)
        self.tool_y = self.y + Robot.TOOL_POS * np.sin(self.theta)
    
    def shift(self, dx, dy, dtheta):
        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.rot_mat = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        self.tool_x = self.x + Robot.TOOL_POS * np.cos(self.theta)
        self.tool_y = self.y + Robot.TOOL_POS * np.sin(self.theta)
    
    def actuate_tool(self, v):
        global dt
        self.tool_height += np.sign(v) * Robot.TOOL_VEL * dt
        return np.clip(self.tool_height, Robot.MIN_TOOL_HEIGHT, Robot.MAX_TOOL_HEIGHT)

def visualize(berm_input_pts, height_grid, robot, path):
    # visualize the height map with legend with y axis inverted
    plt.imshow(height_grid.T, cmap='viridis', origin='lower')
    plt.colorbar(label='Value')

    # visualize the robot as rectangle
    corners = robot.get_corners()
    rectangle = patches.Polygon(corners, color='r')
    plt.gca().add_patch(rectangle)
    # visualize the tool
    corners_tool = robot.get_corners_tool()
    tool = patches.Polygon(corners_tool, color='r')
    plt.gca().add_patch(tool)
    # join robot and tool
    plt.plot([robot.x, robot.tool_x], [robot.y, robot.tool_y], 'r')

    # visualize the path
    plt.plot(path[:-100, 0], path[:-100, 1], 'b')

    # plot berm input points and connect them
    plt.plot(berm_input_pts[:, 0], berm_input_pts[:, 1], 'bo-')

    plt.show()

def getFrame(i, berm_pts_arr, excavation_pts_arr, height_grid_arr, corners_arr, corners_tool_arr, robot_pose_arr, path_arr, task_arr):
    artists = []

    plt.gca().cla()
    
    # visualize the height map with legend with y axis inverted
    artists.append(plt.imshow(height_grid_arr[i].T, cmap='viridis', origin='lower'))
    # artists.append(plt.colorbar(label='Value'))

    # visualize the robot as rectangle
    artists.append(plt.gca().add_patch(patches.Polygon(corners_arr[i], color='r')))
    # visualize the tool
    artists.append(plt.gca().add_patch(patches.Polygon(corners_tool_arr[i], color='r')))
    # join robot and tool
    artists.append(plt.plot([robot_pose_arr[i][0], robot_pose_arr[i][2]], [robot_pose_arr[i][1], robot_pose_arr[i][3]], 'r'))

    # visualize the path
    artists.append(plt.plot(path_arr[i][:, 0], path_arr[i][:, 1], 'b'))

    # plot berm input points and connect them
    artists.append(plt.plot(berm_pts_arr[i][:, 0], berm_pts_arr[i][:, 1], 'bo-'))

    # plot excavation input points
    for pt in excavation_arr[i]:
        x, y, t = pt
        # Plot as arrow
        artists.append(plt.arrow(x, y, 100 * np.cos(t), 100 * np.sin(t), width=2, color='r'))

    # Add task as title
    artists.append(plt.title('Task: ' + task_arr[i]))
    
    return artists

def get_berm(theta: float, length: int, height: int):
    global angle_of_repose
    length += (length % 2) # make length even
    radius = int(np.ceil(height / np.tan(np.deg2rad(angle_of_repose))))
    width = 2 * radius # even 
    total_length = width + length # even
    # total_length = length # even
    # length = total_length - width # even

    berm_grid = np.zeros((width, total_length))
    
    X, Y = np.meshgrid(np.arange(-radius, radius), np.arange(-radius, radius))
    # Calculate distance from the center
    distance = np.sqrt(X**2 + Y**2)
    # Linearly decreasing values from the center outward
    circle = np.clip((radius - distance) * np.tan(np.deg2rad(angle_of_repose)), 0, height)

    berm_grid[:, :width] = circle.copy()
    berm_grid[:, total_length - width:] = circle.copy()

    X, Y = np.meshgrid(np.arange(-length//2, length//2), np.arange(-radius, radius))
    distance = np.abs(Y)
    rect = np.clip((radius - distance) * np.tan(np.deg2rad(angle_of_repose)), 0, height)

    berm_grid[:, radius + 1 : radius + 1 + length] = rect.copy()

    berm_grid = rotate(berm_grid, np.rad2deg(theta), reshape=True)

    # # Plot the grid with the decreasing values
    # plt.imshow(berm_grid.T, cmap='viridis', origin='lower')
    # plt.colorbar(label='Value')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.gca().set_aspect('equal', adjustable='box')
    # plt.show()

    return berm_grid

def overlay_berm(berm, berm_x, berm_y, height_grid):
    lx, ly = berm.shape
    min_x = int(berm_x) - lx//2
    min_y = int(berm_y) - ly//2
    max_x = min_x + lx
    max_y = min_y + ly

    # overlay the berm on the height map
    height_grid[min_x:max_x, min_y:max_y] = np.maximum(height_grid[min_x:max_x, min_y:max_y], berm)

    return height_grid

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python animate.py <map_yaml_file>')
        sys.exit(1)
        
    map_yaml_file = sys.argv[1]
    # desired_berm_height = 15 # cms
    # berm_section_length = 40 # cms
    
    # Set a seed for reproducibility
    # np.random.seed(42)

    # Read parameters from YAML file
    package_directory = get_package_share_directory("lx_planning")
    yaml_file = package_directory + '/maps/' + map_yaml_file + '.yaml'
    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)

    # Extract parameters
    desired_berm_height = int(params['berm_height'] * 100)
    berm_section_length = int(params['section_length'] * 100)
    berm_input = params['berm_input']
    berm_points = np.array([[pt['x'], pt['y']] for pt in berm_input]) * 100
    excavation_input = params['excavation_input']
    excavation_points = np.array([[pt['x'] * 100, pt['y'] * 100, pt['z']] for pt in excavation_input])

    # read output CSV
    output_points = np.genfromtxt(package_directory + '/animation/output.csv', delimiter=',')

    map_string = params['map_image']
    map_string= package_directory + '/maps/' + map_string
    map_image = (255 - cv2.imread(map_string, cv2.IMREAD_GRAYSCALE))
    map_image[map_image < 128] = 0
    map_image[map_image >= 128] = 25

    # scale the map by 5 times
    height_grid = np.kron(map_image, np.ones((5, 5)))
    # rotate clockwise by 90 degrees
    height_grid = np.rot90(height_grid, k=3)

    # generate a robot
    _, init_x, init_y, init_theta = output_points[0]
    robot = Robot(init_x * 100, init_y * 100, np.deg2rad(init_theta))

    nav_count = 0
    
    robot_path = []
    for out in output_points:
        task, x, y, theta = out
        task = int(task)
        print(task, x, y, theta)
        # convert to cms
        x *= 100
        y *= 100
        # convert to radians
        theta = np.deg2rad(theta)
        
        if task == 0: # navigation
            # read path from file
            path = np.genfromtxt(package_directory + '/paths/path_' + str(nav_count) + '.txt', delimiter=',')
            path[:, :2] *= 100

            for i in range(len(path)):
                dx = path[i, 0] - robot.x
                dy = path[i, 1] - robot.y
                if i == len(path)-1:
                    dtheta = path[i, 2] - robot.theta
                else:
                    dtheta = 0

                robot_path.append([robot.x, robot.y])
                height_grid_arr.append(height_grid.copy())
                corners_arr.append(robot.get_corners())
                corners_tool_arr.append(robot.get_corners_tool())
                robot_pose_arr.append([robot.x, robot.y, robot.tool_x, robot.tool_y])
                path_arr.append(np.array(robot_path)[-10:, :])
                berm_pts_arr.append(berm_points.copy())
                excavation_arr.append(excavation_points.copy())
                task_arr.append(task_types[task])

                robot.shift(dx, dy, dtheta)
            
            nav_count += 1
        
        elif task == 1: # excavation
            dx = x + 50 * np.cos(robot.theta) - robot.x
            dy = y + 50 * np.sin(robot.theta) - robot.y
            dtheta = theta - robot.theta
            for i in range(0, 30):
                robot_path.append([robot.x, robot.y])
                height_grid_arr.append(height_grid.copy())
                corners_arr.append(robot.get_corners())
                corners_tool_arr.append(robot.get_corners_tool())
                robot_pose_arr.append([robot.x, robot.y, robot.tool_x, robot.tool_y])
                path_arr.append(np.array(robot_path)[-10:, :])
                berm_pts_arr.append(berm_points.copy())
                excavation_arr.append(excavation_points.copy())
                task_arr.append(task_types[task])

                robot.shift(dx * dt, dy * dt, dtheta * dt)
        
        elif task == 2: # dump
            berm = get_berm(robot.theta, berm_section_length, desired_berm_height)
            height_grid = overlay_berm(berm, robot.tool_x, robot.tool_y, height_grid)
            for i in range(0, 50):
                if i == 45:
                    robot.shift(0, 0, -robot.theta)
                    dtheta = 0
                robot_path.append([robot.x, robot.y])
                height_grid_arr.append(height_grid.copy())
                corners_arr.append(robot.get_corners())
                corners_tool_arr.append(robot.get_corners_tool())
                robot_pose_arr.append([robot.x, robot.y, robot.tool_x, robot.tool_y])
                path_arr.append(np.array(robot_path)[-10:, :])
                berm_pts_arr.append(berm_points.copy())
                excavation_arr.append(excavation_points.copy())
                task_arr.append(task_types[task])


    # # visualize the robot
    # # visualize(berm_points, height_grid, robot, np.array(robot_path))

    animation = FuncAnimation(fig, getFrame, frames=len(height_grid_arr), fargs=(berm_pts_arr, excavation_arr, height_grid_arr, corners_arr, corners_tool_arr, robot_pose_arr, path_arr, task_arr))
    # animation.save('bags/animation.gif', writer='pillow', fps=10)
    animation.save('bags/animation.mp4', writer='ffmpeg', fps=30)

    # save the height map in png
    plt.imshow(height_grid.T, cmap='viridis', origin='lower')
    # colorbar bottom
    plt.colorbar(label='Value', orientation='horizontal')
    plt.savefig('bags/height_map.png')