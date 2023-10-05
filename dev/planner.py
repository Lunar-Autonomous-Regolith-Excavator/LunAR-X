import numpy as np
from scipy.ndimage import gaussian_filter, rotate
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

# Global variables
dt = 1e-2
angle_of_repose = 30 # degrees
next_berm_width = 60 # cms

# class for Robot
class Robot:
    # Constants
    WIDTH = 70
    LENGTH = 100
    
    LENGTH_TOOL = 50
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
    
    def actuate_tool(self, v):
        global dt
        self.tool_height += np.sign(v) * Robot.TOOL_VEL * dt
        return np.clip(self.tool_height, Robot.MIN_TOOL_HEIGHT, Robot.MAX_TOOL_HEIGHT)

def visualize(fig, ax, height_grid, robot, path):
    # visualize the height map with legend with y axis inverted
    plt.imshow(height_grid.T, cmap='viridis', origin='lower')
    plt.colorbar(label='Value')

    # visualize the robot as rectangle
    corners = robot.get_corners()
    rectangle = patches.Polygon(corners, color='r')
    ax.add_patch(rectangle)
    # visualize the tool
    corners_tool = robot.get_corners_tool()
    tool = patches.Polygon(corners_tool, color='r')
    ax.add_patch(tool)
    # join robot and tool
    plt.plot([robot.x, robot.tool_x], [robot.y, robot.tool_y], 'r')

    # visualize the path
    plt.plot(path[:-100, 0], path[:-100, 1], 'b')
    plt.show()


def update_occ_grid(occ_grid, height_grid):
    # update the occupancy grid based on the height map
    occ_grid[height_grid > 5] = 1
    return occ_grid

def get_berm(theta: float, length: int, height: int):
    global angle_of_repose
    length += (length % 2) # make length even
    radius = int(np.ceil(height / np.tan(np.deg2rad(angle_of_repose))))
    width = 2 * radius # even 
    total_length = width + length # even
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




if __name__ == '__main__':

    desired_berm_height = 15 # cms
    
    # Set a seed for reproducibility
    # np.random.seed(42)

    # visualization
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Generate a 70 by 70 array with random height values
    height_grid = np.random.rand(70, 70)
    height_grid = gaussian_filter(height_grid, sigma=5)
    height_grid = np.kron(height_grid, np.ones((10, 10))) # resize the height map to 700 by 700
    height_grid = height_grid - np.min(height_grid) # make the minimum height zero

    # generate occupancy grid if height is more than 5 cms
    occupancy_grid = np.zeros(height_grid.shape)
    occupancy_grid = update_occ_grid(occupancy_grid, height_grid)

    # generate a robot
    robot = Robot(100, 350, np.deg2rad(90))

    # generate a berm
    berm = get_berm(robot.theta, Robot.TOOL_WIDTH, desired_berm_height)
    lx, ly = berm.shape
    min_x = int(robot.tool_x) - lx//2
    min_y = int(robot.tool_y) - ly//2
    max_x = min_x + lx
    max_y = min_y + ly

    # overlay the berm on the height map
    height_grid[min_x:max_x, min_y:max_y] = np.maximum(height_grid[min_x:max_x, min_y:max_y], berm)

    for i in range(1, 5):
        robot = Robot(100 + i * next_berm_width, 350, np.deg2rad(90))

        # generate a berm
        berm = get_berm(robot.theta, Robot.TOOL_WIDTH, desired_berm_height)
        lx, ly = berm.shape
        min_x = int(robot.tool_x) - lx//2
        min_y = int(robot.tool_y) - ly//2
        max_x = min_x + lx
        max_y = min_y + ly

        # overlay the berm on the height map
        height_grid[min_x:max_x, min_y:max_y] = np.maximum(height_grid[min_x:max_x, min_y:max_y], berm)

    
    robot = Robot(100, 150, np.deg2rad(90))


    # visualize the robot
    visualize(fig, ax, height_grid, robot, np.array([[robot.x, robot.y]]))