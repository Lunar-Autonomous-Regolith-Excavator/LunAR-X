import numpy as np
import matplotlib.pyplot as plt
import yaml

def generate_circle_points(cx, cy, r, dtheta, offset_theta, max_theta=360):
    points = []
    for i in range(0, int(max_theta/dtheta)+1):
        x = float(cx + r * np.cos(np.deg2rad(i * dtheta + offset_theta)))
        y = float(cy + r * np.sin(np.deg2rad(i * dtheta + offset_theta)))
        points.append({'x': x, 'y': y})
    return points

# env_width_dict = {1.25: 3, 1.5: 4, 2.0: 5, 1.2: 3}
circle_radius = 1.25
env_width = 4
section_length = 0.4
section_angle = np.rad2deg(np.arccos(1 - (section_length**2)/(2 * circle_radius**2)))

# generate points
num_berm_points = 19
berm_inputs = generate_circle_points(6, env_width/2, circle_radius, section_angle, -num_berm_points/2 * section_angle, num_berm_points * section_angle)
# berm_inputs = generate_circle_points(5.5, env_width - 0.5, circle_radius, section_angle, 180, 190)
# num_berm_points = len(berm_inputs)

# plot points
plt.plot([x['x'] for x in berm_inputs], [x['y'] for x in berm_inputs], 'ro-')

# excavation points
num_exc_pts = int(env_width / 0.5)
num_exc_repeat = int(np.ceil(num_berm_points / (num_exc_pts-1) ))
# middle excavation points
mid_exc_idx = int(num_exc_pts / 2)
exc_pts = []
for _ in range(num_exc_repeat):
    for i in range(mid_exc_idx, num_exc_pts):
        exc_pts.append({'x': 1.0, 'y': i * 0.5, 'z': 0.0})
        plt.plot(1, i * 0.5, 'bo')
    for i in range(1, mid_exc_idx):
        exc_pts.append({'x': 1.0, 'y': i * 0.5, 'z': 0.0})
        plt.plot(1, i * 0.5, 'bo')

assert len(exc_pts) >= num_berm_points, 'Not enough excavation points! {} < {}'.format(len(exc_pts), num_berm_points)
print('Number of excavation points: {}'.format(len(exc_pts)))

plt.axis('equal')
# plt.show()
# exit()

env_name = 'env_{}'.format(env_width)
yaml_dict = {}
yaml_dict['berm_input'] = berm_inputs
yaml_dict['excavation_input'] = exc_pts
yaml_dict['berm_height'] = 0.09
yaml_dict['section_length'] = section_length
yaml_dict['map_image'] = env_name + '.png'
yaml_dict['output_file'] = 'outputs/output_{}_{}.csv'.format(env_name, num_berm_points)

with open('/home/hariharan/lx_ws/LunAR-X/src/lx_autonomy/src/lx_packages/lx_planning/maps/env_{}_{}.yaml'.format(env_width, num_berm_points), 'w') as outfile:
    yaml.dump(yaml_dict, outfile, default_flow_style=False)
