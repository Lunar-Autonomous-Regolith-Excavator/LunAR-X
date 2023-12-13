import numpy as np
import matplotlib.pyplot as plt
import yaml

def generate_x_points(cx, cy, angle, section_length, n_per_arm):
    points = []
    angles = [angle, 180 - angle, 180 + angle, 360 - angle]
    for i in range(4):
        for j in range(0, n_per_arm+1):
            r = j * section_length
            x = float(cx + r * np.cos(np.deg2rad(angles[i])))
            y = float(cy + r * np.sin(np.deg2rad(angles[i])))
            points.append({'x': x, 'y': y})
    return points


env_width = 5
section_length = 0.4

# generate points
berm_inputs = generate_x_points(6, env_width/2, 60, section_length, 3)
num_berm_points = len(berm_inputs) - 4

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

env_name = 'X'
yaml_dict = {}
yaml_dict['berm_input'] = berm_inputs
yaml_dict['excavation_input'] = exc_pts
yaml_dict['berm_height'] = 0.09
yaml_dict['section_length'] = section_length
yaml_dict['map_image'] = 'env_{}.png'.format(env_width)
yaml_dict['output_file'] = 'outputs/output_{}.csv'.format(env_name)

with open('/home/hariharan/lx_ws/LunAR-X/src/lx_autonomy/src/lx_packages/lx_planning/maps/env_{}.yaml'.format(env_name), 'w') as outfile:
    yaml.dump(yaml_dict, outfile, default_flow_style=False)
