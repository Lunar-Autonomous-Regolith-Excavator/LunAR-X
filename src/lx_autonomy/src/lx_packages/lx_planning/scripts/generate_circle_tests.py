import numpy as np
import matplotlib.pyplot as plt

def generate_circle_points(a, b, r, theta, offset_theta, max_theta=360):
    points = []
    for i in range(0, int(max_theta/theta)):
        x = a + r * np.cos(np.deg2rad(i * theta + offset_theta))
        y = b + r * np.sin(np.deg2rad(i * theta + offset_theta))
        points.append([x, y])
    return points


pts = generate_circle_points(6, 1.5, 1.25, 20, 180)

# plot points
plt.plot([x[0] for x in pts], [x[1] for x in pts], 'ro-')
plt.axis('equal')
plt.show()

# for i in range(1, len(pts)):
#     print(np.sqrt((pts[i][0] - pts[i-1][0])**2 + (pts[i][1] - pts[i-1][1])**2))

for pt in pts:
    print('-', 'x:', pt[0], '\n  y:', pt[1])