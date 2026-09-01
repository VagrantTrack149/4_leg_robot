import io
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

raw_data = """x,y,z
883.36,533.16,-2.44
446.02,924.73,1.29
-147.29,1025.16,-2.18
-693.54,757.86,-2.29
-974.91,303.71,2.98
-998.46,-252.93,-3.58
-661.66,-794.67,2.48
-99.44,-1025.89,-1.48
428.05,-922.17,0.26
870.98,-558.61,0.09"""

data = np.loadtxt(io.StringIO(raw_data), delimiter=',', skiprows=1)

unvisited = set(range(len(data)))
current = 0
path = [current]
unvisited.remove(current)

while unvisited:
    distances = cdist([data[current]], data[list(unvisited)])[0]
    next_idx = np.argmin(distances)
    next_node = list(unvisited)[next_idx]
    path.append(next_node)
    unvisited.remove(next_node)
    current = next_node

path_coords = data[path]
path_closed = np.vstack([path_coords, path_coords[0]])

# Generar la gráfica 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(data[:, 0], data[:, 1], data[:, 2], c='red', marker='o', s=50, label='Puntos')
ax.plot(path_closed[:, 0], path_closed[:, 1], path_closed[:, 2], c='blue', linestyle='-', linewidth=2, label='Trayectoria')

# Numerar los puntos en orden de recorrido
for i, idx in enumerate(path):
    ax.text(data[idx, 0], data[idx, 1], data[idx, 2], f' {idx}', fontsize=9, color='black')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trayectoria 3D')
ax.legend()
plt.show()