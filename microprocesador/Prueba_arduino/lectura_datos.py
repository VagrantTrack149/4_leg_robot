import io
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

raw_data = """x,y,z
978.14,279.34,3.32
845.31,546.34,1.86
653.35,759.32,-1.18
343.42,951.00,0.45
82.67,999.51,4.01
-171.19,993.83,-1.54
-481.55,883.60,7.14
-717.80,692.86,-3.73
-915.03,424.23,6.05
-997.44,105.54,-3.86
-992.18,-174.52,-7.13
-906.83,-438.08,-1.47
-717.74,-703.25,0.45
-506.42,-877.72,-1.90
-230.37,-981.76,3.21
65.78,-1000.96,-4.54
343.58,-947.34,5.02
659.46,-771.05,-1.17
837.46,-562.13,2.97
959.70,-329.02,-3.73"""

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