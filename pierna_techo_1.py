import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


# Parámetros de la pierna
a1 = 0.0528
L2 = 0.2142
L3 = 0.2142


# Cadera
CADERA_FR = np.array([-0.29785, -0.055, 0.0])
LADO_FR = -1


# Trayectoria
# Coordenadas relativas a la cadera
trayectoria = []

radio = 0.05

centro_x = 0.0
centro_y = 0.0
centro_z = -0.27

num_puntos = 45

for t in np.linspace(0, 2*np.pi, num_puntos, endpoint=False):

    x = centro_x + radio*np.cos(1*t)
    y = centro_y + radio*np.sin(1*t)
    z = centro_z + radio*np.sin(1*t)

    trayectoria.append(np.array([x, y, z]))


# Estado


estado = {
    'actual': np.array([0.0, -a1, -0.30]),
    'objetivo': np.array([0.0, -a1, -0.30]),

    'velocidad_interp': 0.2,

    'trayectoria': [],
    'indice_actual': 0,
    'siguiendo_trayectoria': False,

    'tolerancia': 0.005
}


# Cinemática directa


def cinematica_directa(q1, q2, q3, lado):

    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)

    O0 = np.array([0.0, 0.0, 0.0])
    O1 = np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2 = O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3 = O2 + np.array([L3 * s23, 0.0, -L3 * c23])

    return O0, O1, O2, O3



# Cinemática inversa


def cinematica_inversa(px, py, pz, lado):
    py_l = py * lado
    ratio = np.clip(py_l / a1, -1.0, 1.0)
    q1 = np.arccos(ratio)

    O1_z = -a1 * np.sin(q1)

    dx = px
    dz = pz - O1_z

    R = np.hypot(dx, dz)
    R = min(R, L2 + L3 - 1e-6)

    C3 = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    C3 = np.clip(C3, -1.0, 1.0)

    q3 = -np.arctan2(np.sqrt(1.0 - C3**2), C3)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)

    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)
    
    return q1, q2, q3


# Obtener puntos de la pierna


def puntos_pata():

    px, py, pz = estado['actual']

    q1, q2, q3 = cinematica_inversa(
        px,
        py,
        pz,
        LADO_FR
    )

    pts_locales = cinematica_directa(
        q1,
        q2,
        q3,
        LADO_FR
    )

    return [CADERA_FR + p for p in pts_locales]


# Figura


fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

ax.set_title("Una pierna seguimiento de puntos desde techo")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-0.5, -0.1)
ax.set_ylim(-0.150, 0.025)
ax.set_zlim(-0.30, -0.05)

ax.set_box_aspect([1, 1, 1])

ax.view_init(
    elev=25,
    azim=145
)


# Dibujar trayectoria


tray_world = np.array([
    CADERA_FR + p
    for p in trayectoria
])

ax.plot(
    tray_world[:,0],
    tray_world[:,1],
    tray_world[:,2],
    '--',
    color='dodgerblue',
    linewidth=1.5
)

ax.scatter(
    tray_world[:,0],
    tray_world[:,1],
    tray_world[:,2],
    color='blue',
    s=30
)


# Dibujar pierna


O0, O1, O2, O3 = puntos_pata()

l01, = ax.plot(
    [O0[0], O1[0]],
    [O0[1], O1[1]],
    [O0[2], O1[2]],
    color='red',
    lw=4
)

l12, = ax.plot(
    [O1[0], O2[0]],
    [O1[1], O2[1]],
    [O1[2], O2[2]],
    color='red',
    lw=4
)

l23, = ax.plot(
    [O2[0], O3[0]],
    [O2[1], O3[1]],
    [O2[2], O3[2]],
    color='red',
    lw=4
)

articulaciones = ax.scatter(
    [O0[0], O1[0], O2[0], O3[0]],
    [O0[1], O1[1], O2[1], O3[1]],
    [O0[2], O1[2], O2[2], O3[2]],
    color='red',
    s=40
)

pie, = ax.plot(
    [O3[0]],
    [O3[1]],
    [O3[2]],
    'o',
    color='gold',
    markersize=10,
    markeredgecolor='black'
)


# Objetivo actual


objetivo_plot, = ax.plot(
    [CADERA_FR[0]],
    [CADERA_FR[1]],
    [CADERA_FR[2]],
    'x',
    color='lime',
    markersize=12,
    markeredgewidth=3
)


# Funciones trayectoria


def actualizar_marcador_objetivo():

    x, y, z = estado['objetivo']

    objetivo_plot.set_data_3d(
        [CADERA_FR[0] + x],
        [CADERA_FR[1] + y],
        [CADERA_FR[2] + z]
    )

def iniciar_trayectoria():

    if len(trayectoria) == 0:
        return

    estado['trayectoria'] = trayectoria
    estado['indice_actual'] = 0
    estado['siguiendo_trayectoria'] = True

    estado['objetivo'] = trayectoria[0]

    actualizar_marcador_objetivo()


# Botón


ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])

boton = Button(
    ax_btn,
    'Iniciar Ruta',
    color='lightgray',
    hovercolor='lightblue'
)

boton.on_clicked(lambda event: iniciar_trayectoria())


# Animación


def actualizar(frame):

    error = estado['objetivo'] - estado['actual']

    estado['actual'] += (
        estado['velocidad_interp'] * error
    )

    # Cambio automático al siguiente waypoint
    if estado['siguiendo_trayectoria']:

        distancia = np.linalg.norm(
            estado['objetivo'] - estado['actual']
        )

        if distancia < estado['tolerancia']:

            estado['indice_actual'] += 1

            if estado['indice_actual'] >= len(
                estado['trayectoria']
            ):

                estado['siguiendo_trayectoria'] = False

            else:

                estado['objetivo'] = (
                    estado['trayectoria'][
                        estado['indice_actual']
                    ]
                )

                actualizar_marcador_objetivo()

    O0, O1, O2, O3 = puntos_pata()

    l01.set_data_3d(
        [O0[0], O1[0]],
        [O0[1], O1[1]],
        [O0[2], O1[2]]
    )

    l12.set_data_3d(
        [O1[0], O2[0]],
        [O1[1], O2[1]],
        [O1[2], O2[2]]
    )

    l23.set_data_3d(
        [O2[0], O3[0]],
        [O2[1], O3[1]],
        [O2[2], O3[2]]
    )

    articulaciones._offsets3d = (
        [O0[0], O1[0], O2[0], O3[0]],
        [O0[1], O1[1], O2[1], O3[1]],
        [O0[2], O1[2], O2[2], O3[2]]
    )

    pie.set_data_3d(
        [O3[0]],
        [O3[1]],
        [O3[2]]
    )

    return []


# Ejecutar


ani = FuncAnimation(
    fig,
    actualizar,
    interval=20,
    blit=False
)

plt.tight_layout()
plt.show()