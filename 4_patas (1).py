import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation

# Parámetros
a1 = 0.0528
L2 = 0.2142
L3 = 0.2142
ALTURA_NEUTRA = -0.30

# Posición caderas/hombros
CADERAS = {
    'RR': np.array([ 0.29785, -0.055, 0.0]),
    'RL': np.array([ 0.29785,  0.055, 0.0]),
    'FR': np.array([-0.29785, -0.055, 0.0]),
    'FL': np.array([-0.29785,  0.055, 0.0]),
}

COLORES = {
    'FR': 'red',
    'FL': 'blue',
    'RR': 'orange',
    'RL': 'green'
}

LADO = {
    'RR': -1,
    'RL': +1,
    'FR': -1,
    'FL': +1
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

# Objetivo del pie
FASES_TROTE = {
    'FR': 0.0,
    'FL': np.pi,
    'RR': np.pi,
    'RL': 0.0
}

estado = {
    'caminando': False,
    't': 0.0,
    'freq': 1,
    'amp_paso': 0.09,
    'amp_elev': 0.045,
    'altura': ALTURA_NEUTRA,
}

def objetivo_pie(nombre, t):
    l = LADO[nombre]
    base = np.array([0.0, l * a1, estado['altura']])

    if not estado['caminando']:
        return base

    w = 2.0 * np.pi * estado['freq']
    phi = w * t + FASES_TROTE[nombre]

    dx = -estado['amp_paso'] * np.sin(phi)
    vuelo = np.sin(phi + np.pi / 2.0)
    dz = estado['amp_elev'] * max(0.0, vuelo)

    return base + np.array([dx, 0.0, dz])

def puntos_pata(nombre, t):
    cadera = CADERAS[nombre]
    lado = LADO[nombre]
    tgt = objetivo_pie(nombre, t)
    q1, q2, q3 = cinematica_inversa(tgt[0], tgt[1], tgt[2], lado)
    pts = cinematica_directa(q1, q2, q3, lado)
    return [cadera + p for p in pts]

# Fijura y ejes 
fig = plt.figure(figsize=(11, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Spot sim", fontsize=13)
ax.set_xlabel("X ")
ax.set_ylabel("Y ")
ax.set_zlabel("Z ")

# Límites 
ax.set_xlim(-0.7, 0.7)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(-0.6, 0.3)
ax.set_box_aspect([1.4, 1.0, 0.9])


# Cuerpo
orden = ['FL', 'FR', 'RR', 'RL']
cx_body = [CADERAS[k][0] for k in orden] + [CADERAS['FL'][0]]
cy_body = [CADERAS[k][1] for k in orden] + [CADERAS['FL'][1]]
ax.plot(cx_body, cy_body, [0.0]*5,
        'k-', linewidth=2, alpha=0.6, label='Cuerpo')

# Patas
segmentos = {}
puntos_art = {}
marcas_pie = {}

for nombre in CADERAS:
    O0, O1, O2, O3 = puntos_pata(nombre, 0.0)
    c = COLORES[nombre]

    # Segmentos, el primero con marcadores pequeños
    l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]],
                   color=c, lw=2.5, marker='o', ms=4, label=nombre)
    l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]],
                   color=c, lw=2.5)
    l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]],
                   color=c, lw=2.5)

    # Puntos articulares (con pie)
    sc = ax.scatter([O0[0], O1[0], O2[0], O3[0]],
                    [O0[1], O1[1], O2[1], O3[1]],
                    [O0[2], O1[2], O2[2], O3[2]],
                    color=c, s=25, edgecolors='k')

    # Marcador del pie amarillo
    pie, = ax.plot([O3[0]], [O3[1]], [O3[2]],
                   'o', color='gold', ms=7, markeredgecolor='black')

    segmentos[nombre] = (l01, l12, l23)
    puntos_art[nombre] = sc
    marcas_pie[nombre] = pie

# Leyenda
ax.legend(loc='upper left', fontsize=9)

# Animación
def actualizar(frame):
    if estado['caminando']:
        estado['t'] += 0.04 / max(estado['freq'], 0.1)

    for nombre in CADERAS:
        O0, O1, O2, O3 = puntos_pata(nombre, estado['t'])
        l01, l12, l23 = segmentos[nombre]

        l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
        l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
        l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])

        puntos_art[nombre]._offsets3d = (
            [O0[0], O1[0], O2[0], O3[0]],
            [O0[1], O1[1], O2[1], O3[1]],
            [O0[2], O1[2], O2[2], O3[2]]
        )

        marcas_pie[nombre].set_data_3d([O3[0]], [O3[1]], [O3[2]])

    return []

# Botón único 
ax_btn = plt.axes([0.80, 0.05, 0.15, 0.06])
boton = Button(ax_btn, 'Iniciar', color='lightgray', hovercolor='lightblue')

def alternar(event):
    estado['caminando'] = not estado['caminando']
    boton.label.set_text('Detener' if estado['caminando'] else 'Iniciar')
    fig.canvas.draw_idle()

boton.on_clicked(alternar)

ani = FuncAnimation(fig, actualizar, interval=50, blit=False)
plt.tight_layout()
plt.show()