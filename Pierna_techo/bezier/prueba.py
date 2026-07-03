import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Parametros del robot (identicos al controlador)
a1 = 0.0528
L2 = 0.2142
L3 = 0.2142
LADO_FL = 1

Q1_MIN, Q1_MAX = 0.0, np.pi
Q2_MIN, Q2_MAX = -0.5, 0.5
Q3_MIN, Q3_MAX = -0.45, 0.0


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
    q2 = np.clip(q2, -0.5, 0.5)
    q3 = np.clip(q3, -0.45, 0.0)
    return q1, q2, q3


def cinematica_directa(q1, q2, q3, lado):
    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
    O0 = np.array([0.0, 0.0, 0.0])
    O1 = np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2 = O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3 = O2 + np.array([L3 * s23, 0.0, -L3 * c23])
    return O0, O1, O2, O3


def punto_alcanzable(px, py, pz, lado):
    py_l = py * lado
    ratio = py_l / a1
    if abs(ratio) > 1.0:
        return False
    q1 = np.arccos(np.clip(ratio, -1.0, 1.0))
    O1_z = -a1 * np.sin(q1)
    dx = px
    dz = pz - O1_z
    R = np.hypot(dx, dz)
    if R > (L2 + L3 - 1e-6):
        return False
    C3 = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    C3 = np.clip(C3, -1.0, 1.0)
    q3 = -np.arctan2(np.sqrt(1.0 - C3**2), C3)
    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)
    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)
    return (Q1_MIN <= q1 <= Q1_MAX and
            Q2_MIN <= q2 <= Q2_MAX and
            Q3_MIN <= q3 <= Q3_MAX)


def generar_espiral_conica(centro, R0=0.03, Rf=0.0, vueltas=2,
                            n_puntos=30, z_inicial=-0.30, z_final=-0.45):
    puntos = []
    for i in range(n_puntos):
        t = i / (n_puntos - 1)
        radio = R0 - (R0 - Rf) * t
        angulo = 2 * np.pi * vueltas * t
        x = centro[0] + radio * np.cos(angulo)
        y = centro[1] + radio * np.sin(angulo)
        z = z_inicial + (z_final - z_inicial) * t
        puntos.append(np.array([x, y, z]))
    return puntos


CENTRO_ESPIRAL = (0.1, -0.02, -0.48)

trayectoria = generar_espiral_conica(
    CENTRO_ESPIRAL,
    R0=0.02, Rf=0.0,
    vueltas=2, n_puntos=30,
    z_inicial=-0.4, z_final=-0.47
)

validos = [p for p in trayectoria if punto_alcanzable(p[0], p[1], p[2], LADO_FL)]
invalidos = [p for p in trayectoria if not punto_alcanzable(p[0], p[1], p[2], LADO_FL)]
pct = len(validos) / len(trayectoria) * 100

# Pierna en la postura inicial (primer punto de la espiral)
p_ini = trayectoria[0]
q1, q2, q3 = cinematica_inversa(p_ini[0], p_ini[1], p_ini[2], LADO_FL)
O0, O1, O2, O3 = cinematica_directa(q1, q2, q3, LADO_FL)

# Distancias relevantes
dist_cadera_pie = np.linalg.norm(O3 - O0)
dist_pie_centro_espiral = np.linalg.norm(np.array(p_ini) - np.array(CENTRO_ESPIRAL))
alcance_max = L2 + L3

fig = plt.figure(figsize=(11, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Espiral conica y pierna FL (estatico) - distancias")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

tray_arr = np.array(trayectoria)
val_arr = np.array(validos)
inv_arr = np.array(invalidos) if len(invalidos) > 0 else None

ax.plot(tray_arr[:, 0], tray_arr[:, 1], tray_arr[:, 2],
        '--', color='dodgerblue', linewidth=1.2, label='Espiral (ruta completa)')
ax.scatter(val_arr[:, 0], val_arr[:, 1], val_arr[:, 2],
           color='green', s=35, label=f'Alcanzables ({len(validos)}/{len(trayectoria)}, {pct:.0f}%)')
if inv_arr is not None:
    ax.scatter(inv_arr[:, 0], inv_arr[:, 1], inv_arr[:, 2],
               color='red', s=35, marker='x', label=f'No alcanzables ({len(invalidos)})')

# Pierna: cadera -> hombro -> codo -> pie
ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='black', lw=4, label='Pierna (eslabones)')
ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='black', lw=4)
ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='black', lw=4)
ax.scatter([O0[0], O1[0], O2[0], O3[0]], [O0[1], O1[1], O2[1], O3[1]], [O0[2], O1[2], O2[2], O3[2]],
           color='black', s=60)
ax.text(O0[0], O0[1], O0[2], '  Cadera (O0)', fontsize=9)
ax.text(O1[0], O1[1], O1[2], '  Hombro (O1)', fontsize=9)
ax.text(O2[0], O2[1], O2[2], '  Codo (O2)', fontsize=9)
ax.text(O3[0], O3[1], O3[2], '  Pie (O3)', fontsize=9)

# Centro de la espiral marcado
ax.scatter([CENTRO_ESPIRAL[0]], [CENTRO_ESPIRAL[1]], [CENTRO_ESPIRAL[2]],
           color='purple', s=80, marker='*', label='Centro espiral')

# Linea de distancia cadera-pie
ax.plot([O0[0], O3[0]], [O0[1], O3[1]], [O0[2], O3[2]],
        ':', color='gray', linewidth=1)

texto = (
    f"Distancia cadera->pie: {dist_cadera_pie:.4f} m\n"
    f"Alcance maximo (L2+L3): {alcance_max:.4f} m\n"
    f"Distancia pie inicial -> centro espiral: {dist_pie_centro_espiral:.4f} m\n"
    f"Radio espiral: {0.025:.3f} m -> {0.01:.3f} m\n"
    f"Puntos alcanzables: {len(validos)}/{len(trayectoria)} ({pct:.0f}%)"
)
fig.text(0.02, 0.02, texto, fontsize=9, family='monospace', va='bottom')

ax.legend(loc='upper left', fontsize=8)
ax.set_box_aspect([1, 1, 1])
ax.view_init(elev=25, azim=45)

plt.tight_layout()
#plt.savefig('/mnt/user-data/outputs/espiral_pierna_3d.png', dpi=150)
print("Distancia cadera->pie:", dist_cadera_pie)
print("Alcance maximo:", alcance_max)
print("Puntos alcanzables:", len(validos), "/", len(trayectoria))