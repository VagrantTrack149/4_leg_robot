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

# Límites reales de los motores (rad)
Q1_MIN, Q1_MAX = -0.6, 0.5      # Shoulder Abduction
Q2_MIN, Q2_MAX = -1.7, 1.7      # Shoulder Rotation
Q3_MIN, Q3_MAX = -0.45, 1.6     # Elbow


trayectoria= []


# Trayectoria espiral conica hacia abajo
def generar_espiral_conica(centro=(0,0,-0.20), R0=0.06, Rf=0.03, vueltas=3, n_puntos=30, z_inicial=-0.30, z_final=-0.45):
    puntos=[]
    for i in range(n_puntos):
        t = i / (n_puntos-1)
        radio = R0 - (R0-Rf)*t
        angulo = 2*np.pi * vueltas * t
        x = centro[0] + radio*np.cos(angulo)
        y = centro[1] + radio*np.sin(angulo)
        z = z_inicial + (z_final-z_inicial)*t
        puntos.append(np.array([x,y,z]))
    return puntos

trayectoria = generar_espiral_conica()
"""

radio= 0.05

centro_x= 0.0
centro_y= 0.0
centro_z= -0.27

num_puntos= 30

for t in np.linspace(0, 2*np.pi, num_puntos, endpoint=True):

    x= centro_x + radio*np.cos(2*t)
    y= centro_y + radio*np.sin(3*t)
    z= centro_z + radio*np.sin(5*t)

    trayectoria.append(np.array([x, y, z]))


trayectoria.append(np.array([0.0,0,-0.3]))
trayectoria.append(np.array([-0.35,0,-0.25]))
"""

def distancia_punto_segmento(P, A, B):
    AB = B - A
    AP = P - A

    largo2 = np.dot(AB, AB)

    if largo2 < 1e-12:
        return np.linalg.norm(P - A)

    t = np.clip(np.dot(AP, AB) / largo2, 0.0, 1.0)
    proyeccion = A + t * AB

    return np.linalg.norm(P - proyeccion)


def distancia_a_polilinea(P, trayectoria):
    mejor = np.inf

    for i in range(len(trayectoria) - 1):
        d = distancia_punto_segmento(
            P,
            trayectoria[i],
            trayectoria[i + 1]
        )

        if d < mejor:
            mejor = d

    return mejor


# Estado del control

estado = {
    'actual': np.array([0.0, -a1, -0.30]),
    'objetivo': np.array([0.0, -a1, -0.30]),

    'velocidad_interp': 0.2,

    'trayectoria': [],
    'indice_actual': 0,
    'siguiendo_trayectoria': False,
    'ruta_terminada': False,

    'tolerancia': 0.005,
}

#  Registro para análisis de trayectorias 
registro = {
    'historial_actual': [],       # posición real del pie en cada frame (relativo a cadera)
    'historial_objetivo': [],     # punto objetivo
    'puntos_alcanzados': [],      # posición real del pie en el instante en que se alcanzo
    'indices_alcanzados': [],     # índice del punto correspondiente, en el mismo orden
    'fuera_de_alcance': [],       # True/False por punto recortado
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
    ratio_sin_clip = py_l / a1
    ratio = np.clip(ratio_sin_clip, -1.0, 1.0)
    q1 = np.arccos(ratio)

    O1_z = -a1 * np.sin(q1)

    dx = px
    dz = pz - O1_z

    R_sin_clip = np.hypot(dx, dz)
    R = min(R_sin_clip, L2 + L3 - 1e-6)

    C3_sin_clip = (R ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    C3 = np.clip(C3_sin_clip, -1.0, 1.0)

    q3 = -np.arctan2(np.sqrt(1.0 - C3 ** 2), C3)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)

    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    fuera_de_alcance = (
        abs(ratio_sin_clip - ratio) > 1e-9
        or abs(R_sin_clip - R) > 1e-9
        or abs(C3_sin_clip - C3) > 1e-9
    )

    return q1, q2, q3, fuera_de_alcance


def angulos_validos(q1, q2, q3):
    return (
        Q1_MIN <= q1 <= Q1_MAX and
        Q2_MIN <= q2 <= Q2_MAX and
        Q3_MIN <= q3 <= Q3_MAX
    )


def punto_alcanzable(P, lado):
    """Verifica si un punto es alcanzable geométricamente Y dentro de los
    límites reales de los motores (minPosition/maxPosition)."""
    px, py, pz = P
    q1, q2, q3, fuera_geo = cinematica_inversa(px, py, pz, lado)
    fuera_rango = not angulos_validos(q1, q2, q3)
    return (not fuera_geo) and (not fuera_rango), (q1, q2, q3)


def puntos_pata():

    px, py, pz = estado['actual']

    q1, q2, q3, _ = cinematica_inversa(px, py, pz, LADO_FR)

    pts_locales = cinematica_directa(q1, q2, q3, LADO_FR)

    return [CADERA_FR + p for p in pts_locales]

def reorganizar_trayectoria(puntos, pos_inicial):
    if len(puntos) == 0:
        return puntos
    puntos_arr = np.array([np.array(p) for p in puntos])
    
    inicio = puntos_arr[0]
    fin = puntos_arr[-1]
    dist_inicio = np.linalg.norm(inicio - pos_inicial)
    dist_fin = np.linalg.norm(fin - pos_inicial)
    
    if dist_fin < dist_inicio:
        puntos_arr = puntos_arr[::-1]
        
    return [p for p in puntos_arr]


# Figura

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

ax.set_title("Una pierna seguimiento de puntos desde techo control de trayectorias")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-0.5, -0.1)
ax.set_ylim(-0.150, 0.025)
ax.set_zlim(-0.30, -0.05)

ax.set_box_aspect([1, 1, 1])

ax.view_init(elev=25, azim=145)



# Dibujar trayectoria planeada

tray_world = np.array([CADERA_FR + p for p in trayectoria])

ax.plot(
    tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
    '--', color='dodgerblue', linewidth=1.2, label='Trayectoria planeada'
)

ax.scatter(
    tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
    color='blue', s=30, label='Puntos planeados'
)

# Línea de trayectoria seguida
linea_seguida, = ax.plot(
    [], [], [],
    '-', color='crimson', linewidth=1.3, alpha=0.85,
    label='Trayectoria'
)



# Dibujar pierna

O0, O1, O2, O3 = puntos_pata()

l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='red', lw=4)
l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='red', lw=4)
l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='red', lw=4)

articulaciones = ax.scatter(
    [O0[0], O1[0], O2[0], O3[0]],
    [O0[1], O1[1], O2[1], O3[1]],
    [O0[2], O1[2], O2[2], O3[2]],
    color='red', s=40
)

pie, = ax.plot(
    [O3[0]], [O3[1]], [O3[2]],
    'o', color='gold', markersize=10, markeredgecolor='black', label='Pie'
)



# Marcador del objetivo vigente

objetivo_plot, = ax.plot(
    [CADERA_FR[0]], [CADERA_FR[1]], [CADERA_FR[2]],
    'x', color='green', markersize=12, markeredgewidth=3, label='Objetivo'
)
funcion_control= ax.plot(
    [], [], [],
    '-', color='orange', linewidth=1.5, label='Funcion \n actual += k*(objetivo-actual)'
)
ax.legend(loc='upper left', fontsize=8)



# Texto de progresos

texto_estado = fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')


def actualizar_marcador_objetivo():
    x, y, z = estado['objetivo']
    objetivo_plot.set_data_3d(
        [CADERA_FR[0] + x], [CADERA_FR[1] + y], [CADERA_FR[2] + z]
    )


def iniciar_trayectoria():

    if len(trayectoria) == 0:
        return
    trayectoria_ordenada = reorganizar_trayectoria(trayectoria, estado['actual'])

    # Descartar puntos que violen los límites reales de los motores
    puntos_validos = [p for p in trayectoria_ordenada if punto_alcanzable(p, LADO_FR)[0]]
    descartados = len(trayectoria_ordenada) - len(puntos_validos)
    if descartados > 0:
        print(f"Aviso: se descartaron {descartados} punto(s) de la trayectoria "
              f"por estar fuera de los límites articulares reales de los motores.")

    if len(puntos_validos) == 0:
        print("Ningún punto de la trayectoria es alcanzable dentro de los "
              "límites articulares. Ruta no iniciada.")
        return

    # Reiniciar registro para una corrida limpia
    registro['historial_actual'].clear()
    registro['historial_objetivo'].clear()
    registro['puntos_alcanzados'].clear()
    registro['indices_alcanzados'].clear()
    registro['fuera_de_alcance'].clear()

    estado['trayectoria'] = puntos_validos
    estado['indice_actual'] = 0
    estado['siguiendo_trayectoria'] = True
    estado['ruta_terminada'] = False

    estado['objetivo'] = estado['trayectoria'][0]

    actualizar_marcador_objetivo()



# Botón

ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])
boton = Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())



# Reporte final de error
def mostrar_reporte_error():
    # 1. Preparación de datos
    hist_world = np.array([CADERA_FR + p for p in registro['historial_actual']])
    tray_world = np.array([CADERA_FR + p for p in trayectoria])
    
    # Cálculo de errores perpendiculares
    errores = []
    for P in hist_world:
        errores.append(
            distancia_a_polilinea(P, tray_world)
        )
        
    errores = np.array(errores)
    rmse = np.sqrt(np.mean(errores**2))
    
    # 2. Cálculos de cinemática
    dt = 0.02
    hist = np.array(registro['historial_actual'])
    
    # Velocidad
    vel = np.diff(hist, axis=0) / dt
    velocidad = np.linalg.norm(vel, axis=1)
    
    # Aceleración
    ace = np.diff(vel, axis=0) / dt
    aceleracion = np.linalg.norm(ace, axis=1)
    
    print(f"\nRMSE (distancia a la trayectoria): {rmse:.5f} m\n")
    
    # 3. Generación de Gráficas
    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    
    # Gráfica 1: Error vs Tolerancia
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(estado['tolerancia'], color='gray', linestyle='--', 
                   label=f"tolerancia ({estado['tolerancia']} m)")
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':', 
                   label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame")
    axs[0].set_ylabel("Error perpendicular")
    axs[0].set_title("Error de seguimiento")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)
    
    # Gráfica 2: Velocidad y Aceleración
    tiempo_v = np.arange(len(velocidad)) * dt
    tiempo_a = np.arange(len(aceleracion)) * dt
    
    axs[1].plot(tiempo_v, velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Tiempo (s)")
    axs[1].set_ylabel("Magnitud")
    axs[1].set_title("Velocidad y aceleración de la trayectoria")
    axs[1].legend()
    axs[1].grid(alpha=0.3)
    
    fig2.tight_layout()
    fig2.show()

# Animación

def actualizar(frame):

    error = estado['objetivo'] - estado['actual']

    estado['actual'] += estado['velocidad_interp'] * error

    # Registrar historial trayectoria
    registro['historial_actual'].append(estado['actual'].copy())
    registro['historial_objetivo'].append(estado['objetivo'].copy())

    # Cambio al siguiente punto
    if estado['siguiendo_trayectoria']:

        distancia = np.linalg.norm(estado['objetivo'] - estado['actual'])

        if distancia < estado['tolerancia']:

            # Verificar si este punto estaba fuera de alcance (geometría o límites de motor)
            fuera, _ = punto_alcanzable(estado['objetivo'], LADO_FR)
            fuera = not fuera

            # Registrar punto alcanzado
            registro['puntos_alcanzados'].append(estado['actual'].copy())
            registro['indices_alcanzados'].append(estado['indice_actual'])
            registro['fuera_de_alcance'].append(fuera)

            estado['indice_actual'] += 1

            if estado['indice_actual'] >= len(estado['trayectoria']):

                estado['siguiendo_trayectoria'] = False

                if not estado['ruta_terminada']:
                    estado['ruta_terminada'] = True
                    mostrar_reporte_error()

            else:

                estado['objetivo'] = estado['trayectoria'][estado['indice_actual']]
                actualizar_marcador_objetivo()

    O0, O1, O2, O3 = puntos_pata()

    # Alerta visual si la posición actual excede los límites reales de los motores
    dentro_de_limites, _ = punto_alcanzable(estado['actual'], LADO_FR)
    color_pierna = 'red' if dentro_de_limites else 'orange'
    l01.set_color(color_pierna)
    l12.set_color(color_pierna)
    l23.set_color(color_pierna)

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])

    articulaciones._offsets3d = (
        [O0[0], O1[0], O2[0], O3[0]],
        [O0[1], O1[1], O2[1], O3[1]],
        [O0[2], O1[2], O2[2], O3[2]]
    )

    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Actualizar la línea de trayectoria seguida 
    if len(registro['historial_actual']) > 1:
        hist = np.array(registro['historial_actual'])
        hist_world = CADERA_FR + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    # Texto de progreso en vivo
    if estado['siguiendo_trayectoria'] or estado['ruta_terminada']:
        n_alcanzados = len(registro['indices_alcanzados'])
        texto_estado.set_text(
            f"Punto: {estado['indice_actual']}/{len(estado['trayectoria'])}\n"
            f"Alcanzados: {n_alcanzados}\n"
            f"Estado: {'en ruta' if estado['siguiendo_trayectoria'] else 'completado'}"
        )

    return []



# Ejecutar

ani = FuncAnimation(fig, actualizar, interval=20, blit=False)

plt.tight_layout()
plt.show()