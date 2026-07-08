import math
import numpy as np
from controller import Robot
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pata_common import (
    cinematica_inversa, cinematica_directa, interno_a_motor,
    punto_alcanzable,
    generar_espiral,
    generar_lissajous,      
    generar_escalon,        
    bezier_cubico,
    preparar_trayectoria_completa, generar_bezier_completa,
    calcular_errores, calcular_velocidad_aceleracion,
    Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX, Q3_MIN, Q3_MAX,
    L_hip, L_thigh, L_calf,
)
L_hip=-L_hip
MOSTRAR_EN_VIVO = True   # Cambiar a False para ejecutar sin gráficas en tiempo real

# Tipo de trayectoria base: 'espiral', 'lissajous', 'escalon', 'manual', o 'simple'
TIPO_TRAYECTORIA = 'manual'

# Modo de seguimiento: 'bezier', 'lineal', 'poligonal'
MODO_SEGUIMIENTO = 'lineal'

# Parámetros para el modo poligonal (número de subsegmentos por tramo)
SUBSEGMENTOS_POLIGONAL = 10

# PARÁMETROS PROPIOS DE LA PATA FL
# L_hip = 0.08 m, L_thigh = 0.34 m, L_calf = 0.35 m
LADO = 1  # pata delantera izquierda (positivo para izquierda, negativo der)
PLOT_CADA_N_PASOS = 1


if TIPO_TRAYECTORIA == 'manual':
    trayectoria = [
        np.array([0.1354, 0.0800, -0.6731]),   # Inicio: bajo-atrás
        np.array([-0.0326, 0.0800, -0.6815]),  # Levanta pata
        np.array([-0.2664, 0.0800, -0.6327]),  # Mueve adelante-arriba
        np.array([0.0596, 0.0800, -0.6327]),   # Vuelve abajo-adelante
    ]
    print("Usando trayectoria manual")

elif TIPO_TRAYECTORIA == 'espiral':
    # Espiral para Spot
    trayectoria = generar_espiral(
        centro=(0.1, 0.08, -0.60),
        R0=0.08, Rf=0.02,
        vueltas=1.5, n_puntos=25,
        z_ini=-0.68, z_fin=-0.55
    )

elif TIPO_TRAYECTORIA == 'lissajous':
    trayectoria = generar_lissajous(
        centro=(0.1, 0.08, -0.60),
        radio=0.08,
        frecuencias=(2, 3, 5),
        n_puntos=100
    )
elif TIPO_TRAYECTORIA == 'escalon':
   trayectoria = generar_escalon()
   
elif TIPO_TRAYECTORIA == 'simple':
    trayectoria = [
        np.array([0.0, 0.0800, -0.65]),   # Arriba
        np.array([0.0, 0.0800, -0.69]),   # Abajo
    ]
   

else:
    raise ValueError("TIPO_TRAYECTORIA no válido.")

# Filtrar puntos alcanzables
trayectoria = [p for p in trayectoria if punto_alcanzable(p, LADO)]

# Si no hay suficientes puntos,  advertencia y usamos una trayectoria mínima
if len(trayectoria) < 2:
    print("No hay suficientes puntos alcanzables. Se usará postura de reposo.")
    # Punto único para mantener la pierna quieta
    trayectoria = [np.array([-0.3197, 0.0528, -0.4284])]   # reposo
    # Desactivamos el seguimiento
    segmentos = []
    referencia = np.array(trayectoria) if trayectoria else np.empty((0,3))
else:
    # PREPARACIÓN DE SEGMENTOS 
    if MODO_SEGUIMIENTO == 'bezier':
        segmentos = preparar_trayectoria_completa(trayectoria, LADO)
        referencia = generar_bezier_completa(segmentos)
        print(f"Segmentos Bézier creados: {len(segmentos)}")
    elif MODO_SEGUIMIENTO == 'lineal':
        segmentos = [(trayectoria[i], trayectoria[i+1]) for i in range(len(trayectoria)-1)]
        referencia = np.array(trayectoria)
        print(f"Segmentos lineales creados: {len(segmentos)}")
    elif MODO_SEGUIMIENTO == 'poligonal':
        segmentos = []
        for i in range(len(trayectoria)-1):
            p0 = trayectoria[i]
            p1 = trayectoria[i+1]
            for j in range(SUBSEGMENTOS_POLIGONAL):
                t0 = j / SUBSEGMENTOS_POLIGONAL
                t1 = (j+1) / SUBSEGMENTOS_POLIGONAL
                a = p0 + t0 * (p1 - p0)
                b = p0 + t1 * (p1 - p0)
                segmentos.append((a, b))
        referencia = np.array(trayectoria)
        print(f"Segmentos poligonales creados: {len(segmentos)}")
    else:
        raise ValueError("MODO_SEGUIMIENTO no válido.")


# CONFIGURACIÓN DE WEbots
robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_names = [
    "front left shoulder abduction motor",
    "front left shoulder rotation motor",
    "front left elbow motor"
]

motors = [robot.getDevice(name) for name in motor_names]
for m in motors:
    m.setVelocity(2.0)      # rad/s
    m.setAcceleration(5.0)  # rad/s²


# GRÁFICA 3D EN VIVO

if MOSTRAR_EN_VIVO:
    plt.ion()
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f"Webots Spot - pata FL ({MODO_SEGUIMIENTO}) [Modelo oficial]")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_xlim(-0.2, 0.8)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.8, 0.1)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=49, azim=1, roll=0)
    ax.scatter([0], [0], [0], color='red', s=80, marker='^', label='Cadera')

    # Waypoints 
    waypoints = np.array(trayectoria) if trayectoria else np.empty((0,3))
    if len(waypoints) > 0:
        ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                   color='blue', s=25, alpha=0.5, label='Waypoints')

    # Referencia 
    if len(referencia) > 0:
        if MODO_SEGUIMIENTO == 'bezier':
            ax.plot(referencia[:, 0], referencia[:, 1], referencia[:, 2],
                    color='lime', linewidth=1.5, linestyle='--', label='Bézier planeada')
        else:
            ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                    color='lime', linewidth=1.5, linestyle='--', label='Trayectoria planeada')

    linea_real, = ax.plot([], [], [], color='crimson', linewidth=1.5, label='Recorrido real')

    # Pierna calculada (naranja)
    l01_calc, = ax.plot([], [], [], color='orange', lw=4, label='Pierna calculada')
    l12_calc, = ax.plot([], [], [], color='orange', lw=4)
    l23_calc, = ax.plot([], [], [], color='orange', lw=4)
    art_calc = ax.scatter([0,0,0,0], [0,0,0,0], [0,0,0,0], color='orange', s=30)
    pie_calc, = ax.plot([], [], [], 'o', color='gold', markersize=9, markeredgecolor='black', label='Pie calc.')

    # Pierna real (datos de motor) – azul
    l01_real, = ax.plot([], [], [], color='cyan', lw=3, label='Pierna real (motor)')
    l12_real, = ax.plot([], [], [], color='cyan', lw=3)
    l23_real, = ax.plot([], [], [], color='cyan', lw=3)
    art_real = ax.scatter([0,0,0,0], [0,0,0,0], [0,0,0,0], color='cyan', s=30)
    pie_real, = ax.plot([], [], [], 's', color='lightseagreen', markersize=7, markeredgecolor='black', label='Pie real')

    ax.legend(loc='upper left', fontsize=8)
    texto_estado = fig.text(0.02, 0.95, "", fontsize=8, family='monospace', va='top')

    plt.tight_layout()
    plt.show(block=False)
    fig.canvas.draw()
    fig.canvas.flush_events()

else:
    fig = None
    ax = None
    linea_real = None
    l01_calc = l12_calc = l23_calc = None
    art_calc = None
    pie_calc = None
    l01_real = l12_real = l23_real = None
    art_real = None
    pie_real = None
    texto_estado = None


# VARIABLES GLOBALES PARA LA PIERNA REAL
ultima_q1_m = 0.0
ultima_q2_m = 0.0
ultima_q3_m = 0.0

historial_real = []

def actualizar_grafica(punto_pie, q_interno):
    if not MOSTRAR_EN_VIVO:
        return

    # Pierna calculada
    O0, O1, O2, O3 = cinematica_directa(*q_interno, LADO)
    l01_calc.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12_calc.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23_calc.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    art_calc._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                           [O0[1], O1[1], O2[1], O3[1]],
                           [O0[2], O1[2], O2[2], O3[2]])
    pie_calc.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Pierna real (datos de motor)
    q1m_cmd, q2m_cmd, q3m_cmd = ultima_q1_m, ultima_q2_m, ultima_q3_m
    O0r, O1r, O2r, O3r = cinematica_directa(q1m_cmd, q2m_cmd, q3m_cmd, LADO)

    l01_real.set_data_3d([O0r[0], O1r[0]], [O0r[1], O1r[1]], [O0r[2], O1r[2]])
    l12_real.set_data_3d([O1r[0], O2r[0]], [O1r[1], O2r[1]], [O1r[2], O2r[2]])
    l23_real.set_data_3d([O2r[0], O3r[0]], [O2r[1], O3r[1]], [O2r[2], O3r[2]])
    art_real._offsets3d = ([O0r[0], O1r[0], O2r[0], O3r[0]],
                           [O0r[1], O1r[1], O2r[1], O3r[1]],
                           [O0r[2], O1r[2], O2r[2], O3r[2]])
    pie_real.set_data_3d([O3r[0]], [O3r[1]], [O3r[2]])

    if len(historial_real) > 1:
        hist = np.array(historial_real)
        linea_real.set_data_3d(hist[:, 0], hist[:, 1], hist[:, 2])

    q1, q2, q3 = q_interno
    q1_m, q2_m, q3_m = interno_a_motor(q1, q2, q3)
    porc_codo = 100.0 * (q3_m - Q3_MIN) / (Q3_MAX - Q3_MIN) if Q3_MAX != Q3_MIN else 0
    texto_estado.set_text(
        f"Motor obj : q1m={q1_m:+.3f} q2m={q2_m:+.3f} q3m={q3_m:+.3f}\n"
        f"Motor cmd : q1m={q1m_cmd:+.3f} q2m={q2m_cmd:+.3f} q3m={q3m_cmd:+.3f}\n"
        f"Codo: {porc_codo:5.1f} % del rango"
    )

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# POSTURA DE REPOSO
# Ajustado a los límites del Webots
q1_reposo = 0.0        # sin abducción
q2_reposo = 0.0        # cadera neutral
q3_reposo = 0.0        # rodilla en posición neutra
q1m_r, q2m_r, q3m_r = interno_a_motor(q1_reposo, q2_reposo, q3_reposo)
motors[0].setPosition(q1m_r)
motors[1].setPosition(q2m_r)
motors[2].setPosition(q3m_r)
ultima_q1_m, ultima_q2_m, ultima_q3_m = q1m_r, q2m_r, q3m_r

# Calcular posición del pie en reposo para visualización
O0_reposo, O1_reposo, O2_reposo, O3_reposo = cinematica_directa(q1_reposo, q2_reposo, q3_reposo, LADO)
pie_reposo = O3_reposo

for _ in range(30):
    robot.step(timestep)
    if MOSTRAR_EN_VIVO:
        actualizar_grafica(pie_reposo, (q1_reposo, q2_reposo, q3_reposo))

# MOVER AL PRIMER PUNTO
if len(trayectoria) > 0:
    p_inicial = trayectoria[0]
    q = cinematica_inversa(p_inicial[0], p_inicial[1], p_inicial[2], LADO)
    if q is not None:
        q1_m, q2_m, q3_m = interno_a_motor(*q)
        motors[0].setPosition(q1_m)
        motors[1].setPosition(q2_m)
        motors[2].setPosition(q3_m)
        ultima_q1_m, ultima_q2_m, ultima_q3_m = q1_m, q2_m, q3_m
        for _ in range(30):
            robot.step(timestep)
            if MOSTRAR_EN_VIVO:
                actualizar_grafica(p_inicial, q)

DURACION_SEGMENTO = 0.5
indice_segmento = 0
t_segmento = 0.0
trayectoria_terminada = False
paso_actual = 0

if len(segmentos) == 0:
    print("Sin segmentos de trayectoria. La pierna esta en reposo.")
    # mostrando la pierna quieta
    while robot.step(timestep) != -1:
        # No hacemos nada, solo mantenemos la gráfica actualizada
        if MOSTRAR_EN_VIVO:
            # Usamos la postura reposo
            q_actual = (q1_reposo, q2_reposo, q3_reposo)
            # Mantenemos el punto del pie de la pierna real
            actualizar_grafica(trayectoria[0] if trayectoria else np.zeros(3), q_actual)
else:
    print("Iniciando seguimiento de trayectoria")
    while robot.step(timestep) != -1:
        if trayectoria_terminada:
            print("Terminada.")
            for _ in range(10):
                robot.step(timestep)
            break

        dt = timestep / 1000.0
        t_segmento += dt
        s = min(t_segmento / DURACION_SEGMENTO, 1.0)

        if MODO_SEGUIMIENTO == 'bezier':
            u = 3 * s**2 - 2 * s**3
            p0, p1, p2, p3 = segmentos[indice_segmento]
            punto = bezier_cubico(p0, p1, p2, p3, u)
        else:
            p0, p1 = segmentos[indice_segmento]
            punto = p0 + s * (p1 - p0)

        historial_real.append(punto.copy())

        q = cinematica_inversa(punto[0], punto[1], punto[2], LADO)
        if q is not None:
            q1_m, q2_m, q3_m = interno_a_motor(*q)
            motors[0].setPosition(q1_m)
            motors[1].setPosition(q2_m)
            motors[2].setPosition(q3_m)
            ultima_q1_m, ultima_q2_m, ultima_q3_m = q1_m, q2_m, q3_m

            paso_actual += 1
            if paso_actual % PLOT_CADA_N_PASOS == 0 and MOSTRAR_EN_VIVO:
                actualizar_grafica(punto, q)

        if t_segmento >= DURACION_SEGMENTO:
            t_segmento = 0.0
            indice_segmento += 1
            if indice_segmento >= len(segmentos):
                trayectoria_terminada = True


#REPORTE FINAL DE ERROR
if len(historial_real) > 0:
    dt_sim = timestep / 1000.0
    errores, rmse = calcular_errores(historial_real, referencia)
    print(f"\nRMSE (distancia real vs. trayectoria planeada): {rmse:.6f} m\n")

    velocidad, aceleracion = calcular_velocidad_aceleracion(historial_real, dt_sim)

    plt.ioff()
    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':', label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame"); axs[0].set_ylabel("Error perpendicular (m)")
    axs[0].set_title("Error de seguimiento de trayectoria")
    axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3)

    tiempo_v = np.arange(len(velocidad)) * dt_sim
    tiempo_a = np.arange(len(aceleracion)) * dt_sim
    axs[1].plot(tiempo_v, velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Tiempo (s)"); axs[1].set_ylabel("Magnitud")
    axs[1].set_title("Velocidad y aceleración de la trayectoria")
    axs[1].legend(); axs[1].grid(alpha=0.3)
    fig2.tight_layout()
    plt.show()
else:
    print("No se registraron puntos.")