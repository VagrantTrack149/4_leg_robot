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
)

# ============================================================
# VARIABLE PARA DESACTIVAR GRÁFICAS EN VIVO (más rápido)
# ============================================================
MOSTRAR_EN_VIVO = False   # Cambiar a False para ejecutar sin gráficas en tiempo real

# Tipo de trayectoria base: 'espiral', 'lissajous', 'escalon'
TIPO_TRAYECTORIA = 'escalon'

# Modo de seguimiento: 'bezier', 'lineal', 'poligonal'
MODO_SEGUIMIENTO = 'lineal'  

# Parámetros para el modo poligonal (número de subsegmentos por tramo)
SUBSEGMENTOS_POLIGONAL = 10


# PARÁMETROS PROPIOS DE LA PATA FL
LADO = 1  # pata delantera izquierda
PLOT_CADA_N_PASOS = 1


# GENERACIÓN DE LA TRAYECTORIA BASE

if TIPO_TRAYECTORIA == 'espiral':
    trayectoria = generar_espiral(
        centro=(0.0, 0.0, -0.3820),
        R0=0.06, Rf=0.008,
        vueltas=3, n_puntos=15,
        z_ini=-0.360, z_fin=-0.425
    )
elif TIPO_TRAYECTORIA == 'lissajous':
    trayectoria = generar_lissajous(
        centro=(0.0, 0.04, -0.35),   # ajustado para FL
        radio=0.015,
        frecuencias=(2, 3, 5),
        n_puntos=100
    )
elif TIPO_TRAYECTORIA == 'escalon':
    trayectoria = generar_escalon()
else:
    raise ValueError("TIPO_TRAYECTORIA no válido. Usa 'espiral', 'lissajous' o 'escalon'.")

# Filtrar puntos alcanzables
trayectoria = [p for p in trayectoria if punto_alcanzable(p, LADO)]
if len(trayectoria) < 2:
    raise RuntimeError("No hay suficientes puntos alcanzables en la trayectoria.")


# PREPARACIÓN DE SEGMENTOS SEGÚN MODO_SEGUIMIENTO


if MODO_SEGUIMIENTO == 'bezier':
    segmentos = preparar_trayectoria_completa(trayectoria, LADO)
    referencia = generar_bezier_completa(segmentos)  # curva Bézier completa
    print(f"Segmentos Bézier creados: {len(segmentos)}")

elif MODO_SEGUIMIENTO == 'lineal':
    # Segmentos rectos entre puntos consecutivos
    segmentos = [(trayectoria[i], trayectoria[i+1]) for i in range(len(trayectoria)-1)]
    referencia = np.array(trayectoria)   # polilínea original
    print(f"Segmentos lineales creados: {len(segmentos)}")

elif MODO_SEGUIMIENTO == 'poligonal':
    # Subdividir cada tramo en varios segmentos rectos (poligonal)
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
    referencia = np.array(trayectoria)   # polilínea original
    print(f"Segmentos poligonales creados: {len(segmentos)} (subdivisión {SUBSEGMENTOS_POLIGONAL})")

else:
    raise ValueError("MODO_SEGUIMIENTO no válido. Usa 'bezier', 'lineal' o 'poligonal'.")


# CONFIGURACIÓN DE WEBOTS


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor_names = [
    "front left shoulder abduction motor",
    "front left shoulder rotation motor",
    "front left elbow motor"
]
motors = [robot.getDevice(name) for name in motor_names]
for m in motors:
    m.setVelocity(2.0)
    m.setAcceleration(5.0)


# GRÁFICA 3D EN VIVO (solo si MOSTRAR_EN_VIVO es True)

if MOSTRAR_EN_VIVO:
    plt.ion()
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f"Webots - pata FL ({MODO_SEGUIMIENTO})")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_xlim(-0.3, 0.4)
    ax.set_ylim(-0.2, 0.5)
    ax.set_zlim(-0.5, 0.0)
    ax.set_box_aspect([1, 1, 1])

    ax.scatter([0], [0], [0], color='red', s=80, marker='^', label='Cadera')

    waypoints = np.array(trayectoria)
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
               color='blue', s=25, alpha=0.5, label='Waypoints')

    # Mostrar la referencia según el modo
    if MODO_SEGUIMIENTO == 'bezier':
        ax.plot(referencia[:, 0], referencia[:, 1], referencia[:, 2],
                color='lime', linewidth=1.5, linestyle='--', label='Bézier planeada')
    else:
        # Para lineal y poligonal mostramos la polilínea original
        ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                color='lime', linewidth=1.5, linestyle='--', label='Trayectoria planeada')

    linea_real, = ax.plot([], [], [], color='crimson', linewidth=1.5, label='Real (alcanzada)')

    # Pata: 3 segmentos
    l01, = ax.plot([], [], [], color='orange', lw=4)
    l12, = ax.plot([], [], [], color='orange', lw=4)
    l23, = ax.plot([], [], [], color='orange', lw=4)
    articulaciones = ax.scatter([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], color='orange', s=30)
    pie, = ax.plot([], [], [], 'o', color='gold', markersize=9, markeredgecolor='black', label='Pie')

    ax.legend(loc='upper left', fontsize=8)
    texto_estado = fig.text(0.02, 0.95, "", fontsize=8, family='monospace', va='top')

    plt.tight_layout()
    plt.show(block=False)
    fig.canvas.draw()
    fig.canvas.flush_events()


    # SEGUNDA FIGURA: ÁNGULOS DE MOTOR EN VIVO

    fig_ang, ax_ang = plt.subplots(figsize=(8, 5))
    ax_ang.set_title("Ángulos de motor en vivo")
    ax_ang.set_xlabel("Tiempo (s)")
    ax_ang.set_ylabel("Ángulo de motor (rad)")

    linea_q1m, = ax_ang.plot([], [], color='steelblue', linewidth=1.8, label='q1_m (abducción)')
    linea_q2m, = ax_ang.plot([], [], color='seagreen', linewidth=1.8, label='q2_m (rotación)')
    linea_q3m, = ax_ang.plot([], [], color='darkorange', linewidth=1.8, label='q3_m (codo)')

    ax_ang.axhline(Q1_MIN, color='steelblue', linestyle=':', alpha=0.5)
    ax_ang.axhline(Q1_MAX, color='steelblue', linestyle=':', alpha=0.5)
    ax_ang.axhline(Q2_MIN, color='seagreen', linestyle=':', alpha=0.5)
    ax_ang.axhline(Q2_MAX, color='seagreen', linestyle=':', alpha=0.5)
    ax_ang.axhline(Q3_MIN, color='darkorange', linestyle=':', alpha=0.5)
    ax_ang.axhline(Q3_MAX, color='darkorange', linestyle=':', alpha=0.5)

    ax_ang.set_ylim(min(Q1_MIN, Q2_MIN, Q3_MIN) - 0.2, max(Q1_MAX, Q2_MAX, Q3_MAX) + 0.2)
    ax_ang.legend(loc='upper right', fontsize=8)
    ax_ang.grid(alpha=0.3)
    fig_ang.tight_layout()
    plt.show(block=False)
    fig_ang.canvas.draw()
    fig_ang.canvas.flush_events()
else:
    # Variables dummy para evitar errores si no se usan
    fig = None
    ax = None
    linea_real = None
    l01 = l12 = l23 = None
    articulaciones = None
    pie = None
    texto_estado = None
    fig_ang = None
    ax_ang = None
    linea_q1m = linea_q2m = linea_q3m = None


# HISTORIALES Y FUNCIÓN DE ACTUALIZACIÓN


historial_real = []
historial_motor = []
historial_tiempo = []
_tiempo_acumulado = [0.0]

def actualizar_grafica(punto_pie, q_interno, avanzar_tiempo=True):
    if not MOSTRAR_EN_VIVO:
        # Solo actualizar el tiempo si se solicita
        if avanzar_tiempo:
            _tiempo_acumulado[0] += timestep / 1000.0
        return

    O0, O1, O2, O3 = cinematica_directa(*q_interno, LADO)

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    articulaciones._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                                  [O0[1], O1[1], O2[1], O3[1]],
                                  [O0[2], O1[2], O2[2], O3[2]])
    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    if len(historial_real) > 1:
        hist = np.array(historial_real)
        linea_real.set_data_3d(hist[:, 0], hist[:, 1], hist[:, 2])

    q1, q2, q3 = q_interno
    q1_m, q2_m, q3_m = interno_a_motor(q1, q2, q3)  # usa el offset original
    porc_codo = 100.0 * (q3_m - Q3_MIN) / (Q3_MAX - Q3_MIN)
    texto_estado.set_text(
        f"q_interno  q1={q1:+.3f} q2={q2:+.3f} q3={q3:+.3f}\n"
        f"q_motor    q1={q1_m:+.3f} q2={q2_m:+.3f} q3={q3_m:+.3f}\n"
        f"Codo: {porc_codo:5.1f} % del rango de flexión"
    )

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    if avanzar_tiempo:
        _tiempo_acumulado[0] += timestep / 1000.0
    t = _tiempo_acumulado[0]
    historial_motor.append((q1_m, q2_m, q3_m))
    historial_tiempo.append(t)

    hm = np.array(historial_motor)
    linea_q1m.set_data(historial_tiempo, hm[:, 0])
    linea_q2m.set_data(historial_tiempo, hm[:, 1])
    linea_q3m.set_data(historial_tiempo, hm[:, 2])
    ax_ang.set_xlim(0, max(t, 0.1))

    fig_ang.canvas.draw_idle()
    fig_ang.canvas.flush_events()


# POSTURA DE REPOSO (pierna vertical)


q1_reposo = math.pi / 2
q2_reposo = 0.0
q3_reposo = 0.0
q1m_r, q2m_r, q3m_r = interno_a_motor(q1_reposo, q2_reposo, q3_reposo)
motors[0].setPosition(q1m_r)
motors[1].setPosition(q2m_r)
motors[2].setPosition(q3m_r)
for _ in range(30):
    robot.step(timestep)
    if MOSTRAR_EN_VIVO:
        actualizar_grafica(np.array([0.0, 0.0, -(0.2142 + 0.2142)]), (q1_reposo, q2_reposo, q3_reposo))


# MOVER AL PRIMER PUNTO DE LA TRAYECTORIA


p_inicial = trayectoria[0]
q = cinematica_inversa(p_inicial[0], p_inicial[1], p_inicial[2], LADO)
if q is not None:
    q1_m, q2_m, q3_m = interno_a_motor(*q)
    motors[0].setPosition(q1_m)
    motors[1].setPosition(q2_m)
    motors[2].setPosition(q3_m)
    for _ in range(30):
        robot.step(timestep)
        if MOSTRAR_EN_VIVO:
            actualizar_grafica(p_inicial, q)


# BUCLE PRINCIPAL


DURACION_SEGMENTO = 0.5
indice_segmento = 0
t_segmento = 0.0
trayectoria_terminada = False
paso_actual = 0

print("Iniciando seguimiento de trayectoria")
while robot.step(timestep) != -1:
    if not segmentos or trayectoria_terminada:
        print("Terminada.")
        for _ in range(10):
            robot.step(timestep)
        break

    dt = timestep / 1000.0
    t_segmento += dt
    s = min(t_segmento / DURACION_SEGMENTO, 1.0)

    # Interpolación según el modo
    if MODO_SEGUIMIENTO == 'bezier':
        u = 3 * s**2 - 2 * s**3   # easing cúbico
        p0, p1, p2, p3 = segmentos[indice_segmento]
        punto = bezier_cubico(p0, p1, p2, p3, u)
    else:
        # Lineal o poligonal: interpolación lineal entre extremos del segmento
        p0, p1 = segmentos[indice_segmento]
        punto = p0 + s * (p1 - p0)

    historial_real.append(punto.copy())

    q = cinematica_inversa(punto[0], punto[1], punto[2], LADO)
    if q is not None:
        q1_m, q2_m, q3_m = interno_a_motor(*q)
        motors[0].setPosition(q1_m)
        motors[1].setPosition(q2_m)
        motors[2].setPosition(q3_m)

        paso_actual += 1
        if paso_actual % PLOT_CADA_N_PASOS == 0 and MOSTRAR_EN_VIVO:
            actualizar_grafica(punto, q)

    if t_segmento >= DURACION_SEGMENTO:
        t_segmento = 0.0
        indice_segmento += 1
        if indice_segmento >= len(segmentos):
            trayectoria_terminada = True


# REPORTE FINAL DE ERROR (siempre se muestra, independientemente de MOSTRAR_EN_VIVO)


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