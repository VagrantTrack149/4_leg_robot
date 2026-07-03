from controller import Robot
import numpy as np


# ROBOT

robot = Robot()
timestep = int(robot.getBasicTimeStep())


# PARAMETROS DEL ROBOT
a1 = 0.0528
L2 = 0.2142
L3 = 0.2142

LADO_FL = 1  # signo de la pata delantera izquierda


# Motores
motors = [
    robot.getDevice("front left shoulder abduction motor"),
    robot.getDevice("front left shoulder rotation motor"),
    robot.getDevice("front left elbow motor")
]

for motor in motors:
    motor.setVelocity(10.0)


# CINEMATICA INVERSA 
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

    # Truncar a los limites reales de los motores en Webots

    q2 = np.clip(q2, -0.5, 0.5)
    q3 = np.clip(q3, -0.45, 0.0)

    return q1, q2, q3


Q1_MIN, Q1_MAX = 0.0, np.pi          # limite geometrico del hombro
Q2_MIN, Q2_MAX = -0.5, 0.5           # limite real del motor de rotacion de hombro 
Q3_MIN, Q3_MAX = -0.45, 0.0          # limite real del motor de 


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

    # Verificar tambien los angulos articulares resultantes contra los limites reales del motor
    C3 = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    C3 = np.clip(C3, -1.0, 1.0)
    q3 = -np.arctan2(np.sqrt(1.0 - C3**2), C3)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)
    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    return (Q1_MIN <= q1 <= Q1_MAX and
            Q2_MIN <= q2 <= Q2_MAX and
            Q3_MIN <= q3 <= Q3_MAX)


# TRAYECTORIA: ESPIRAL CONICA
def generar_espiral_conica(centro, R0=0.12, Rf=0.03, vueltas=3,
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


# Centro de la espiral en el marco local de la pata FL.
CENTRO_ESPIRAL = (0.0, 0.02, -0.48)

trayectoria = generar_espiral_conica(
    CENTRO_ESPIRAL,
    R0=0.025, Rf=0.01,
    vueltas=3, n_puntos=30,
    z_inicial=-0.43, z_final=-0.47
)

# Se descartan los puntos que quedan fuera del espacio de trabajo de la pata
trayectoria = [p for p in trayectoria if punto_alcanzable(p[0], p[1], p[2], LADO_FL)]


# TANGENTES Y CURVAS BEZIER 

def calcular_tangentes(puntos):
    n = len(puntos)
    tangentes = []
    for i in range(n):
        if i == 0:
            tang = puntos[1] - puntos[0]
        elif i == n - 1:
            tang = puntos[-1] - puntos[-2]
        else:
            tang = (puntos[i + 1] - puntos[i - 1]) / 2.0

        if i < n - 1:
            dist = np.linalg.norm(puntos[i + 1] - puntos[i])
            norma = np.linalg.norm(tang)
            if norma > 0:
                tang = tang / norma * dist

        tangentes.append(tang)
    return tangentes


def bezier_cubico(p0, p1, p2, p3, t):
    u = 1 - t
    return u**3 * p0 + 3 * u**2 * t * p1 + 3 * u * t**2 * p2 + t**3 * p3


def construir_segmentos_bezier(puntos, factor_curv=1.0):
    tangentes = calcular_tangentes(puntos)
    segmentos = []
    for i in range(len(puntos) - 1):
        p0 = puntos[i]
        p3 = puntos[i + 1]
        p1 = p0 + (1.0 / 3.0) * tangentes[i] * factor_curv
        p2 = p3 - (1.0 / 3.0) * tangentes[i + 1] * factor_curv
        segmentos.append((p0, p1, p2, p3))
    return segmentos


segmentos = construir_segmentos_bezier(trayectoria)


# POSTURA INICIAL, primer punto de la espiral

if len(trayectoria) > 0:
    p_ini = trayectoria[0]
    q1, q2, q3 = cinematica_inversa(p_ini[0], p_ini[1], p_ini[2], LADO_FL)

    motors[0].setPosition(q1)
    motors[1].setPosition(q2)
    motors[2].setPosition(q3)


# BUCLE PRINCIPAL

DURACION_SEGMENTO = 1.0  # segundos que tarda en recorrer cada tramo bezier

indice_segmento = 0
t_segmento = 0.0

while robot.step(timestep) != -1:

    if len(segmentos) == 0:
        continue

    dt = timestep / 1000.0
    t_segmento += dt

    s = np.clip(t_segmento / DURACION_SEGMENTO, 0.0, 1.0)
    u = 3 * s**2 - 2 * s**3  # suavizado smoothstep

    p0, p1, p2, p3 = segmentos[indice_segmento]
    punto = bezier_cubico(p0, p1, p2, p3, u)

    q1, q2, q3 = cinematica_inversa(punto[0], punto[1], punto[2], LADO_FL)

    motors[0].setPosition(q1)
    motors[1].setPosition(q2)
    motors[2].setPosition(q3)

    if t_segmento >= DURACION_SEGMENTO:
        t_segmento = 0.0
        indice_segmento += 1

        if indice_segmento >= len(segmentos):
            indice_segmento = 0  # repite la espiral en bucle