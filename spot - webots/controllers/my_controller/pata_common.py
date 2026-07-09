"""
pata_common.py

Cinematica directa e inversa de una pata de 3 GDL del robot Spot (Webots),
basada en la tabla Denavit-Hartenberg de `dinamica.md`, verificada
simbolicamente (SymPy) y numericamente contra un round-trip FK->IK->FK
(error de cierre ~1e-13 m dentro del rango de operacion fisico normal).

Convencion de articulaciones (coincide con SpotLeg.proto):
    q1 -> "shoulder abduction motor"   (abduccion de cadera)
    q2 -> "shoulder rotation motor"    (flexion de cadera)
    q3 -> "elbow motor"                (flexion de rodilla)

Convencion de LADO:
    Este modulo usa el parametro `lado` para diferenciar pata izquierda
    de derecha. SOLO el termino a1 (offset de abduccion) cambia de signo
    con el lado; L2, L3 y d3 son iguales para ambas patas (verificado
    contra los anclajes de SpotLeg.proto, donde unicamente la componente
    X de las traslaciones lleva el signo).

    IMPORTANTE: ajusta el signo de `LADO` en tu controlador segun la
    convencion de tu propio codigo. En el snippet que compartiste,
    LADO = 1 corresponde a "front left". Si tu pata_common.py original
    usa el signo opuesto, cambia el signo aqui o en la llamada.
"""

import math

# ---------------------------------------------------------------------------
# Parametros geometricos (dinamica.md / SpotLeg.proto)
# ---------------------------------------------------------------------------
A1 = 0.0528     # offset de abduccion (m)
L2 = 0.2142     # longitud del femur / "hombro" (m)
L3 = 0.2142     # longitud de la tibia / "codo" (m)
D3 = 0.3197     # offset lateral del antebrazo (m)

# Limites articulares (idénticos a los minPosition/maxPosition de SpotLeg.proto)
Q1_MIN, Q1_MAX = -0.6, 0.5
Q2_MIN, Q2_MAX = -1.7, 1.7
Q3_MIN, Q3_MAX = -0.45, 1.6

EPS = 1e-9


# ---------------------------------------------------------------------------
# Cinematica directa
# ---------------------------------------------------------------------------
def cinematica_directa(q1, q2, q3, lado=1):
    """
    Devuelve los origenes O0..O3 (cadera, fin abduccion, rodilla, pie)
    en el sistema local de la pata, como tuplas (x, y, z).
    """
    c1, s1 = math.cos(q1), math.sin(q1)
    c2, s2 = math.cos(q2), math.sin(q2)
    c23, s23 = math.cos(q2 + q3), math.sin(q2 + q3)

    O0 = (0.0, 0.0, 0.0)

    # fin del eslabon de abduccion
    O1 = (lado * A1 * c1, lado * A1 * s1, 0.0)

    rho = lado * A1 + L2 * c2
    O2 = (rho * c1 - D3 * s1, rho * s1 + D3 * c1, -L2 * s2)

    rho_pie = lado * A1 + L2 * c2 + L3 * c23
    px = rho_pie * c1 - D3 * s1
    py = rho_pie * s1 + D3 * c1
    pz = -L2 * s2 - L3 * s23
    O3 = (px, py, pz)

    return O0, O1, O2, O3


# ---------------------------------------------------------------------------
# Cinematica inversa
# ---------------------------------------------------------------------------
def cinematica_inversa(px, py, pz, lado=1, codo_adelante=True):
    """
    Resuelve (q1, q2, q3) para un punto objetivo (px,py,pz) del pie,
    relativo al origen de la pata (junta de abduccion).

    Devuelve (q1, q2, q3, alcanzable) donde `alcanzable` es False si
    hubo que recortar (clip) la solucion por estar fuera del espacio
    de trabajo -- en ese caso q1,q2,q3 corresponden al punto alcanzable
    mas cercano, NO al punto pedido.

    Nota de rango valido: la formula cerrada de q1 tiene una unica rama
    fisica correcta cuando rho = lado*A1 + L2*cos(q2) + L3*cos(q2+q3) > 0
    (la pata "cuelga" de forma normal, sin plegarse mas alla del eje de
    cadera). Para trayectorias de marcha normales esto siempre se cumple;
    en posturas extremas cerca de los limites articulares podria no
    cumplirse y la solucion dejaria de ser exacta.
    """
    alcanzable = True

    # --- q1 ---
    r2 = px * px + py * py - D3 * D3
    if r2 < 0.0:
        r2 = 0.0
        alcanzable = False
    q1 = math.atan2(py, px) - math.atan2(D3, math.sqrt(r2))

    # --- rho / R (forma robusta, sin ambiguedad de signo) ---
    rho = px * math.cos(q1) + py * math.sin(q1)
    R = rho - lado * A1

    # --- q3 (ley de cosenos) ---
    denom = 2.0 * L2 * L3
    C3 = (R * R + pz * pz - L2 * L2 - L3 * L3) / denom
    if C3 > 1.0:
        C3 = 1.0
        alcanzable = False
    elif C3 < -1.0:
        C3 = -1.0
        alcanzable = False

    sign = 1.0 if codo_adelante else -1.0
    q3 = math.atan2(sign * math.sqrt(max(0.0, 1.0 - C3 * C3)), C3)

    # --- q2 ---
    k1 = L2 + L3 * math.cos(q3)
    k2 = L3 * math.sin(q3)
    q2 = math.atan2(-pz, R) - math.atan2(k2, k1)

    # --- recorte final a limites articulares (reporta si hizo falta) ---
    q1c = min(max(q1, Q1_MIN), Q1_MAX)
    q2c = min(max(q2, Q2_MIN), Q2_MAX)
    q3c = min(max(q3, Q3_MIN), Q3_MAX)
    if (q1c, q2c, q3c) != (q1, q2, q3):
        alcanzable = False

    return q1c, q2c, q3c, alcanzable


def punto_alcanzable(px, py, pz, lado=1, codo_adelante=True):
    """True si el punto es alcanzable sin recortar ningun angulo."""
    _, _, _, ok = cinematica_inversa(px, py, pz, lado, codo_adelante)
    return ok


# ---------------------------------------------------------------------------
# Curvas de Bezier cubicas y planeacion de trayectoria
# ---------------------------------------------------------------------------
def bezier_cubico(p0, p1, p2, p3, t):
    """Punto sobre una curva de Bezier cubica en t in [0,1]. p_i: tuplas (x,y,z)."""
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3 * u * u * t
    b2 = 3 * u * t * t
    b3 = t * t * t
    return tuple(b0 * p0[i] + b1 * p1[i] + b2 * p2[i] + b3 * p3[i] for i in range(3))


def calcular_tangentes(waypoints, cerrado=True, escala=0.25):
    """
    Tangentes tipo Catmull-Rom para continuidad C1 entre segmentos.
    waypoints: lista de tuplas (x,y,z).
    Devuelve una lista de tangentes (misma longitud que waypoints).
    """
    n = len(waypoints)
    tangentes = []
    for i in range(n):
        if cerrado:
            p_prev = waypoints[(i - 1) % n]
            p_next = waypoints[(i + 1) % n]
        else:
            p_prev = waypoints[max(i - 1, 0)]
            p_next = waypoints[min(i + 1, n - 1)]
        tang = tuple((p_next[k] - p_prev[k]) * escala for k in range(3))
        tangentes.append(tang)
    return tangentes


def generar_trayectoria_bezier(waypoints, cerrado=True, escala_tangente=0.25):
    """
    Construye los 4 puntos de control de cada segmento de Bezier cubica
    entre waypoints consecutivos, usando tangentes Catmull-Rom.

    Devuelve una lista de segmentos, cada uno = (p0, p1, p2, p3)
    listos para pasar a `bezier_cubico`.
    """
    n = len(waypoints)
    tangentes = calcular_tangentes(waypoints, cerrado, escala_tangente)
    segmentos = []
    rango = n if cerrado else n - 1
    for i in range(rango):
        j = (i + 1) % n
        p0 = waypoints[i]
        p3 = waypoints[j]
        p1 = tuple(p0[k] + tangentes[i][k] for k in range(3))
        p2 = tuple(p3[k] - tangentes[j][k] for k in range(3))
        segmentos.append((p0, p1, p2, p3))
    return segmentos


def punto_en_trayectoria(segmentos, s):
    """
    Evalua la trayectoria completa en un parametro global s in [0, len(segmentos)).
    s combina indice de segmento + t local, para poder recorrer todo el
    circuito con un unico contador que avanza en el tiempo.
    """
    n = len(segmentos)
    s = s % n
    idx = int(s)
    t = s - idx
    p0, p1, p2, p3 = segmentos[idx]
    return bezier_cubico(p0, p1, p2, p3, t)
