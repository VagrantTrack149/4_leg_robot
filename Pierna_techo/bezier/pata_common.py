"""
pata_common.py
================
Funciones de cinemática y generación de trayectorias para una pata
tipo "Spot" (abducción de cadera + muslo + pantorrilla).

Toda la cinemática (constantes a1, L2, L3 y las fórmulas de
cinematica_inversa) está tomada TAL CUAL de my_controller.py, quitando
únicamente lo relacionado con Webots (Robot, motores, timestep, etc).

cinematica_directa() es la función complementaria (no existía en
my_controller.py porque ahí solo se enviaban posiciones a los motores
reales); aquí se construye para que sea consistente e invertible con
cinematica_inversa(), y así poder dibujar la pierna (hombro -> rodilla
-> pie) igual que hacía el wireframe de my_controller, pero en 3D
completo (ahí el wireframe ignoraba q1; aquí sí se usa).
"""

import numpy as np

# -------------------------------------------------------------------
# Parámetros geométricos (idénticos a my_controller.py)
# -------------------------------------------------------------------
a1 = 0.08       # Offset lateral de cadera (L_hip)
L2 = 0.34       # Largo del muslo (L_thigh)
L3 = 0.35       # Largo de la pantorrilla (L_calf)

# Tolerancia numérica para el chequeo de alcanzabilidad
_TOL = 1e-6


# -------------------------------------------------------------------
# CINEMÁTICA INVERSA (misma lógica que my_controller.py)
# -------------------------------------------------------------------
def cinematica_inversa(px, py, pz, lado):
    """
    Misma fórmula que en my_controller.py. Adicionalmente, si el punto
    pedido está fuera del alcance geométrico real de la pata (fuera
    de la corona L3-L2 ... L2+L3), devuelve None en vez de forzar
    (clip) una solución inválida, para que pierna_Techo_prueba_3.py
    pueda detectarlo (puntos_pata ya contempla ese caso).
    """
    # Posición relativa a la cadera
    x = px
    y = py - lado * a1   # lado: 1=izquierda, -1=derecha
    z = pz

    # Ángulo de abducción de cadera (q1)
    q1 = np.arctan2(y, -z)

    # Distancia desde la cadera hasta el pie en el plano sagital
    L = np.hypot(y, z)
    d = np.hypot(x, L)

    # Chequeo de alcanzabilidad real (triángulo L2, L3, d)
    if d > (L2 + L3 + _TOL) or d < (abs(L2 - L3) - _TOL):
        return None

    # Ángulo de la rodilla (q3) - ley de cosenos
    cos_knee = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    q3 = np.pi - np.arccos(cos_knee)

    # Ángulo de flexión de cadera (q2)
    alpha = np.arctan2(x, L)
    beta = np.arccos(np.clip((L2**2 + d**2 - L3**2) / (2 * L2 * d), -1.0, 1.0))
    q2 = alpha + beta

    return q1, q2, q3


# -------------------------------------------------------------------
# CINEMÁTICA DIRECTA (complemento necesario para dibujar la pierna)
# -------------------------------------------------------------------
def cinematica_directa(q1, q2, q3, lado):
    """
    Devuelve los 4 puntos de la cadena cinemática (relativos al
    origen de la pata, sin sumar CADERA todavía):

        O0 -> punto de anclaje al cuerpo (0,0,0)
        O1 -> pivote de abducción, tras el offset a1  (equivale a H
              en my_controller.py: H = [0, lado*a1, 0])
        O2 -> rodilla
        O3 -> pie

    Es la función inversa de cinematica_inversa(): si le pasas los
    ángulos que ésta devolvió para un punto (px,py,pz), O3 reconstruye
    ese mismo (px,py,pz).
    """
    O0 = np.array([0.0, 0.0, 0.0])
    O1 = np.array([0.0, lado * a1, 0.0])

    # Dirección radial en el plano de abducción (misma convención que
    # q1 = arctan2(y, -z) en cinematica_inversa)
    e_r = np.array([0.0, np.sin(q1), -np.cos(q1)])

    # Rodilla: avanza L2 en el plano sagital/abducción
    O2 = O1 + L2 * np.sin(q2) * np.array([1.0, 0.0, 0.0]) + L2 * np.cos(q2) * e_r

    # Pie: continúa desde la rodilla; el ángulo interior de la rodilla
    # (pi - q3, por cómo se define q3 en cinematica_inversa) hace que
    # la pantorrilla se doble hacia atrás respecto al muslo, es decir
    # el ángulo acumulado respecto al eje radial es (q2 - q3)
    O3 = O2 + L3 * np.sin(q2 - q3) * np.array([1.0, 0.0, 0.0]) + L3 * np.cos(q2 - q3) * e_r

    return [O0, O1, O2, O3]


def punto_alcanzable(p, lado):
    """True si (px,py,pz) está dentro del alcance geométrico real."""
    px, py, pz = p
    y = py - lado * a1
    z = pz
    L = np.hypot(y, z)
    d = np.hypot(px, L)
    return (abs(L2 - L3) - _TOL) <= d <= (L2 + L3 + _TOL)


# -------------------------------------------------------------------
# TRAYECTORIAS - misma lógica que TIPO/obtener_punto de my_controller.py,
# pero expresadas como listas de waypoints (px,py,pz) en vez de una
# función continua de t, porque pierna_Techo_prueba_3.py arma después
# segmentos Bézier entre waypoints.
# -------------------------------------------------------------------
def generar_espiral(centro, R0, Rf, vueltas, n_puntos, z_ini, z_fin):
    """
    Espiral cónica: mismo principio que el caso 'espiral' de
    obtener_punto() en my_controller.py (radio variable * cos/sin de
    un ángulo creciente, con z variando junto con el progreso),
    generalizado con radio inicial/final, número de vueltas y rango
    de altura explícitos en vez de estar atados al ciclo de 4s de t_mod.
    """
    cx, cy, cz = centro
    puntos = []
    for i in range(n_puntos):
        s = i / (n_puntos - 1) if n_puntos > 1 else 0.0
        angulo = 2 * np.pi * vueltas * s
        r = R0 + (Rf - R0) * s
        px = cx + r * np.cos(angulo)
        py = cy + r * np.sin(angulo)
        pz = z_ini + (z_fin - z_ini) * s
        puntos.append(np.array([px, py, pz]))
    return puntos


def generar_lissajous(centro, radio, frecuencias, n_puntos):
    """
    Misma idea que el caso 'lissajous' de obtener_punto() en
    my_controller.py: cada eje (x,y,z) es un seno con su propia
    frecuencia, generando una figura de Lissajous en 3D.
    """
    cx, cy, cz = centro
    fx, fy, fz = frecuencias
    puntos = []
    for i in range(n_puntos):
        s = i / (n_puntos - 1) if n_puntos > 1 else 0.0
        t = 2 * np.pi * s
        px = cx + radio * np.sin(fx * t)
        py = cy + radio * np.sin(fy * t)
        pz = cz + radio * np.sin(fz * t)
        puntos.append(np.array([px, py, pz]))
    return puntos


def generar_escalon(centro=(0.0, 0.0, -0.38), amplitud_x=0.05, amplitud_z=0.05,
                     lado=1, a1_offset=None, n_escalones=4):
    """
    Misma idea que el caso 'escalon' de obtener_punto() en
    my_controller.py: onda cuadrada, alternando entre dos posiciones
    en X y dos alturas en Z (ahí eran px=0.1/-0.1 y pz=ALTURA_NEUTRA /
    ALTURA_NEUTRA+0.1 según t_mod). Aquí se generaliza a un número de
    escalones alrededor de un centro, devolviendo 2 waypoints por
    escalón (posición baja y posición alta) para que
    preparar_trayectoria_completa() los conecte.
    """
    cx, cy, cz = centro
    if a1_offset is None:
        a1_offset = a1  # mismo offset lateral por defecto que en my_controller

    puntos = []
    for i in range(n_escalones):
        signo = 1 if i % 2 == 0 else -1
        px = cx + signo * amplitud_x
        py = cy + lado * a1_offset
        # Alterna entre altura neutra y altura elevada, igual que
        # ALTURA_NEUTRA / ALTURA_NEUTRA + 0.1 en my_controller.py
        pz_bajo = cz
        pz_alto = cz + amplitud_z
        puntos.append(np.array([px, py, pz_bajo]))
        puntos.append(np.array([px, py, pz_alto]))
    return puntos


# -------------------------------------------------------------------
# UTILIDADES DE TRAYECTORIA (infraestructura para el seguimiento
# Bézier de pierna_Techo_prueba_3.py; no existían en my_controller.py
# porque ahí el movimiento era continuo en t, sin segmentos)
# -------------------------------------------------------------------
def reorganizar_trayectoria(puntos, actual):
    """Reordena la lista para empezar por el extremo más cercano a 'actual'."""
    if len(puntos) == 0:
        return puntos
    d_inicio = np.linalg.norm(np.asarray(puntos[0]) - np.asarray(actual))
    d_final = np.linalg.norm(np.asarray(puntos[-1]) - np.asarray(actual))
    if d_final < d_inicio:
        return list(reversed(puntos))
    return list(puntos)


def bezier_cubico(p0, p1, p2, p3, t):
    """Curva de Bézier cúbica estándar, evaluada en t (0..1)."""
    p0, p1, p2, p3 = (np.asarray(p) for p in (p0, p1, p2, p3))
    return ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + \
           3 * (1 - t) * (t ** 2) * p2 + (t ** 3) * p3


def preparar_trayectoria_completa(puntos, lado):
    """
    Convierte una lista de waypoints en segmentos Bézier cúbicos
    (p0,p1,p2,p3) consecutivos, con puntos de control tipo
    Catmull-Rom para que la trayectoria sea suave y pase por todos
    los waypoints (los puntos de control se calculan a partir de los
    vecinos, así el segmento i conecta con el i+1 sin quiebres bruscos).
    """
    pts = [np.asarray(p, dtype=float) for p in puntos]
    n = len(pts)
    segmentos = []
    if n < 2:
        return segmentos

    for i in range(n - 1):
        p0 = pts[i]
        p3 = pts[i + 1]

        p_prev = pts[i - 1] if i - 1 >= 0 else p0
        p_next = pts[i + 2] if i + 2 < n else p3

        # Tangentes tipo Catmull-Rom -> puntos de control Bézier
        m0 = (p3 - p_prev) / 2.0
        m1 = (p_next - p0) / 2.0

        p1 = p0 + m0 / 3.0
        p2 = p3 - m1 / 3.0

        segmentos.append((p0, p1, p2, p3))

    return segmentos


def generar_bezier_completa(segmentos, puntos_por_segmento=60):
    """Muestrea densamente todos los segmentos Bézier -> curva de referencia."""
    muestras = []
    for (p0, p1, p2, p3) in segmentos:
        for t in np.linspace(0, 1, puntos_por_segmento):
            muestras.append(bezier_cubico(p0, p1, p2, p3, t))
    return np.array(muestras)


def calcular_errores(historial, bezier_planeada):
    """
    Para cada punto real del historial, distancia mínima a la curva
    Bézier planeada (muestreada densamente). Devuelve (errores, rmse).
    """
    hist = np.asarray(historial)
    plan = np.asarray(bezier_planeada)
    if len(hist) == 0 or len(plan) == 0:
        return np.array([]), 0.0

    errores = np.empty(len(hist))
    for i, p in enumerate(hist):
        dist = np.linalg.norm(plan - p, axis=1)
        errores[i] = dist.min()

    rmse = np.sqrt(np.mean(errores ** 2))
    return errores, rmse


def calcular_velocidad_aceleracion(historial, dt):
    """Velocidad y aceleración (magnitudes) del historial real vía diferencias finitas."""
    hist = np.asarray(historial)
    if len(hist) < 2:
        return np.array([]), np.array([])

    diffs = np.diff(hist, axis=0)
    velocidad = np.linalg.norm(diffs, axis=1) / dt

    if len(velocidad) < 2:
        return velocidad, np.array([])

    aceleracion = np.diff(velocidad) / dt
    return velocidad, aceleracion