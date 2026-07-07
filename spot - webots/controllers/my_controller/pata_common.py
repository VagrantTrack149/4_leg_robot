#CODIGO AUXILIAR PARA FUNCIONES DE CINEMATICA, TRAZADO DE CURVAS Y METRICAS DE ERROR PARA LA PATA DEL ROBOT SPOT
#TIENE PARAMETROS COMUNES PARA LAS 4 PATAS.

import math
import numpy as np

# Parametros 
a1 = 0.0528
L2 = 0.2142
L3 = 0.2142

# Limites reales de los motores (rad)
Q1_MIN, Q1_MAX = -0.6, 0.5
Q2_MIN, Q2_MAX = -1.7, 1.7
Q3_MIN, Q3_MAX = -0.45, 1.6

# Conversion angulos internos -> motor
Q1_OFFSET = math.pi / 2
Q3_SIGN = -1.0


def interno_a_motor(q1, q2, q3):
    #Convierte angulos internos de la cinematica a angulos de motor.
    return q1 - Q1_OFFSET, q2, Q3_SIGN * q3



# Cinematica inversa / directa

def cinematica_inversa(px, py, pz, lado):
    
    #Devuelve (q1, q2, q3) en angulos internos si el punto es alcanzable y respeta los limites reales de los motores, o None si no lo es.
    py_l = py * lado
    ratio = py_l / a1
    if abs(ratio) > 1.0:
        return None
    q1 = math.acos(max(-1.0, min(1.0, ratio)))

    O1_z = -a1 * math.sin(q1)
    dx = px
    dz = pz - O1_z
    R = math.hypot(dx, dz)
    R = min(R, L2 + L3 - 1e-6)

    cos_q3 = (R ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3 = -math.acos(cos_q3)

    k1 = L2 + L3 * math.cos(q3)
    k2 = L3 * math.sin(q3)
    q2 = math.atan2(dx, -dz) - math.atan2(k2, k1)
    q2 = math.atan2(math.sin(q2), math.cos(q2))

    q1_m, q2_m, q3_m = interno_a_motor(q1, q2, q3)
    if not (Q1_MIN <= q1_m <= Q1_MAX and
            Q2_MIN <= q2_m <= Q2_MAX and
            Q3_MIN <= q3_m <= Q3_MAX):
        return None
    return q1, q2, q3


def punto_alcanzable(p, lado):
    return cinematica_inversa(p[0], p[1], p[2], lado) is not None

#usado unicamente para simulacion y pruebas de alcanzabilidad, no es necesario para el controlador real
def cinematica_directa(q1, q2, q3, lado):
    #Devuelve O0 (cadera), O1 (hombro), O2 (codo), O3 (pie) en el marco local de la pata
    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
    O0 = np.array([0.0, 0.0, 0.0])
    O1 = np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2 = O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3 = O2 + np.array([L3 * s23, 0.0, -L3 * c23])
    return O0, O1, O2, O3



# Trayectoria: espiral conica

def generar_espiral(centro, R0, Rf, vueltas, n_puntos, z_ini, z_fin):
    puntos = []
    for i in range(n_puntos):
        t = i / (n_puntos - 1)
        radio = R0 - (R0 - Rf) * t
        angulo = 2 * np.pi * vueltas * t
        x = centro[0] + radio * np.cos(angulo)
        y = centro[1] + radio * np.sin(angulo)
        z = z_ini + (z_fin - z_ini) * t
        puntos.append(np.array([x, y, z]))
    return puntos

def generar_lissajous(centro=(0.0, 0.0, -0.27), radio=0.02,
                      frecuencias=(2, 3, 5), n_puntos=150):
    
    #Genera una trayectoria de Lissajous en 3D con diferentes frecuencias. centro: (cx, cy, cz)  radio: amplitud en cada eje (misma para los tres) 
    #frecuencias: (fx, fy, fz) multiplicadores de t, n_puntos: número de puntos
    
    puntos = []
    for t in np.linspace(0, 2*np.pi, n_puntos, endpoint=False):
        x = centro[0] + radio * np.cos(frecuencias[0] * t)
        y = centro[1] + radio * np.sin(frecuencias[1] * t)
        z = centro[2] + radio * np.sin(frecuencias[2] * t)
        puntos.append(np.array([x, y, z]))
    return puntos
def generar_escalon():
    puntos = []
    puntos.append(np.array([0.1, 0.0, -0.48]))   #  baja
    puntos.append(np.array([-0.2, 0.01, -0.36]))   #  alta
    return puntos



# Curvas de Bezier cubicas

def bezier_cubico(p0, p1, p2, p3, t):
    u = 1.0 - t
    return u ** 3 * p0 + 3 * u ** 2 * t * p1 + 3 * u * t ** 2 * p2 + t ** 3 * p3


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


def segmento_bezier_valido(p0, p1, p2, p3, lado, muestras=20):
    for u in np.linspace(0.0, 1.0, muestras):
        punto = bezier_cubico(p0, p1, p2, p3, u)
        if not punto_alcanzable(punto, lado):
            return False
    return True


def preparar_segmento(p0, p3, tang_in, tang_out, lado):
    for factor in (1.5, 1.0, 0.6, 0.3, 0.0):
        p1 = p0 + (1.0 / 3.0) * tang_in * factor
        p2 = p3 - (1.0 / 3.0) * tang_out * factor
        if segmento_bezier_valido(p0, p1, p2, p3, lado):
            return (p0, p1, p2, p3)
    p1 = p0 + (1.0 / 3.0) * tang_in * 0.0
    p2 = p3 - (1.0 / 3.0) * tang_out * 0.0
    return (p0, p1, p2, p3)


def preparar_trayectoria_completa(puntos, lado):
    tang = calcular_tangentes(puntos)
    segmentos = []
    for i in range(len(puntos) - 1):
        seg = preparar_segmento(puntos[i], puntos[i + 1], tang[i], tang[i + 1], lado)
        segmentos.append(seg)
    return segmentos


def generar_bezier_completa(segmentos, puntos_por_segmento=30):
    pts = []
    for p0, p1, p2, p3 in segmentos:
        for u in np.linspace(0.0, 1.0, puntos_por_segmento, endpoint=False):
            pts.append(bezier_cubico(p0, p1, p2, p3, u))
    if segmentos:
        p0, p1, p2, p3 = segmentos[-1]
        pts.append(bezier_cubico(p0, p1, p2, p3, 1.0))
    return np.array(pts)


def reorganizar_trayectoria(puntos, pos_inicial):
    #Decide si recorrer la trayectoria en orden normal o invertido segun cual extremo esta más cerca de la posicion actual de la pata.
    if len(puntos) == 0:
        return puntos
    puntos_arr = np.array(puntos)
    inicio = puntos_arr[0]
    fin = puntos_arr[-1]
    dist_inicio = np.linalg.norm(inicio - pos_inicial)
    dist_fin = np.linalg.norm(fin - pos_inicial)
    if dist_fin < dist_inicio:
        puntos_arr = puntos_arr[::-1]
    return [p for p in puntos_arr]



# Metricas de error: distancia a la ruta planeada, RMSE, velocidad, aceleracion

def distancia_punto_segmento(P, A, B):
    AB = B - A
    AP = P - A
    largo2 = np.dot(AB, AB)
    if largo2 < 1e-12:
        return np.linalg.norm(P - A)
    t = np.clip(np.dot(AP, AB) / largo2, 0.0, 1.0)
    proyeccion = A + t * AB
    return np.linalg.norm(P - proyeccion)


def distancia_a_polilinea(P, polilinea):
    mejor = np.inf
    for i in range(len(polilinea) - 1):
        d = distancia_punto_segmento(P, polilinea[i], polilinea[i + 1])
        if d < mejor:
            mejor = d
    return mejor


def calcular_errores(historial, referencia):
    #    historial: lista/array de puntos (x,y,z) realmente realizados por la pata
    #referencia: lista/array de puntos que definen la trayectoria planeada curva de Bezier
    #Devuelve (errores por punto, RMSE)
    
    hist = np.array(historial)
    ref = np.array(referencia)
    errores = np.array([distancia_a_polilinea(P, ref) for P in hist])
    rmse = float(np.sqrt(np.mean(errores ** 2))) if len(errores) else 0.0
    return errores, rmse


def calcular_velocidad_aceleracion(historial, dt):
    hist = np.array(historial)
    if len(hist) < 2:
        return np.array([]), np.array([])
    vel = np.diff(hist, axis=0) / dt
    velocidad = np.linalg.norm(vel, axis=1)
    if len(vel) < 2:
        return velocidad, np.array([])
    ace = np.diff(vel, axis=0) / dt
    aceleracion = np.linalg.norm(ace, axis=1)
    return velocidad, aceleracion