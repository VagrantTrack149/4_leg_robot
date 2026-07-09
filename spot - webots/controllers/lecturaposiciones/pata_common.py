# CODIGO AUXILIAR PARA FUNCIONES DE CINEMATICA, TRAZADO DE CURVAS Y METRICAS DE ERROR
# Referencia: https://harunkurtdev.github.io/webots-simulink/examples/spot-quadruped/

import math
import numpy as np


# Parámetros geométricos del Spot
L_hip = 0.08           # Cadera offset (abducción) [m]
L_thigh = 0.34         # Thigh length (fémur) [m]
L_calf = 0.35          # Calf length (tibia) [m]

# Límites reales de los motores (rad)
Q1_MIN, Q1_MAX = -0.6, 0.5      # Cadera abduction
Q2_MIN, Q2_MAX = -1.7, 1.7      # Cadera flexion
Q3_MIN, Q3_MAX = -0.45, 1.6     # codo flexion


# Conversión ángulos internos -> ángulos de motor
#esta función es innecesaria xd 
#tengo que quitarla luego, ahora ya funciona mejorcito
def interno_a_motor(q1, q2, q3):
    return q1, q2, q3

# CINEMÁTICA DIRECTA
#usada para la animación de los wireframe pero no para webots

def cinematica_directa(q1, q2, q3, lado=1):
    """
    q1 : cadera abduction angulo [rad]
    q2 : cadera flexion angulo [rad]
    q3 : knee flexion angulo [rad]
    lado : 1 para pata izquierda (FL, RL), -1 para derecha (FR, RR)
    Returns: O0 (cadera), O1 (cadera actual), O2 (knee), O3 (foot)
    """
    # Hip position (abduction offset)
    c1 = math.cos(q1)
    s1 = math.sin(q1)
    
    O0 = np.array([0.0, 0.0, 0.0])  # Cadera frame origin
    O1 = np.array([0.0, lado * L_hip * c1, -L_hip * s1])  # Cadera joint position
    
    # Thigh segment (from cadera flexion angulo)
    c2 = math.cos(q2)
    s2 = math.sin(q2)
    
    O2 = O1 + np.array([L_thigh * s2, 0.0, -L_thigh * c2])
    
    # Calf segment (from codo flexion angulo)
    # q2 + q3 is the total angulo from vertical for the calf
    c23 = math.cos(q2 + q3)
    s23 = math.sin(q2 + q3)
    
    O3 = O2 + np.array([L_calf * s23, 0.0, -L_calf * c23])
    
    return O0, O1, O2, O3



# CINEMÁTICA INVERSA - Modelo Spot

def cinematica_inversa(px, py, pz, lado):
    """
    px, py, pz : posicion del pie en el marco de trabajo de la cadera [m]
    lado : 1 para pata izquierda (FL, RL), -1 para derecha (FR, RR)
    Returns: (q1, q2, q3) in radians, or None if unreachable
    """
    
    #  Cadera abduction angulo (q1)
    # Position relative to cadera: subtract cadera offset
    y_rel = py - lado * L_hip
    z_rel = pz
    
    # Cadera abduction: q1 = atan2(y, -z)
    q1 = math.atan2(y_rel, -z_rel)
    
    # distancia del plano sagital (xz) al pie
    # After cadera rotation, position in the sagittal plane
    L = math.sqrt(y_rel**2 + z_rel**2)
    d = math.sqrt(px**2 + L**2)
    
    #  Codo angulo (q3) usando ley de cosenos
    # d² = L_thigh² + L_calf² - 2*L_thigh*L_calf*cos(π - q3)
    # cos(π - q3) = -cos(q3)
    cos_knee = (L_thigh**2 + L_calf**2 - d**2) / (2.0 * L_thigh * L_calf)
    cos_knee = max(-1.0, min(1.0, cos_knee))  # Clamp to [-1, 1]
    
    # q3 = π - arccos(cos_knee)
    q3 = math.pi - math.acos(cos_knee)
    
    #  Cadera flexion angulo (q2)
    # 2 componentes: alpha (direccion pie) and beta (influencia del codo)
    alpha = math.atan2(px, L)
    
    # usando ley de cosenos for the angulo at thigh:
    cos_thigh = (L_thigh**2 + d**2 - L_calf**2) / (2.0 * L_thigh * d)
    cos_thigh = max(-1.0, min(1.0, cos_thigh))
    beta = math.acos(cos_thigh)
    
    q2 = alpha + beta
    
    # Verificar limites
    q1_m, q2_m, q3_m = interno_a_motor(q1, q2, q3)
    if not (Q1_MIN <= q1_m <= Q1_MAX and
            Q2_MIN <= q2_m <= Q2_MAX and
            Q3_MIN <= q3_m <= Q3_MAX):
        return None
    
    return q1, q2, q3


def punto_alcanzable(p, lado):
    #Verifica si un punto en el espacio Cartesiano es alcanzable por la pata
    return cinematica_inversa(p[0], p[1], p[2], lado) is not None



# GENERADORES DE TRAYECTORIAS

def generar_espiral(centro, R0, Rf, vueltas, n_puntos, z_ini, z_fin):
    puntos = []
    for i in range(n_puntos):
        t = i / (n_puntos - 1) if n_puntos > 1 else 0
        radio = R0 - (R0 - Rf) * t
        angulo = 2 * np.pi * vueltas * t
        x = centro[0] + radio * np.cos(angulo)
        y = centro[1] + radio * np.sin(angulo)
        z = z_ini + (z_fin - z_ini) * t
        puntos.append(np.array([x, y, z]))
    return puntos

def generar_lissajous(centro=(0.0, 0.0, -0.4), radio=0.05, frecuencias=(2, 3, 5), n_puntos=150):
    puntos = []
    for t in np.linspace(0, 2*np.pi, n_puntos, endpoint=False):
        x = centro[0] + radio * np.cos(frecuencias[0] * t)
        y = centro[1] + radio * np.sin(frecuencias[1] * t)
        z = centro[2] + radio * np.sin(frecuencias[2] * t)
        puntos.append(np.array([x, y, z]))
    return puntos

def generar_escalon():
    puntos = []
    puntos.append(np.array([0.15, 0.0, -0.55]))
    puntos.append(np.array([-0.15, 0.0, -0.45]))
    return puntos



# CURVAS DE BÉZIER

def bezier_cubico(p0, p1, p2, p3, t):
    u = 1.0 - t
    return u**3 * p0 + 3*u**2*t * p1 + 3*u*t**2 * p2 + t**3 * p3

def calcular_tangentes(puntos):
    n = len(puntos)
    tangentes = []
    for i in range(n):
        if i == 0:
            tang = puntos[1] - puntos[0]
        elif i == n - 1:
            tang = puntos[-1] - puntos[-2]
        else:
            tang = (puntos[i+1] - puntos[i-1]) / 2.0
        if i < n - 1:
            dist = np.linalg.norm(puntos[i+1] - puntos[i])
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
        p1 = p0 + (1.0/3.0) * tang_in * factor
        p2 = p3 - (1.0/3.0) * tang_out * factor
        if segmento_bezier_valido(p0, p1, p2, p3, lado):
            return (p0, p1, p2, p3)
    p1 = p0 + (1.0/3.0) * tang_in * 0.0
    p2 = p3 - (1.0/3.0) * tang_out * 0.0
    return (p0, p1, p2, p3)

def preparar_trayectoria_completa(puntos, lado):
    tang = calcular_tangentes(puntos)
    segmentos = []
    for i in range(len(puntos)-1):
        seg = preparar_segmento(puntos[i], puntos[i+1], tang[i], tang[i+1], lado)
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



# MÉTRICAS DE ERROR
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
    for i in range(len(polilinea)-1):
        d = distancia_punto_segmento(P, polilinea[i], polilinea[i+1])
        if d < mejor:
            mejor = d
    return mejor

def calcular_errores(historial, referencia):
    hist = np.array(historial)
    ref = np.array(referencia)
    errores = np.array([distancia_a_polilinea(P, ref) for P in hist])
    rmse = float(np.sqrt(np.mean(errores**2))) if len(errores) else 0.0
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