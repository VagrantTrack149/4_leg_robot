import numpy as np
import math
import os

# Parámetros
a1 = 0.0528       # Offset de abducción (D1)
D1_Y = 0.0006
D2_Y = -0.00053
D3 = 0.1122       # Longitud del fémur
FEMUR_Y = -0.319729
FEMUR_Z = 0.182338
FOOT_OFFSET = np.array([0.001038, -0.271321, -0.225038])

# Límites articulares
Q1_MIN, Q1_MAX = -0.6, 0.5
Q2_MIN, Q2_MAX = -1.7, 1.7
Q3_MIN, Q3_MAX = -0.45, 1.6

#  Transformaciones Homogéneas 
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def Tx(d):
    T = np.eye(4); T[0, 3] = d; return T

def Ty(d):
    T = np.eye(4); T[1, 3] = d; return T

def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T

#  Cinemática 
def cinematica_directa(q1, q2, q3, left=True):
    #Cinemática Directa. Devuelve 4 puntos: [Base, Hombro, Rodilla, Pie]  en el marco LOCAL de la pata.
    s = -1 if left else 1
    
    # Base de la pata (origen local)
    p_base = np.array([0.0, 0.0, 0.0])
    
    T1 = Tx(s * a1) @ Ty(D1_Y) @ Rz(q1)
    p_hombro = (T1 @ np.array([0, 0, 0, 1]))[:3]
    
    T2 = T1 @ Ty(D2_Y) @ Rx(q2)
    T_rodilla = T2 @ Tx(s * D3) @ Ty(FEMUR_Y) @ Tz(FEMUR_Z)
    p_rodilla = (T_rodilla @ np.array([0, 0, 0, 1]))[:3]
    
    T3 = T_rodilla @ Rx(q3)
    p_forearm = np.array([s * FOOT_OFFSET[0], FOOT_OFFSET[1], FOOT_OFFSET[2], 1.0])
    p_pie = (T3 @ p_forearm)[:3]
    
    return [p_base, p_hombro, p_rodilla, p_pie]

def cinematica_inversa(x, y, z, left=True, q_init=None, max_iter=150, tol=1e-4):
    #Cinemática Inversa numérica (Newton-Raphson).  Devuelve [q1, q2, q3] o None si no converge.
    target_pos = np.array([x, y, z], dtype=float)
    if q_init is None:
        q_init = np.array([0.0, 0.0, 0.0])
    
    q = np.array(q_init, dtype=float)
    
    for _ in range(max_iter):
        pts = cinematica_directa(q[0], q[1], q[2], left)
        p = pts[3] # Posición del pie
        error = target_pos - p
        
        if np.linalg.norm(error) < tol:
            # Verificar límites antes de devolver
            if (Q1_MIN <= q[0] <= Q1_MAX and 
                Q2_MIN <= q[1] <= Q2_MAX and 
                Q3_MIN <= q[2] <= Q3_MAX):
                return q
            else:
                return None # Convergió pero fuera de límites
                
        # Jacobiano por diferencias finitas
        J = np.zeros((3, 3))
        delta = 1e-6
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += delta
            pts_plus = cinematica_directa(q_plus[0], q_plus[1], q_plus[2], left)
            J[:, i] = (pts_plus[3] - p) / delta
            
        try:
            dq = np.linalg.solve(J, error)
        except np.linalg.LinAlgError:
            dq = np.linalg.pinv(J) @ error
            
        q += dq
        # Clip durante la iteración para ayudar a la convergencia
        q[0] = np.clip(q[0], Q1_MIN, Q1_MAX)
        q[1] = np.clip(q[1], Q2_MIN, Q2_MAX)
        q[2] = np.clip(q[2], Q3_MIN, Q3_MAX)
        
    return None # No convergió

def punto_alcanzable(pos, left=True):
    return cinematica_inversa(pos[0], pos[1], pos[2], left) is not None

#  Procesamiento de Trayectorias 
def cargar_ruta_resultante_npz(path, n_puntos=60):
    if not os.path.exists(path):
        raise FileNotFoundError(f"No se encontró el archivo: {path}")
    
    data = np.load(path)
    # Intentar obtener el array
    key = list(data.keys())[0]
    ruta = data[key]
    
    # Si hay más puntos de los necesarios, submuestrear
    if len(ruta) > n_puntos:
        indices = np.linspace(0, len(ruta) - 1, n_puntos).astype(int)
        ruta = ruta[indices]
    return ruta

def reorganizar_trayectoria(trayectoria, start_pos):
    #Reordena la trayectoria para empezar en el punto más cercano a start_pos.
    if len(trayectoria) == 0:
        return trayectoria
    dists = [np.linalg.norm(p - start_pos) for p in trayectoria]
    idx_min = np.argmin(dists)
    return np.roll(trayectoria, -idx_min, axis=0)

def distancia_a_polilinea(punto, polilinea):
    min_dist = float('inf')
    for i in range(len(polilinea) - 1):
        p1, p2 = polilinea[i], polilinea[i+1]
        # Proyección del punto sobre el segmento
        v = p2 - p1
        w = punto - p1
        c1 = np.dot(w, v)
        if c1 <= 0:
            dist = np.linalg.norm(punto - p1)
        else:
            c2 = np.dot(v, v)
            if c2 <= c1:
                dist = np.linalg.norm(punto - p2)
            else:
                b = c1 / c2
                pb = p1 + b * v
                dist = np.linalg.norm(punto - pb)
        if dist < min_dist:
            min_dist = dist
    return min_dist

def calcular_errores(actual, referencia):
    #Calcula el error perpendicular y el RMSE respecto a la trayectoria de referencia.
    errores = []
    ref_array = np.array(referencia)
    for p in actual:
        errores.append(distancia_a_polilinea(p, ref_array))
    errores = np.array(errores)
    rmse = np.sqrt(np.mean(errores**2))
    return errores, rmse

def calcular_velocidad_aceleracion(posiciones, dt=0.02):
    pos = np.array(posiciones)
    if len(pos) < 3:
        return np.zeros(len(pos)), np.zeros(len(pos))
    
    velocidad = np.diff(pos, axis=0) / dt
    aceleracion = np.diff(velocidad, axis=0) / dt
    
    # Magnitudes
    v_mag = np.linalg.norm(velocidad, axis=1)
    a_mag = np.linalg.norm(aceleracion, axis=1)
    
    # Ajustar tamaños para que coincidan con el eje de tiempo original
    v_mag = np.append(v_mag, v_mag[-1])
    a_mag = np.append(a_mag, [a_mag[-1], a_mag[-1]])
    
    return v_mag, a_mag