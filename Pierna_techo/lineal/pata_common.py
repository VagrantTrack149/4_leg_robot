import numpy as np
from scipy.interpolate import CubicSpline

# Parámetros físicos de la pata (idénticos a los usados en supervisor.py)

a1 = 0.0528
L2 = 0.2142
L3 = 0.2142

Q1_MIN, Q1_MAX = -0.6, 0.6     # abducción / aducción
Q2_MIN, Q2_MAX = -1.6, 1.6     # hombro (rotación / "hip pitch")
Q3_MIN, Q3_MAX = 0.05, 3.0     # codo / rodilla (0=recta, pi=totalmente flexionada)

_EPS = 1e-9



# Cinemática

def cinematica_directa(q1, q2, q3, lado):

    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)

    O0 = np.array([0.0, 0.0, 0.0])
    O1 = O0 + np.array([0.0, -lado * a1 * c1, -a1 * s1])
    O2 = O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3 = O2 + np.array([L3 * s23, 0.0, -L3 * c23])

    return O0, O1, O2, O3


def cinematica_inversa(px, py, pz, lado):
    #  q1 (abducción), a partir de la componente py 
    # (tolerancia de 1e-3, no 1e-6: evita rechazar puntos válidos por ruido
    # numérico de punto flotante cuando py está casi exactamente en el borde)
    ratio = -py * lado / a1
    if abs(ratio) > 1.0 + 1e-3:
        return None
    ratio = np.clip(ratio, -1.0, 1.0)
    q1 = np.arccos(ratio)

    #  q2, q3 (2 eslabones planos en X-Z) 
    O1_z = -a1 * np.sin(q1)
    dx = px
    dz = pz - O1_z

    R = np.hypot(dx, dz)
    if R > (L2 + L3) or R < abs(L2 - L3):
        return None

    C3 = (R ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    if abs(C3) > 1.0 + 1e-6:
        return None
    C3 = np.clip(C3, -1.0, 1.0)

    q3 = np.arctan2(np.sqrt(max(0.0, 1.0 - C3 ** 2)), C3)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)
    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    #  Verificación de límites articulares 
    if not (Q1_MIN <= q1 <= Q1_MAX):
        return None
    if not (Q2_MIN <= q2 <= Q2_MAX):
        return None
    if not (Q3_MIN <= q3 <= Q3_MAX):
        return None

    return q1, q2, q3


def punto_alcanzable(pos_local, lado):
    return cinematica_inversa(pos_local[0], pos_local[1], pos_local[2], lado) is not None


def generar_espiral(centro, R0, Rf, vueltas, n_puntos, z_ini, z_fin):
    cx, cy, _cz = centro
    t = np.linspace(0.0, 1.0, n_puntos)
    theta = 2 * np.pi * vueltas * t
    radio = R0 + (Rf - R0) * t
    z = z_ini + (z_fin - z_ini) * t
    x = cx + radio * np.cos(theta)
    return [np.array([xi, cy, zi]) for xi, zi in zip(x, z)]


def generar_lissajous(centro, radio, frecuencias, n_puntos):
    cx, cy, cz = centro
    fx, fy, fz = frecuencias
    t = np.linspace(0.0, 2 * np.pi, n_puntos)
    x = cx + radio * np.sin(fx * t)
    y = cy + radio * np.sin(fy * t)
    z = cz + radio * np.sin(fz * t)
    return [np.array([xi, yi, zi]) for xi, yi, zi in zip(x, y, z)]


def generar_escalon(centro=(0.0, -a1, -0.30), lado_paso=0.05, altura_paso=0.05):
    cx, cy, cz = centro
    return [
        np.array([cx - lado_paso, cy, cz]),
        np.array([cx, cy, cz + altura_paso]),
        np.array([cx + lado_paso, cy, cz + altura_paso]),
    ]



# Carga de la RUTA FINAL/RESULTANTE desde el .npz del optimizador

def cargar_ruta_resultante_npz(path_npz, n_puntos=200):

    datos = np.load(path_npz, allow_pickle=True)
    vertices = datos['vertices_opt']
    parametro = datos['param_vertices']

    spline = CubicSpline(parametro, vertices, axis=0, bc_type='periodic')
    t_muestreo = np.linspace(0.0, 1.0, n_puntos, endpoint=False)
    ruta = spline(t_muestreo)
    return ruta


# Reordenamiento de trayectoria (empezar por el punto más cercano al actual)
def reorganizar_trayectoria(trayectoria, punto_inicial):
    pts = np.array(trayectoria)
    dists = np.linalg.norm(pts - np.array(punto_inicial), axis=1)
    idx0 = int(np.argmin(dists))
    reordenada = np.concatenate([pts[idx0:], pts[:idx0]], axis=0)
    return [p for p in reordenada]


# Errores de seguimiento y cinemática de la trayectoria real (para gráficas)

def _dist_punto_segmento(p, a, b):
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < _EPS:
        return np.linalg.norm(p - a)
    t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
    proyeccion = a + t * ab
    return np.linalg.norm(p - proyeccion)


def distancia_a_polilinea(punto, polilinea, cerrada=True):
    pts = np.asarray(polilinea)
    p = np.asarray(punto)
    n = len(pts)
    if n == 0:
        return 0.0
    if n == 1:
        return float(np.linalg.norm(p - pts[0]))
    segmentos = n if cerrada else n - 1
    dmin = np.inf
    for i in range(segmentos):
        a = pts[i]
        b = pts[(i + 1) % n]
        d = _dist_punto_segmento(p, a, b)
        if d < dmin:
            dmin = d
    return float(dmin)


def calcular_errores(historial_actual, referencia, cerrada=True):
    if len(historial_actual) == 0:
        return np.array([]), 0.0
    errores = np.array([distancia_a_polilinea(p, referencia, cerrada) for p in historial_actual])
    rmse = float(np.sqrt(np.mean(errores ** 2)))
    return errores, rmse


def calcular_velocidad_aceleracion(historial, dt):
    pts = np.asarray(historial)
    if len(pts) < 2:
        return np.array([]), np.array([])
    vel_vec = np.diff(pts, axis=0) / dt
    velocidad = np.linalg.norm(vel_vec, axis=1)
    if len(velocidad) < 2:
        return velocidad, np.array([])
    acel_vec = np.diff(vel_vec, axis=0) / dt
    aceleracion = np.linalg.norm(acel_vec, axis=1)
    return velocidad, aceleracion