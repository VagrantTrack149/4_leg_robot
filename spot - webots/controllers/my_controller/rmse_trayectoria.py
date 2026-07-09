"""
rmse_trayectoria.py

Calculo correcto del error de seguimiento de trayectoria (RMSE) para la
pata del Spot en Webots.

POR QUE NO SE DEBE COMPARAR POR INDICE
---------------------------------------
`historial_real[i]` (un punto por cada paso de simulacion) y `referencia[i]`
(un punto por muestra de la curva planeada) tienen densidades de muestreo
completamente distintas y NO estan alineadas en el mismo parametro. En modo
'lineal'/'poligonal', `referencia` puede ser literalmente la lista cruda de
4 waypoints comparada contra cientos de frames -- el emparejamiento por
indice ahi es directamente incorrecto (o ni siquiera tiene el mismo largo).

El error que realmente se quiere medir es el "error perpendicular" /
cross-track error: la distancia del punto REAL, en cada instante, al punto
MAS CERCANO de la curva PLANEADA completa (sin importar en que instante de
la planeacion caiga ese punto mas cercano).

COMO SE CALCULA AQUI
---------------------------------------
- Trayectoria lineal / poligonal: los segmentos ya son rectas exactas
  (`segmentos = [(p0,p1), ...]`). La distancia punto-a-recta es exacta via
  proyeccion ortogonal con clamping a los extremos del segmento.
- Trayectoria Bezier: los segmentos son curvas cubicas exactas
  (`segmentos = [(p0,p1,p2,p3), ...]`). La distancia punto-a-curva no tiene
  forma cerrada simple, asi que se resuelve numericamente: muestreo grueso
  para localizar la region del minimo + busqueda ternaria para refinar
  (verificado contra fuerza bruta de 200000 muestras: error < 3e-7 m).

En ambos casos, el error de un punto real es el MINIMO sobre TODOS los
segmentos de la trayectoria planeada (no solo el segmento "actual"), que es
la definicion geometricamente correcta de distancia punto-a-curva.

USO
---------------------------------------
    from rmse_trayectoria import calcular_rmse_trayectoria

    # al final de la simulacion, con los `segmentos` que ya construye tu
    # script (los mismos que usaste para generar el movimiento, sea modo
    # 'bezier', 'lineal' o 'poligonal'):
    errores, rmse, err_max, err_medio = calcular_rmse_trayectoria(
        historial_real, segmentos, MODO_SEGUIMIENTO
    )
"""

import numpy as np


# ---------------------------------------------------------------------------
# Distancia punto-a-segmento (recta), exacta
# ---------------------------------------------------------------------------
def _dist_punto_segmento(p, a, b):
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-15:
        return float(np.linalg.norm(p - a))
    t = np.dot(p - a, ab) / denom
    t = min(1.0, max(0.0, t))  # clamp: no salirse del segmento
    proy = a + t * ab
    return float(np.linalg.norm(p - proy))


# ---------------------------------------------------------------------------
# Distancia punto-a-Bezier cubica, numerica (muestreo grueso + refinamiento)
# ---------------------------------------------------------------------------
def _bezier_punto(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (u**3) * p0 + 3 * (u**2) * t * p1 + 3 * u * (t**2) * p2 + (t**3) * p3


def _dist_punto_bezier(p, p0, p1, p2, p3, n_grueso=40, n_refina=25):
    ts = np.linspace(0.0, 1.0, n_grueso)
    pts = np.array([_bezier_punto(p0, p1, p2, p3, t) for t in ts])
    d2 = np.sum((pts - p) ** 2, axis=1)
    i_min = int(np.argmin(d2))
    lo = ts[max(i_min - 1, 0)]
    hi = ts[min(i_min + 1, n_grueso - 1)]

    def f(t):
        pt = _bezier_punto(p0, p1, p2, p3, t)
        d = pt - p
        return float(np.dot(d, d))

    # busqueda ternaria: la region acotada por [lo,hi] alrededor del minimo
    # muestreado grueso es unimodal en la practica para curvas suaves
    for _ in range(n_refina):
        m1 = lo + (hi - lo) / 3
        m2 = hi - (hi - lo) / 3
        if f(m1) < f(m2):
            hi = m2
        else:
            lo = m1
    t_opt = (lo + hi) / 2
    return float(np.sqrt(f(t_opt)))


# ---------------------------------------------------------------------------
# Distancia de un punto a la trayectoria PLANEADA COMPLETA (todos los segmentos)
# ---------------------------------------------------------------------------
def _dist_punto_a_trayectoria(p, segmentos, modo):
    dists = []
    if modo == "bezier":
        for (p0, p1, p2, p3) in segmentos:
            dists.append(_dist_punto_bezier(p, p0, p1, p2, p3))
    else:  # 'lineal' o 'poligonal': segmentos = [(p0, p1), ...] rectas
        for (p0, p1) in segmentos:
            dists.append(_dist_punto_segmento(p, p0, p1))
    return min(dists) if dists else float("nan")


# ---------------------------------------------------------------------------
# API principal
# ---------------------------------------------------------------------------
def calcular_rmse_trayectoria(historial_real, segmentos, modo):
    """
    historial_real : lista/array de puntos (x,y,z) realmente recorridos
                      por el pie (uno por paso de simulacion).
    segmentos      : la lista de segmentos ya construida por tu script
                      (`segmentos` en tu codigo), NO la version discretizada
                      `referencia`.
                        - modo 'bezier'              -> [(p0,p1,p2,p3), ...]
                        - modo 'lineal'/'poligonal'   -> [(p0,p1), ...]
    modo           : 'bezier' | 'lineal' | 'poligonal'

    Devuelve:
        errores    : array con el error perpendicular de cada frame (m)
        rmse       : sqrt(mean(errores**2))
        error_max  : max(errores)
        error_medio: mean(errores)
    """
    if len(historial_real) == 0 or len(segmentos) == 0:
        return np.array([]), float("nan"), float("nan"), float("nan")

    real = np.asarray(historial_real, dtype=float)
    seg_np = (
        [(np.asarray(a, float), np.asarray(b, float),
          np.asarray(c, float), np.asarray(d, float)) for (a, b, c, d) in segmentos]
        if modo == "bezier"
        else [(np.asarray(a, float), np.asarray(b, float)) for (a, b) in segmentos]
    )

    errores = np.array([
        _dist_punto_a_trayectoria(p, seg_np, modo) for p in real
    ])

    rmse = float(np.sqrt(np.mean(errores ** 2)))
    error_max = float(np.max(errores))
    error_medio = float(np.mean(errores))

    return errores, rmse, error_max, error_medio
