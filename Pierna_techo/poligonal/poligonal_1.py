# -*- coding: utf-8 -*-
"""
poligonal_1.py (refactorizado con pata_common)

Simulador de trayectoria poligonal (aproximación lineal de la espiral)
con evasión de obstáculos (bloque). Usa el módulo pata_common para toda
la cinemática, generación de trayectorias y métricas de error.
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from pata_common import (
    cinematica_directa,
    cinematica_inversa,
    punto_alcanzable,
    generar_espiral,
    generar_lissajous,
    generar_escalon,
    reorganizar_trayectoria,
    calcular_errores,
    calcular_velocidad_aceleracion,
    distancia_a_polilinea,
)


# Parámetros de la pierna (FR)

CADERA_FR = np.array([-0.29785, -0.055, 0.0])   # offset de la cadera en el mundo
LADO_FR = -1                                    # pata delantera derecha


# Trayectoria espiral cónica 

TIPO_TRAYECTORIA = 'espiral'

if TIPO_TRAYECTORIA == 'espiral':
    trayectoria = generar_espiral(
        centro=(0.0, 0.0, -0.3820),
        R0=0.026, Rf=0.0083,
        vueltas=3, n_puntos=30,
        z_ini=-0.360, z_fin=-0.425
    )
elif TIPO_TRAYECTORIA == 'lissajous':
    trayectoria = generar_lissajous(
        centro=(0.0, 0.04, -0.38),
        radio=0.015,
        frecuencias=(2, 3, 5),
        n_puntos=100
    )
elif TIPO_TRAYECTORIA == 'escalon':
    trayectoria = generar_escalon()
else:
    raise ValueError("TIPO_TRAYECTORIA no válido. Usa 'espiral', 'lissajous' o 'escalon'.")


# Aproximación poligonal
def aproximar_poligonal(puntos, n_valor):
    #Aproxima una trayectoria por una poligonal con n_valor vértices.
    pts = np.asarray(puntos, dtype=float)
    if n_valor < 1:
        raise ValueError("n_valor debe ser >= 1")
    if len(pts) == 1 or n_valor == 1:
        return [pts[0].copy()]
    dist_seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    arco = np.concatenate([[0.0], np.cumsum(dist_seg)])
    longitud_total = arco[-1]
    if longitud_total < 1e-12:
        return [pts[0].copy() for _ in range(n_valor)]
    objetivos = np.linspace(0.0, longitud_total, n_valor)
    vertices = []
    for s in objetivos:
        idx = np.searchsorted(arco, s)
        idx = int(np.clip(idx, 1, len(arco)-1))
        s0, s1 = arco[idx-1], arco[idx]
        p0, p1 = pts[idx-1], pts[idx]
        if (s1 - s0) < 1e-12:
            vertices.append(p0.copy())
        else:
            t = (s - s0) / (s1 - s0)
            vertices.append(p0 + t*(p1-p0))
    return vertices


# Bloque (volumen escalón)

ESCALON_X_MIN = -0.70
ESCALON_X_MAX = -0.40
ESCALON_Y_MIN = -0.15
ESCALON_Y_MAX =  0.15
ESCALON_Z_MIN = -0.40
ESCALON_Z_MAX = -0.28

vertices_bloque = np.array([
    [ESCALON_X_MAX,  ESCALON_Y_MAX, ESCALON_Z_MIN],
    [ESCALON_X_MAX,  ESCALON_Y_MIN, ESCALON_Z_MIN],
    [ESCALON_X_MIN,  ESCALON_Y_MIN, ESCALON_Z_MIN],
    [ESCALON_X_MIN,  ESCALON_Y_MAX, ESCALON_Z_MIN],
    [ESCALON_X_MAX,  ESCALON_Y_MAX, ESCALON_Z_MAX],
    [ESCALON_X_MAX,  ESCALON_Y_MIN, ESCALON_Z_MAX],
    [ESCALON_X_MIN,  ESCALON_Y_MIN, ESCALON_Z_MAX],
    [ESCALON_X_MIN,  ESCALON_Y_MAX, ESCALON_Z_MAX],
])
caras = [
    [vertices_bloque[0], vertices_bloque[1], vertices_bloque[2], vertices_bloque[3]],
    [vertices_bloque[4], vertices_bloque[5], vertices_bloque[6], vertices_bloque[7]],
    [vertices_bloque[0], vertices_bloque[1], vertices_bloque[5], vertices_bloque[4]],
    [vertices_bloque[2], vertices_bloque[3], vertices_bloque[7], vertices_bloque[6]],
    [vertices_bloque[0], vertices_bloque[3], vertices_bloque[7], vertices_bloque[4]],
    [vertices_bloque[1], vertices_bloque[2], vertices_bloque[6], vertices_bloque[5]],
]
cubo = Poly3DCollection(caras, alpha=0.35, facecolor='gray', edgecolor='black')

def punto_dentro_bloque(P):
    return (ESCALON_X_MIN <= P[0] <= ESCALON_X_MAX and
            ESCALON_Y_MIN <= P[1] <= ESCALON_Y_MAX and
            ESCALON_Z_MIN <= P[2] <= ESCALON_Z_MAX)

def segmento_colisiona(A, B, muestras=50):
    for t in np.linspace(0, 1, muestras):
        P = A + t * (B - A)
        if punto_dentro_bloque(P):
            return True
    return False


# Segmentos rectos (poligonales) con evasión de colisión

def segmento_recto_colisiona(p0, p3, muestras=40):
    #Verifica si el segmento recto entre p0 y p3 (coordenadas locales) colisiona con el bloque.
    for u in np.linspace(0.0, 1.0, muestras):
        punto_local = p0 + u*(p3-p0)
        if punto_dentro_bloque(CADERA_FR + punto_local):
            return True
    return False

def segmento_recto_valido(p0, p3, lado, muestras=20):
    #Verifica que todos los puntos del segmento recto sean alcanzables.
    for u in np.linspace(0.0, 1.0, muestras):
        punto = p0 + u*(p3-p0)
        if not punto_alcanzable(punto, lado):
            return False
    return True

def preparar_segmento_poligonal(p0, p3, lado):
    #Genera una poligonal de 1 o 2 segmentos rectos que evita el bloque.
    elevacion_max = 0.20
    paso = 0.02
    elevacion = 0.0
    while True:
        if elevacion == 0.0:
            cadena = [p0, p3]
        else:
            pm = (p0 + p3) / 2.0
            pm = pm.copy()
            pm[2] += elevacion
            cadena = [p0, pm, p3]
        choca = any(segmento_recto_colisiona(cadena[i], cadena[i+1]) for i in range(len(cadena)-1))
        if not choca:
            valido = all(segmento_recto_valido(cadena[i], cadena[i+1], lado) for i in range(len(cadena)-1))
            if valido:
                return cadena, False
        elevacion += paso
        if elevacion > elevacion_max:
            return [p0, p3], True

def preparar_trayectoria_completa_poligonal(puntos, lado):
    #Construye la poligonal completa con evasión de obstáculos.
    puntos_validos = [p for p in puntos if punto_alcanzable(p, lado)]
    if len(puntos_validos) < 2:
        return [], True
    vertices_totales = [puntos_validos[0]]
    bloqueado_global = False
    for i in range(len(puntos_validos)-1):
        p0 = puntos_validos[i]
        p3 = puntos_validos[i+1]
        cadena, bloqueado = preparar_segmento_poligonal(p0, p3, lado)
        vertices_totales.extend(cadena[1:])
        if bloqueado:
            bloqueado_global = True
            break
    segmentos = [(vertices_totales[i], vertices_totales[i+1]) for i in range(len(vertices_totales)-1)]
    return segmentos, vertices_totales, bloqueado_global


# Estado y registro

estado = {
    'actual': np.array([0.0, -0.0528, -0.30]),   # posición local inicial
    't_segmento': 0.0,
    'indice_segmento': 0,
    'siguiendo': False,
    'terminado': False,
    'segmentos': [],
    'vertices': [],
    'segmento_bloqueado': False,
    'tolerancia': 0.005
}
registro = {'historial': [], 'segmentos_reales': []}

def puntos_pata(pos_local):
    #Devuelve las posiciones de las articulaciones en coordenadas del mundo.
    q = cinematica_inversa(pos_local[0], pos_local[1], pos_local[2], LADO_FR)
    if q is None:
        q = (np.pi/2, 0.0, 0.0)   # pose por defecto
    pts = cinematica_directa(*q, LADO_FR)
    return [CADERA_FR + p for p in pts]

def iniciar_trayectoria():
    if len(poligonal) < 2:
        return
    poli_ordenada = reorganizar_trayectoria(poligonal, estado['actual'])
    registro['historial'].clear()
    registro['segmentos_reales'].clear()
    segs, verts, bloqueado = preparar_trayectoria_completa_poligonal(poli_ordenada, LADO_FR)
    estado['segmentos'] = segs
    estado['vertices'] = verts
    estado['actual'] = poli_ordenada[0].copy()
    estado['indice_segmento'] = 0
    estado['t_segmento'] = 0.0
    estado['terminado'] = False
    estado['segmento_bloqueado'] = bloqueado
    if bloqueado:
        estado['siguiendo'] = False
        print(f"Ruta no iniciada, se preparó hasta el segmento {len(segs)} (colisión sin solución o ángulos fuera de rango).")
    else:
        estado['siguiendo'] = True
        print(f"Poligonal preparada con {len(segs)} segmentos rectos (n={n} vértices).")

def avanzar_al_siguiente():
    if estado['indice_segmento'] >= len(estado['segmentos']) - 1:
        estado['siguiendo'] = False
        estado['terminado'] = True
        mostrar_reporte_error()
    else:
        estado['indice_segmento'] += 1
        estado['t_segmento'] = 0.0

def mostrar_reporte_error():
    if len(registro['historial']) == 0:
        return
    #  calcular errores, velocidad y aceleración
    errores, rmse = calcular_errores(registro['historial'], trayectoria)
    velocidad, aceleracion = calcular_velocidad_aceleracion(registro['historial'], dt=0.02)

    print(f"\nRMSE (distancia a la trayectoria original, n={n}): {rmse:.5f} m\n")

    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(estado['tolerancia'], color='gray', linestyle='--',
                   label=f"tolerancia ({estado['tolerancia']} m)")
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':',
                   label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame")
    axs[0].set_ylabel("Error perpendicular (m)")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)

    tiempo_v = np.arange(len(velocidad)) * 0.02
    tiempo_a = np.arange(len(aceleracion)) * 0.02
    axs[1].plot(tiempo_v, velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Tiempo (s)")
    axs[1].set_ylabel("Magnitud (m/s, m/s²)")
    axs[1].set_title(f"Velocidad y aceleración de la trayectoria poligonal (n={n})")
    axs[1].legend()
    axs[1].grid(alpha=0.3)
    fig2.tight_layout()
    fig2.show()


# RMSE vs n (selección automática de n)

def rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_valor, muestras_por_segmento=25):
    ref = np.asarray(puntos_trayectoria, dtype=float)
    verts = np.array(aproximar_poligonal(ref, n_valor))
    errores = []
    for i in range(len(verts)-1):
        p0, p1 = verts[i], verts[i+1]
        for u in np.linspace(0.0, 1.0, muestras_por_segmento, endpoint=False):
            punto = p0 + u*(p1-p0)
            errores.append(distancia_a_polilinea(punto, ref))
    errores.append(distancia_a_polilinea(verts[-1], ref))
    return np.sqrt(np.mean(np.array(errores)**2))

RMSE_OBJETIVO = 0.005
N_AUTOMATICO = True

def encontrar_n_automatico(puntos_trayectoria, rmse_objetivo, n_min=2, n_max=None):
    if n_max is None:
        n_max = len(puntos_trayectoria)
    for n_candidato in range(n_min, n_max + 1):
        rmse_actual = rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_candidato)
        if n_max >= 5:
            if rmse_actual <= rmse_objetivo and n_candidato > 4:
                return n_candidato, rmse_actual
        else:
            if rmse_actual <= rmse_objetivo:
                return n_candidato, rmse_actual
    return n_max, rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_max)

# Si N_AUTOMATICO es True, se calcula n automáticamente
if N_AUTOMATICO:
    n, rmse_logrado = encontrar_n_automatico(trayectoria, RMSE_OBJETIVO)
    poligonal = aproximar_poligonal(trayectoria, n)
    cumplido = "sí" if rmse_logrado <= RMSE_OBJETIVO else "no (se llegó al máximo n posible)"
    print(f"n automático = {n}  (RMSE = {rmse_logrado:.5f} m, objetivo <= {RMSE_OBJETIVO} m, cumplido: {cumplido})")
else:
    n = 10   # valor por defecto si no es automático
    poligonal = aproximar_poligonal(trayectoria, n)

def graficar_rmse_vs_n(puntos_trayectoria, valores_n, n_usado):
    rmses = [rmse_poligonal_vs_trayectoria(puntos_trayectoria, nv) for nv in valores_n]
    fig3, ax3 = plt.subplots(figsize=(7,5))
    ax3.plot(valores_n, rmses, 'o-', color='darkviolet')
    ax3.axvline(n_usado, color='gray', linestyle='--', alpha=0.6, label=f"n usado en la simulación = {n_usado}")
    ax3.set_xlabel("n (número de vértices/segmentos de la poligonal)")
    ax3.set_ylabel("RMSE respecto a la trayectoria original [m]")
    ax3.set_title("Convergencia del RMSE de la poligonal al aumentar n")
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.3)
    fig3.tight_layout()
    fig3.show()
    return rmses

valores_n_prueba = list(range(2, len(trayectoria)+1))
rmses_por_n = graficar_rmse_vs_n(trayectoria, valores_n_prueba, n)


# Figura y animación

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title(f"Trayectoria original ({len(trayectoria)} pts) aproximada con poligonal n={n}")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
limite = 0.3
ax.set_xlim(-limite, limite)
ax.set_ylim(-limite, limite)
ax.set_zlim(-0.50, -0.20)
ax.set_box_aspect([1,1,1])
ax.view_init(elev=30, azim=45)
ax.add_collection3d(cubo)

# Mostrar trayectoria original (waypoints)
tray_world = np.array([CADERA_FR + p for p in trayectoria])
ax.plot(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
        '--', color='dodgerblue', linewidth=1.2, label='Waypoints')
ax.scatter(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2], color='blue', s=30)

linea_seguida, = ax.plot([], [], [], '-', color='crimson', linewidth=1.3, alpha=0.85, label='Real')
curva_poligonal, = ax.plot([], [], [], '-', color='lime', linewidth=2, label='Poligonal')

# Pierna inicial
O0, O1, O2, O3 = puntos_pata(poligonal[0])
l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='red', lw=4)
l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='red', lw=4)
l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='red', lw=4)
articulaciones = ax.scatter([O0[0], O1[0], O2[0], O3[0]],
                            [O0[1], O1[1], O2[1], O3[1]],
                            [O0[2], O1[2], O2[2], O3[2]],
                            color='red', s=40)
pie, = ax.plot([O3[0]], [O3[1]], [O3[2]], 'o', color='gold', markersize=10, markeredgecolor='black')
objetivo_plot, = ax.plot([CADERA_FR[0]], [CADERA_FR[1]], [CADERA_FR[2]],
                         'x', color='green', markersize=12, markeredgewidth=3)

ax.legend(loc='upper left', fontsize=8)
texto_estado = fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')
ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])
boton = Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())

def actualizar(frame):
    dt = 0.02
    if estado['siguiendo']:
        estado['t_segmento'] += dt
        s = np.clip(estado['t_segmento'] / 0.50, 0.0, 1.0)
        u = 3*s**2 - 2*s**3          # easing suave
        idx = estado['indice_segmento']
        p0, p1 = estado['segmentos'][idx]
        estado['actual'] = p0 + u*(p1-p0)
        registro['historial'].append(estado['actual'].copy())
        if estado['t_segmento'] >= 0.50:
            registro['segmentos_reales'].append((p0.copy(), p1.copy()))
            avanzar_al_siguiente()

    O0, O1, O2, O3 = puntos_pata(estado['actual'])

    # Detectar colisiones de los segmentos de la pierna con el bloque
    if segmento_colisiona(O1, O2) or segmento_colisiona(O2, O3):
        l12.set_color('orange')
        l23.set_color('orange')
    else:
        l12.set_color('red')
        l23.set_color('red')

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    articulaciones._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                                 [O0[1], O1[1], O2[1], O3[1]],
                                 [O0[2], O1[2], O2[2], O3[2]])
    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    if len(registro['historial']) > 1:
        hist = np.array(registro['historial'])
        hist_world = CADERA_FR + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    if estado['siguiendo'] or estado['terminado']:
        if len(estado['vertices']) > 0:
            verts_world = np.array([CADERA_FR + v for v in estado['vertices']])
            curva_poligonal.set_data_3d(verts_world[:, 0], verts_world[:, 1], verts_world[:, 2])
        curva_poligonal.set_color('red' if estado['segmento_bloqueado'] else 'lime')
    else:
        curva_poligonal.set_data_3d([], [], [])

    if estado['siguiendo'] or estado['terminado']:
        if estado['siguiendo']:
            estado_txt = 'en ruta'
        elif estado['segmento_bloqueado']:
            estado_txt = 'detenido (bloqueado)'
        else:
            estado_txt = 'completado'
        texto_estado.set_text(
            f"Segmento: {estado['indice_segmento']+1}/{len(estado['segmentos'])}\n"
            f"Puntos: {len(registro['historial'])}\n"
            f"Estado: {estado_txt}"
        )
    return []

ani = FuncAnimation(fig, actualizar, interval=20, blit=False)
plt.tight_layout()
plt.show()