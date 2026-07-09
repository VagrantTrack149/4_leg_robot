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
    bezier_cubico,
    preparar_trayectoria_completa,
    reorganizar_trayectoria,
    calcular_errores,
    calcular_velocidad_aceleracion,
    generar_bezier_completa,
)

# -------------------------------------------------------------------
# Configuración de la trayectoria
# -------------------------------------------------------------------
TIPO_TRAYECTORIA = 'escalon'   # Opciones: 'espiral', 'lissajous', 'escalon'

if TIPO_TRAYECTORIA == 'espiral':
    trayectoria = generar_espiral(
        centro=(0.0, 0.0, -0.3820),
        R0=0.026, Rf=0.008,
        vueltas=3, n_puntos=60,
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
    raise ValueError("TIPO_TRAYECTORIA no válido.")

# -------------------------------------------------------------------
# Parámetros de la pata (FL)
# -------------------------------------------------------------------
LADO = 1                       # pata delantera izquierda
CADERA = np.array([0.0, 0.0, 0.0])
DT = 0.02                      # paso de tiempo (s)
DURACION_SEGMENTO = 0.3        # duración de cada segmento Bézier (s)

# -------------------------------------------------------------------
# Filtrado de puntos alcanzables
# -------------------------------------------------------------------
def reporte_alcanzabilidad(puntos, lado):
    total = len(puntos)
    alcanzables = 0
    for i, p in enumerate(puntos):
        if punto_alcanzable(p, lado):
            alcanzables += 1
        else:
            print(f"  Punto {i} fuera de rango: {p}")
    print(f"Alcanzabilidad: {alcanzables}/{total} puntos dentro de los límites reales del motor")
    return alcanzables

alcanzables = reporte_alcanzabilidad(trayectoria, LADO)
if alcanzables < 2:
    raise RuntimeError("No hay suficientes puntos alcanzables en la trayectoria.")

trayectoria_valida = [p for p in trayectoria if punto_alcanzable(p, LADO)]

# -------------------------------------------------------------------
# Estado y registro
# -------------------------------------------------------------------
estado = {
    'actual': trayectoria_valida[0].copy(),
    't_segmento': 0.0,
    'indice_segmento': 0,
    'siguiendo': False,
    'terminado': False,
    'segmentos': [],
}
registro = {'historial': []}

def puntos_pata(pos):
    """Devuelve las coordenadas de las articulaciones para una posición del pie."""
    q = cinematica_inversa(pos[0], pos[1], pos[2], LADO)
    if q is None:
        # Si momentáneamente no es alcanzable, se mantiene la última pose válida
        q = (0.0, 0.0, 0.0)  # postura neutra
    pts = cinematica_directa(*q, LADO)
    return [CADERA + p for p in pts]

def iniciar_trayectoria():
    if len(trayectoria_valida) < 2:
        print("No hay suficientes puntos alcanzables para iniciar la ruta.")
        return

    # Reordenar para empezar por el extremo más cercano a la posición actual
    tray_ordenada = reorganizar_trayectoria(trayectoria_valida, estado['actual'])

    registro['historial'].clear()
    segmentos = preparar_trayectoria_completa(tray_ordenada, LADO)

    estado['segmentos'] = segmentos
    estado['actual'] = tray_ordenada[0].copy()
    estado['indice_segmento'] = 0
    estado['t_segmento'] = 0.0
    estado['terminado'] = False
    estado['siguiendo'] = True
    print(f"Trayectoria preparada con {len(segmentos)} segmentos")

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

    # Curva Bézier planeada muestreada densamente → referencia para el error
    bezier_planeada = generar_bezier_completa(estado['segmentos'], puntos_por_segmento=60)

    errores, rmse = calcular_errores(registro['historial'], bezier_planeada)
    velocidad, aceleracion = calcular_velocidad_aceleracion(registro['historial'], DT)

    print(f"\nRMSE (distancia real vs. Bézier planeada): {rmse:.6f} m\n")

    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':', label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame")
    axs[0].set_ylabel("Error perpendicular (m)")
    axs[0].set_title("Error de seguimiento de trayectoria")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)

    tiempo_v = np.arange(len(velocidad)) * DT
    tiempo_a = np.arange(len(aceleracion)) * DT
    axs[1].plot(tiempo_v, velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Tiempo (s)")
    axs[1].set_ylabel("Magnitud")
    axs[1].set_title("Velocidad y aceleración de la trayectoria")
    axs[1].legend()
    axs[1].grid(alpha=0.3)
    fig2.tight_layout()
    plt.show()

# -------------------------------------------------------------------
# Figura y animación
# -------------------------------------------------------------------
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Simulador de pata FL - seguimiento Bézier")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_xlim(-0.2, 0.8)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.8, 0.1)
ax.set_box_aspect([1, 1, 1])
ax.view_init(elev=49, azim=1, roll=0)

# Mostrar todos los waypoints (incluidos los no alcanzables, pero se distinguen)
tray_world = np.array([CADERA + p for p in trayectoria])
ax.plot(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
        '--', color='dodgerblue', linewidth=1.2, label='Waypoints')
ax.scatter(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
           color='blue', s=30, alpha=0.6)

# Línea de seguimiento real
linea_seguida, = ax.plot([], [], [], '-', color='crimson', linewidth=1.3,
                         alpha=0.85, label='Recorrido real')

# Curva Bézier planeada (se dibujará cuando se inicie)
curva_bezier, = ax.plot([], [], [], '-', color='lime', linewidth=2,
                        label='Bézier planeada')

# Elementos de la pierna (se actualizarán en cada frame)
O0, O1, O2, O3 = puntos_pata(estado['actual'])
l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]],
               color='orange', lw=4, label='Pierna')
l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]],
               color='orange', lw=4)
l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]],
               color='orange', lw=4)
articulaciones = ax.scatter([O0[0], O1[0], O2[0], O3[0]],
                            [O0[1], O1[1], O2[1], O3[1]],
                            [O0[2], O1[2], O2[2], O3[2]],
                            color='orange', s=50)
pie, = ax.plot([O3[0]], [O3[1]], [O3[2]], 'o', color='gold',
               markersize=10, markeredgecolor='black', label='Pie')

ax.legend(loc='upper left', fontsize=8)
texto_estado = fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')

# Botón para iniciar la ruta
ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])
boton = Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())

def actualizar(frame):
    if estado['siguiendo']:
        estado['t_segmento'] += DT
        s = np.clip(estado['t_segmento'] / DURACION_SEGMENTO, 0.0, 1.0)
        u = 3 * s**2 - 2 * s**3   # suavizado (easing)
        idx = estado['indice_segmento']
        p0, p1, p2, p3 = estado['segmentos'][idx]
        estado['actual'] = bezier_cubico(p0, p1, p2, p3, u)
        registro['historial'].append(estado['actual'].copy())

        if estado['t_segmento'] >= DURACION_SEGMENTO:
            avanzar_al_siguiente()

    # Actualizar la representación de la pierna
    O0, O1, O2, O3 = puntos_pata(estado['actual'])
    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    articulaciones._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                                 [O0[1], O1[1], O2[1], O3[1]],
                                 [O0[2], O1[2], O2[2], O3[2]])
    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Actualizar la línea de seguimiento
    if len(registro['historial']) > 1:
        hist = np.array(registro['historial'])
        hist_world = CADERA + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    # Dibujar la curva Bézier completa si se está siguiendo o ya terminó
    if estado['siguiendo'] or estado['terminado']:
        pts = []
        for seg in estado['segmentos']:
            p0, p1, p2, p3 = seg
            for t in np.linspace(0, 1, 20):
                pts.append(bezier_cubico(CADERA + p0, CADERA + p1,
                                         CADERA + p2, CADERA + p3, t))
        pts = np.array(pts)
        curva_bezier.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])

        if estado['siguiendo']:
            estado_txt = 'en ruta'
        else:
            estado_txt = 'completado'
        texto_estado.set_text(
            f"Segmento: {estado['indice_segmento'] + 1}/{len(estado['segmentos'])}\n"
            f"Puntos registrados: {len(registro['historial'])}\n"
            f"Estado: {estado_txt}"
        )
    else:
        curva_bezier.set_data_3d([], [], [])
        texto_estado.set_text("Presiona 'Iniciar Ruta' para comenzar")

    return []

ani = FuncAnimation(fig, actualizar, interval=int(DT * 1000), blit=False)
plt.tight_layout()
plt.show()