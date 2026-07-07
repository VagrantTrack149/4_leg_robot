import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

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
    Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX, Q3_MIN, Q3_MAX,
)

LADO = 1                       # pata delantera izquierda
CADERA = np.array([0.0, 0.0, 0.0])

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


# Estado y registro

estado = {
    'actual': np.array([0.0, -0.0528, -0.30]),   # posición inicial local
    'objetivo': np.array([0.0, -0.0528, -0.30]),
    'velocidad_interp': 0.2,
    'trayectoria': [],
    'indice_actual': 0,
    'siguiendo_trayectoria': False,
    'ruta_terminada': False,
    'tolerancia': 0.005,
}

registro = {
    'historial_actual': [],
    'historial_objetivo': [],
    'puntos_alcanzados': [],
    'indices_alcanzados': [],
    'fuera_de_alcance': [],
}


# Funciones auxiliares

def puntos_pata(pos_local):
    #devuelve las posiciones de las articulaciones en coordenadas del mundo.
    q = cinematica_inversa(pos_local[0], pos_local[1], pos_local[2], LADO)
    if q is None:
        # Si no es alcanzable, devolvemos una pose por defecto (vertical), despues cambiaremos esto espero... :/
        q = (np.pi/2, 0.0, 0.0)
    pts_locales = cinematica_directa(*q, LADO)
    return [CADERA + p for p in pts_locales]


# Configuración de la figura 3D

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Pierna FR - seguimiento lineal punto a punto")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
limite = 0.3
ax.set_xlim(-limite, limite)
ax.set_ylim(-limite, limite)
ax.set_zlim(-0.50, -0.20)
ax.set_box_aspect([1,1,1])
ax.view_init(elev=30, azim=45)

# Trayectoria planeada (coordenadas mundo)
tray_world = np.array([CADERA + p for p in trayectoria])
ax.plot(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
        '--', color='dodgerblue', linewidth=1.2, label='Trayectoria planeada')
ax.scatter(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
           color='blue', s=30, label='Puntos planeados')

linea_seguida, = ax.plot([], [], [], '-', color='crimson',
                         linewidth=1.3, alpha=0.85, label='Trayectoria real')

# Pierna inicial
O0, O1, O2, O3 = puntos_pata(estado['actual'])
l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='red', lw=4)
l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='red', lw=4)
l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='red', lw=4)
articulaciones = ax.scatter([O0[0], O1[0], O2[0], O3[0]],
                            [O0[1], O1[1], O2[1], O3[1]],
                            [O0[2], O1[2], O2[2], O3[2]],
                            color='red', s=40)
pie, = ax.plot([O3[0]], [O3[1]], [O3[2]], 'o', color='gold',
               markersize=10, markeredgecolor='black', label='Pie')
objetivo_plot, = ax.plot([CADERA[0]], [CADERA[1]], [CADERA[2]],
                         'x', color='green', markersize=12, markeredgewidth=3,
                         label='Objetivo')
ax.legend(loc='upper left', fontsize=8)

texto_estado = fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')

def actualizar_marcador_objetivo():
    x, y, z = estado['objetivo']
    objetivo_plot.set_data_3d([CADERA[0] + x], [CADERA[1] + y],
                              [CADERA[2] + z])


# Función para iniciar la ruta

def iniciar_trayectoria():
    if len(trayectoria) == 0:
        return
    trayectoria_ordenada = reorganizar_trayectoria(trayectoria, estado['actual'])
    # Filtrar puntos alcanzables
    puntos_validos = [p for p in trayectoria_ordenada if punto_alcanzable(p, LADO)]
    descartados = len(trayectoria_ordenada) - len(puntos_validos)
    if descartados > 0:
        print(f"Aviso: se descartaron {descartados} punto(s) de la trayectoria "
              f"por estar fuera de los límites articulares.")
    if len(puntos_validos) == 0:
        print("Ningún punto de la trayectoria es alcanzable. Ruta no iniciada.")
        return
    # Limpiar registros
    for key in registro:
        registro[key].clear()
    estado['trayectoria'] = puntos_validos
    estado['indice_actual'] = 0
    estado['siguiendo_trayectoria'] = True
    estado['ruta_terminada'] = False
    estado['objetivo'] = estado['trayectoria'][0]
    actualizar_marcador_objetivo()

# Botón iniciar
ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])
boton = Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())


# Reporte de error 

def mostrar_reporte_error():
    if len(registro['historial_actual']) == 0:
        return
    referencia = np.array(trayectoria)
    errores, rmse = calcular_errores(registro['historial_actual'], referencia)
    velocidad, aceleracion = calcular_velocidad_aceleracion(registro['historial_actual'], dt=0.02)

    print(f"\nRMSE (distancia a la trayectoria): {rmse:.5f} m\n")

    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(estado['tolerancia'], color='gray', linestyle='--',
                   label=f"tolerancia ({estado['tolerancia']} m)")
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':',
                   label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame")
    axs[0].set_ylabel("Error perpendicular (m)")
    axs[0].set_title("Error de seguimiento")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)

    tiempo_v = np.arange(len(velocidad)) * 0.02
    tiempo_a = np.arange(len(aceleracion)) * 0.02
    axs[1].plot(tiempo_v, velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Tiempo (s)")
    axs[1].set_ylabel("Magnitud (m/s, m/s²)")
    axs[1].set_title("Velocidad y aceleración de la trayectoria real")
    axs[1].legend()
    axs[1].grid(alpha=0.3)
    fig2.tight_layout()
    fig2.show()


# Función de actualización de la animación

def actualizar(frame):
    # Interpolación lineal hacia el objetivo
    error = estado['objetivo'] - estado['actual']
    estado['actual'] += estado['velocidad_interp'] * error
    registro['historial_actual'].append(estado['actual'].copy())
    registro['historial_objetivo'].append(estado['objetivo'].copy())

    if estado['siguiendo_trayectoria']:
        distancia = np.linalg.norm(estado['objetivo'] - estado['actual'])
        if distancia < estado['tolerancia']:
            # Verificar si el punto alcanzado está dentro de límites (para registro)
            alcanzable = punto_alcanzable(estado['actual'], LADO)
            registro['puntos_alcanzados'].append(estado['actual'].copy())
            registro['indices_alcanzados'].append(estado['indice_actual'])
            registro['fuera_de_alcance'].append(not alcanzable)

            estado['indice_actual'] += 1
            if estado['indice_actual'] >= len(estado['trayectoria']):
                estado['siguiendo_trayectoria'] = False
                if not estado['ruta_terminada']:
                    estado['ruta_terminada'] = True
                    mostrar_reporte_error()
            else:
                estado['objetivo'] = estado['trayectoria'][estado['indice_actual']]
                actualizar_marcador_objetivo()

    # Actualizar visualización de la pata
    O0, O1, O2, O3 = puntos_pata(estado['actual'])
    # Color según alcanzabilidad
    dentro = punto_alcanzable(estado['actual'], LADO)
    color = 'red' if dentro else 'orange'
    l01.set_color(color)
    l12.set_color(color)
    l23.set_color(color)

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    articulaciones._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                                 [O0[1], O1[1], O2[1], O3[1]],
                                 [O0[2], O1[2], O2[2], O3[2]])
    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Actualizar trayectoria real
    if len(registro['historial_actual']) > 1:
        hist = np.array(registro['historial_actual'])
        hist_world = CADERA + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    # Texto de estado
    if estado['siguiendo_trayectoria'] or estado['ruta_terminada']:
        n_alcanzados = len(registro['indices_alcanzados'])
        texto_estado.set_text(
            f"Punto: {estado['indice_actual']}/{len(estado['trayectoria'])}\n"
            f"Alcanzados: {n_alcanzados}\n"
            f"Estado: {'en ruta' if estado['siguiendo_trayectoria'] else 'completado'}"
        )
    return []

ani = FuncAnimation(fig, actualizar, interval=20, blit=False)
plt.tight_layout()
plt.show()