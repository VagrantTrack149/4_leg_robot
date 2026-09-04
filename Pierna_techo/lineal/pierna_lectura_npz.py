import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import os

from pata_common import (
    cinematica_directa, cinematica_inversa, punto_alcanzable,
    reorganizar_trayectoria, calcular_errores, calcular_velocidad_aceleracion,
    cargar_ruta_resultante_npz, a1
)


LADO = 1  # 1 para izquierda (True), 0 para derecha (False)
CADERA = np.array([0.0, 0.0, 0.0]) # Origen de la pata en el mundo

NPZ_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "circulo_p10_g750_pop600_rep0.npz")
N_PUNTOS_TRAYECTORIA = 60
ESCALA_NPZ = 3e-5  # Factor de escala
# Centro local de la trayectoria ajustado al espacio alcanzable de la pata
CENTRO_LOCAL = np.array([0.0, -0.30, -0.30]) 

# Cargar, centrar y procesar trayectoria desde el NPZ
try:
    ruta_cruda = cargar_ruta_resultante_npz(NPZ_PATH, n_puntos=N_PUNTOS_TRAYECTORIA)
    
    ruta_centrada = ruta_cruda - np.mean(ruta_cruda, axis=0)
    
    trayectoria = [CENTRO_LOCAL + ESCALA_NPZ * np.array([p[0], p[2], p[1]]) for p in ruta_centrada]
except Exception as e:
    print(f"Error cargando NPZ: {e}.")
    t = np.linspace(0, 2*np.pi, N_PUNTOS_TRAYECTORIA)
    trayectoria = [CENTRO_LOCAL + np.array([0.1*np.cos(ti), 0.1*np.sin(ti), 0.0]) for ti in t]

# Posición inicial de la pata alineada con el inicio de la trayectoria
inicio_trayectoria = trayectoria[0] if len(trayectoria) > 0 else np.array([0.0, -0.30, -0.30])

estado = {
    'actual': inicio_trayectoria.copy(),   
    'objetivo': inicio_trayectoria.copy(),
    'velocidad_interp': 0.15,                # Suavidad de la interpolación
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
}

#  Funciones Auxiliares de Visualización 
def puntos_pata(pos_local):
    """Devuelve las posiciones de las 4 articulaciones (Base, Hombro, Rodilla, Pie)"""
    q = cinematica_inversa(pos_local[0], pos_local[1], pos_local[2], left=bool(LADO))
    if q is None:
        q = (0.0, 0.5, -0.8) 
    pts_locales = cinematica_directa(*q, left=bool(LADO))
    return [CADERA + p for p in pts_locales]

#  Configuración de la figura 3D 
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Pierna trayectoria desde NPZ")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.view_init(elev=-50, azim=46, roll=43)
limite = 0.4
ax.set_xlim(-limite, limite)
ax.set_ylim(-limite, limite)
ax.set_zlim(-0.60, 0.10)
ax.set_box_aspect([1,1,1])

# Trayectoria planeada
tray_world = np.array([CADERA + p for p in trayectoria])
ax.plot(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
        '--', color='dodgerblue', linewidth=1.5, label='Trayectoria planeada')
ax.scatter(tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
           color='blue', s=20, alpha=0.6)

linea_seguida, = ax.plot([], [], [], '-', color='crimson', linewidth=2, alpha=0.8, label='Trayectoria real')

# Inicializar pierna
O0, O1, O2, O3 = puntos_pata(estado['actual'])
l01, = ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='black', lw=5, solid_capstyle='round')
l12, = ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='darkgray', lw=4, solid_capstyle='round')
l23, = ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='dimgray', lw=3, solid_capstyle='round')

articulaciones = ax.scatter([O0[0], O1[0], O2[0], O3[0]],
                            [O0[1], O1[1], O2[1], O3[1]],
                            [O0[2], O1[2], O2[2], O3[2]],
                            color='gold', s=50, edgecolor='black')

objetivo_plot, = ax.plot([estado['objetivo'][0]], [estado['objetivo'][1]], [estado['objetivo'][2]],
                         'x', color='green', markersize=12, markeredgewidth=3, label='Objetivo actual')

ax.legend(loc='upper left', fontsize=9)
texto_estado = fig.text(0.02, 0.95, "", fontsize=10, family='monospace', va='top', 
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

def actualizar_marcador_objetivo():
    x, y, z = estado['objetivo']
    objetivo_plot.set_data_3d([x], [y], [z])

#  Lógica de Control 
def iniciar_trayectoria():
    if len(trayectoria) == 0:
        return
    
    trayectoria_ordenada = reorganizar_trayectoria(trayectoria, estado['actual'])
    puntos_validos = [p for p in trayectoria_ordenada if punto_alcanzable(p, left=bool(LADO))]
    descartados = len(trayectoria_ordenada) - len(puntos_validos)
    
    if descartados > 0:
        print(f"Aviso: se descartaron {descartados} punto(s) por estar fuera de los límites.")
        
    if len(puntos_validos) == 0:
        print("Ningún punto es alcanzable. Ruta no iniciada.")
        return

    for key in registro:
        registro[key].clear()
        
    estado['trayectoria'] = puntos_validos
    estado['indice_actual'] = 0
    estado['siguiendo_trayectoria'] = True
    estado['ruta_terminada'] = False
    estado['objetivo'] = estado['trayectoria'][0]
    actualizar_marcador_objetivo()
    print("Ruta iniciada.")

# Botón iniciar
ax_btn = plt.axes([0.75, 0.03, 0.18, 0.05])
boton = Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())

# Reporte final
def mostrar_reporte_error():
    if len(registro['historial_actual']) == 0:
        return
    
    referencia = np.array(trayectoria)
    errores, rmse = calcular_errores(registro['historial_actual'], referencia)
    velocidad, aceleracion = calcular_velocidad_aceleracion(registro['historial_actual'], dt=0.033)
    
    print(f"\nREPORTE FINAL")
    print(f"RMSE (distancia a la trayectoria): {rmse:.5f} m\n")

    fig2, axs = plt.subplots(2, 1, figsize=(9, 7))
    axs[0].plot(errores, 'o-', color='crimson', markersize=3)
    axs[0].axhline(estado['tolerancia'], color='gray', linestyle='--', label=f"Tolerancia")
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':', label=f"RMSE ({rmse:.4f} m)")
    axs[0].set_ylabel("Error (m)")
    axs[0].set_title("Error de seguimiento")
    axs[0].legend()
    axs[0].grid(alpha=0.3)

    axs[1].plot(velocidad, color='royalblue', label='Velocidad')
    axs[1].plot(aceleracion, color='darkorange', label='Aceleración')
    axs[1].set_xlabel("Frame")
    axs[1].set_ylabel("Magnitud")
    axs[1].set_title("Velocidad y aceleración")
    axs[1].legend()
    axs[1].grid(alpha=0.3)

    fig2.tight_layout()
    fig2.show()

#  Bucle de Animación 
def actualizar(frame):
    error = estado['objetivo'] - estado['actual']
    dist = np.linalg.norm(error)
    
    if dist > 0.001:
        estado['actual'] += estado['velocidad_interp'] * error
    
    registro['historial_actual'].append(estado['actual'].copy())
    registro['historial_objetivo'].append(estado['objetivo'].copy())

    if estado['siguiendo_trayectoria']:
        if dist < estado['tolerancia']:
            registro['puntos_alcanzados'].append(estado['actual'].copy())
            registro['indices_alcanzados'].append(estado['indice_actual'])
            
            estado['indice_actual'] += 1
            if estado['indice_actual'] >= len(estado['trayectoria']):
                estado['siguiendo_trayectoria'] = False
                if not estado['ruta_terminada']:
                    estado['ruta_terminada'] = True
                    mostrar_reporte_error()
            else:
                estado['objetivo'] = estado['trayectoria'][estado['indice_actual']]
                actualizar_marcador_objetivo()

    O0, O1, O2, O3 = puntos_pata(estado['actual'])
    
    dentro = punto_alcanzable(estado['actual'], left=bool(LADO))
    color = 'black' if dentro else 'red'
    
    l01.set_color(color)
    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])
    
    articulaciones._offsets3d = ([O0[0], O1[0], O2[0], O3[0]],
                                 [O0[1], O1[1], O2[1], O3[1]],
                                 [O0[2], O1[2], O2[2], O3[2]])

    if len(registro['historial_actual']) > 1:
        hist = np.array(registro['historial_actual'])
        linea_seguida.set_data_3d(hist[:, 0], hist[:, 1], hist[:, 2])

    estado_txt = 'en ruta' if estado['siguiendo_trayectoria'] else ('completado' if estado['ruta_terminada'] else 'esperando')
    texto_estado.set_text(
        f"Punto: {estado['indice_actual']}/{len(estado['trayectoria'])}\n"
        f"Alcanzados: {len(registro['indices_alcanzados'])}\n"
        f"Estado: {estado_txt}"
    )
    
    return []

ani = FuncAnimation(fig, actualizar, interval=33, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()