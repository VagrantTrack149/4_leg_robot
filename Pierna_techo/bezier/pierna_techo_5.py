import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection



# Parámetros de la pierna

a1= 0.0528
L2= 0.2142
L3= 0.2142


# Cadera
CADERA_FR= np.array([-0.29785, -0.055, 0.0])
LADO_FR= -1



# Trayectoria planeada

trayectoria= []

radio= 0.05

centro_x= 0.0
centro_y= 0.0
centro_z= -0.27

num_puntos= 50
"""
for t in np.linspace(0, 2*np.pi, num_puntos, endpoint=False):

    x= centro_x + radio*np.cos(2*t)
    y= centro_y + radio*np.sin(3*t)
    z= centro_z + radio*np.sin(5*t)

    trayectoria.append(np.array([x, y, z]))
"""
#puntos de escalones
trayectoria.append(np.array([0.0,0,-0.3]))
trayectoria.append(np.array([-0.35,0,-0.10]))

#volumen escalon
# esquina1 [-0.30, 0.05, -0.15], esquina2 [-0.30, -0.05, -0.15], esquina3 [-0.40, 0.05, -0.15], esquina4 [-0.40, -0.05, -0.15]
# esquina1 [-0.30, 0.05, -0.30], esquina2 [-0.30, -0.05, -0.30], esquina3 [-0.40, 0.05, -0.30], esquina4 [-0.40, -0.05, -0.30]

ESCALON_X_MIN = -0.40
ESCALON_X_MAX = -0.30

ESCALON_Y_MIN = -0.05
ESCALON_Y_MAX =  0.05

ESCALON_Z_MIN = -0.30
ESCALON_Z_MAX = -0.15
vertices = np.array([
    [-0.50,  0.15, -0.15],
    [-0.50, -0.15, -0.15],
    [-0.60, -0.15, -0.15],
    [-0.60,  0.15, -0.15],

    [-0.50,  0.15, -0.30],
    [-0.50, -0.15, -0.30],
    [-0.60, -0.15, -0.30],
    [-0.60,  0.15, -0.30],
])

caras = [
    [vertices[0], vertices[1], vertices[2], vertices[3]],
    [vertices[4], vertices[5], vertices[6], vertices[7]],
    [vertices[0], vertices[1], vertices[5], vertices[4]],
    [vertices[2], vertices[3], vertices[7], vertices[6]],
    [vertices[0], vertices[3], vertices[7], vertices[4]],
    [vertices[1], vertices[2], vertices[6], vertices[5]],
]

cubo = Poly3DCollection(
    caras,
    alpha=0.35,
    facecolor='gray',
    edgecolor='black'
)


# Estado del control

estado= {
    'actual': np.array([0.0, -a1, -0.30]),
    'objetivo': np.array([0.0, -a1, -0.30]),

    'trayectoria': [],
    'p0': np.array([0.0, -a1, -0.30]),
    'p1': np.array([0.0, -a1, -0.30]),
    'tiempo_segmento': 0.50,
    't_segmento': 0.0,


    'indice_actual': 0,
    'siguiendo_trayectoria': False,
    'ruta_terminada': False,

    'tolerancia': 0.005,
}

#  Registro para análisis de trayectorias 
registro= {
    'historial_actual': [],       # posición real del pie en cada frame (relativo a cadera)
    'historial_objetivo': [],     # punto objetivo
    'puntos_alcanzados': [],      # posición real del pie en el instante en que se alcanzo
    'indices_alcanzados': [],     # índice del punto correspondiente, en el mismo orden
    'fuera_de_alcance': [],       # True/False por punto recortado
}
# Cinemática directa
def cinematica_directa(q1, q2, q3, lado):
    c1, s1= np.cos(q1), np.sin(q1)
    c2, s2= np.cos(q2), np.sin(q2)
    c23, s23= np.cos(q2 + q3), np.sin(q2 + q3)

    O0= np.array([0.0, 0.0, 0.0])
    O1= np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2= O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3= O2 + np.array([L3 * s23, 0.0, -L3 * c23])

    return O0, O1, O2, O3

# Cinemática inversa
def cinematica_inversa(px, py, pz, lado):

    py_l= py * lado
    ratio_sin_clip= py_l / a1
    ratio= np.clip(ratio_sin_clip, -1.0, 1.0)
    q1= np.arccos(ratio)

    O1_z= -a1 * np.sin(q1)

    dx= px
    dz= pz - O1_z

    R_sin_clip= np.hypot(dx, dz)
    R= min(R_sin_clip, L2 + L3 - 1e-6)

    C3_sin_clip= (R ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    C3= np.clip(C3_sin_clip, -1.0, 1.0)

    q3= -np.arctan2(np.sqrt(1.0 - C3 ** 2), C3)

    k1= L2 + L3 * np.cos(q3)
    k2= L3 * np.sin(q3)

    q2= np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    fuera_de_alcance= (
        abs(ratio_sin_clip - ratio) > 1e-9
        or abs(R_sin_clip - R) > 1e-9
        or abs(C3_sin_clip - C3) > 1e-9
    )

    return q1, q2, q3, fuera_de_alcance

def puntos_pata():

    px, py, pz= estado['actual']

    q1, q2, q3, _= cinematica_inversa(px, py, pz, LADO_FR)

    pts_locales= cinematica_directa(q1, q2, q3, LADO_FR)

    return [CADERA_FR + p for p in pts_locales]



# Figura

fig= plt.figure(figsize=(12, 8))
ax= fig.add_subplot(111, projection='3d')

ax.set_title("Una pierna seguimiento de puntos desde techo control de trayectorias")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-0.8, 0.1)
ax.set_ylim(-0.350, 0.035)
ax.set_zlim(-0.50, -0.08)

ax.set_box_aspect([1, 1, 1])

ax.view_init(elev=25, azim=145)
ax.add_collection3d(cubo)



# Dibujar trayectoria planeada

tray_world= np.array([CADERA_FR + p for p in trayectoria])

ax.plot(
    tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
    '--', color='dodgerblue', linewidth=1.2, label='Trayectoria planeada'
)

ax.scatter(
    tray_world[:, 0], tray_world[:, 1], tray_world[:, 2],
    color='blue', s=30, label='Puntos planeados'
)

# Línea de trayectoria seguida
linea_seguida,= ax.plot(
    [], [], [],
    '-', color='crimson', linewidth=1.3, alpha=0.85,
    label='Trayectoria'
)



# Dibujar pierna

O0, O1, O2, O3= puntos_pata()

l01,= ax.plot([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]], color='red', lw=4)
l12,= ax.plot([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]], color='red', lw=4)
l23,= ax.plot([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]], color='red', lw=4)

articulaciones= ax.scatter(
    [O0[0], O1[0], O2[0], O3[0]],
    [O0[1], O1[1], O2[1], O3[1]],
    [O0[2], O1[2], O2[2], O3[2]],
    color='red', s=40
)

pie,= ax.plot(
    [O3[0]], [O3[1]], [O3[2]],
    'o', color='gold', markersize=10, markeredgecolor='black', label='Pie'
)



# Marcador del objetivo vigente

objetivo_plot,= ax.plot(
    [CADERA_FR[0]], [CADERA_FR[1]], [CADERA_FR[2]],
    'x', color='green', markersize=12, markeredgewidth=3, label='Objetivo'
)
funcion_control= ax.plot(
    [], [], [],
    label='Funcion \n Bézier'
)
ax.legend(loc='upper left', fontsize=8)



# Texto de progresos

texto_estado= fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')


def actualizar_marcador_objetivo():
    x, y, z= estado['objetivo']
    objetivo_plot.set_data_3d(
        [CADERA_FR[0] + x], [CADERA_FR[1] + y], [CADERA_FR[2] + z]
    )


def iniciar_trayectoria():

    if len(trayectoria)== 0:
        return

    # Reiniciar registro para una corrida limpia
    registro['historial_actual'].clear()
    registro['historial_objetivo'].clear()
    registro['puntos_alcanzados'].clear()
    registro['indices_alcanzados'].clear()
    registro['fuera_de_alcance'].clear()

    estado['trayectoria']= trayectoria
    estado['indice_actual']= 0
    estado['siguiendo_trayectoria']= True
    estado['ruta_terminada']= False

    estado['objetivo']= trayectoria[0]
    estado['p0']= estado['actual'].copy()
    estado['p1']= trayectoria[0].copy()
    estado['t_segmento']= 0.0

    actualizar_marcador_objetivo()



# Botón

ax_btn= plt.axes([0.75, 0.03, 0.18, 0.05])
boton= Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())



# Reporte final de error

def mostrar_reporte_error():

    planeados= np.array([trayectoria[i] for i in registro['indices_alcanzados']])
    alcanzados= np.array(registro['puntos_alcanzados'])

    if len(planeados)== 0:
        print("No hay datos suficientes para calcular error (no se alcanzó ningún punto).")
        return

    errores_vec= alcanzados - planeados                  # (N, 3)
    errores_norma= np.linalg.norm(errores_vec, axis=1)    # (N,)

    n_fuera= sum(registro['fuera_de_alcance'])

    print("Error de trayectoria")
    print(f"Puntos totales: {len(trayectoria)}")
    print(f"Puntos alcanzados: {len(planeados)}")
    print(f"Puntos fuera de alcance: {n_fuera}  ")
    print(f"Error promedio: {errores_norma.mean():.5f} m")
    print(f"Error max: {errores_norma.max():.5f} m  (punto #{int(np.argmax(errores_norma))})")
    print(f"Error min: {errores_norma.min():.5f} m")

    print("Error promedio por eje (x, y, z):")
    print(f"  x: {errores_vec[:,0].mean():+.5f} m   (desv. std: {errores_vec[:,0].std():.5f})")
    print(f"  y: {errores_vec[:,1].mean():+.5f} m   (desv. std: {errores_vec[:,1].std():.5f})")
    print(f"  z: {errores_vec[:,2].mean():+.5f} m   (desv. std: {errores_vec[:,2].std():.5f})")
    #  Gráfica de error por punto 
    fig2, (ax_err, ax_comp)= plt.subplots(2, 1, figsize=(9, 7))

    ax_err.plot(registro['indices_alcanzados'], errores_norma, 'o-', color='crimson')
    ax_err.axhline(estado['tolerancia'], color='gray', linestyle='--',
                   label=f"tolerancia ({estado['tolerancia']} m)")
    ax_err.set_xlabel("Índice de punto")
    ax_err.set_ylabel("Error planeado - real")
    ax_err.set_title("Error de seguimiento por punto")
    ax_err.legend()
    ax_err.grid(alpha=0.3)

    ax_comp.plot(registro['indices_alcanzados'], planeados[:, 0], 'o--', color='dodgerblue', label='x planeado')
    ax_comp.plot(registro['indices_alcanzados'], alcanzados[:, 0], 'x-', color='crimson', label='x real')
    ax_comp.plot(registro['indices_alcanzados'], planeados[:, 2], 'o--', color='seagreen', label='z planeado')
    ax_comp.plot(registro['indices_alcanzados'], alcanzados[:, 2], 'x-', color='darkorange', label='z real')
    ax_comp.set_xlabel("Índice de punto")
    ax_comp.set_ylabel("Posición relativa a la cadera")
    ax_comp.set_title("Comparación: planeado vs  alcanzado (X y Z)")
    ax_comp.legend(fontsize=8)
    ax_comp.grid(alpha=0.3)

    fig2.tight_layout()
    fig2.show()



# Animación

def actualizar(frame):

    dt= 0.02

    if estado['siguiendo_trayectoria']:

        estado['t_segmento']+= dt

        T= estado['tiempo_segmento']

        s= np.clip(estado['t_segmento'] / T, 0.0, 1.0)

        # suavizado temporal
        u= 3*s**2 - 2*s**3

        p0= estado['p0']
        p2= estado['p1']

        # punto de control
        altura = 0.02

        pm = (p0 + p2)/2
        pm[2] += altura
        interseca_x = (
            min(p0[0], p2[0]) <= ESCALON_X_MAX and
            max(p0[0], p2[0]) >= ESCALON_X_MIN
        )

        interseca_y = (
            min(p0[1], p2[1]) <= ESCALON_Y_MAX and
            max(p0[1], p2[1]) >= ESCALON_Y_MIN
        )

        if interseca_x and interseca_y:

            margen = 0.03

            pm[2] = max(
                p0[2],
                p2[2],
                ESCALON_Z_MAX + margen
            )

        else:

            pm[2] += altura

        estado['actual']= (
            (1-u)**2 * p0 +
            2*(1-u)*u * pm +
            u**2 * p2
        )

    # Registrar historial trayectoria
    registro['historial_actual'].append(estado['actual'].copy())
    registro['historial_objetivo'].append(estado['objetivo'].copy())

    # Cambio al siguiente punto
    if estado['siguiendo_trayectoria']:

        if estado['t_segmento'] >= estado['tiempo_segmento']:

            # Verificar si este punto estaba fuera de alcance
            px, py, pz= estado['objetivo']
            _, _, _, fuera= cinematica_inversa(px, py, pz, LADO_FR)

            # Registrar punto alcanzado
            registro['puntos_alcanzados'].append(estado['actual'].copy())
            registro['indices_alcanzados'].append(estado['indice_actual'])
            registro['fuera_de_alcance'].append(fuera)

            estado['indice_actual']+= 1

            if estado['indice_actual'] >= len(estado['trayectoria']):

                estado['siguiendo_trayectoria']= False

                if not estado['ruta_terminada']:
                    estado['ruta_terminada']= True
                    mostrar_reporte_error()

            else:

                estado['objetivo']= estado['trayectoria'][estado['indice_actual']]

                estado['p0']= estado['actual'].copy()
                estado['p1']= estado['objetivo'].copy()
                estado['t_segmento']= 0.0

                actualizar_marcador_objetivo()

    O0, O1, O2, O3= puntos_pata()

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])

    articulaciones._offsets3d= (
        [O0[0], O1[0], O2[0], O3[0]],
        [O0[1], O1[1], O2[1], O3[1]],
        [O0[2], O1[2], O2[2], O3[2]]
    )

    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Actualizar la línea de trayectoria seguida 
    if len(registro['historial_actual']) > 1:
        hist= np.array(registro['historial_actual'])
        hist_world= CADERA_FR + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    # Texto de progreso en vivo
    if estado['siguiendo_trayectoria'] or estado['ruta_terminada']:
        n_alcanzados= len(registro['indices_alcanzados'])
        texto_estado.set_text(
            f"Punto: {estado['indice_actual']}/{len(estado['trayectoria'])}\n"
            f"Alcanzados: {n_alcanzados}\n"
            f"Estado: {'en ruta' if estado['siguiendo_trayectoria'] else 'completado'}"
        )

    return []



# Ejecutar

ani= FuncAnimation(fig, actualizar, interval=20, blit=False)

plt.tight_layout()
plt.show()