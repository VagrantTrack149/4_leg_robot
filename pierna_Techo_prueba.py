import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection




a1= 0.0528
L2= 0.2142
L3= 0.2142
Q1_MIN, Q1_MAX= 0.0, np.pi
Q2_MIN, Q2_MAX= -np.pi/2, np.pi/2
Q3_MIN, Q3_MAX= -np.pi, 0.0


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
trayectoria.append(np.array([-0.35,0,-0.25]))

#volumen escalon
# esquina1 [-0.30, 0.05, -0.15], esquina2 [-0.30, -0.05, -0.15], esquina3 [-0.40, 0.05, -0.15], esquina4 [-0.40, -0.05, -0.15]
# esquina1 [-0.30, 0.05, -0.30], esquina2 [-0.30, -0.05, -0.30], esquina3 [-0.40, 0.05, -0.30], esquina4 [-0.40, -0.05, -0.30]

ESCALON_X_MIN = -0.70
ESCALON_X_MAX = -0.40

ESCALON_Y_MIN = -0.15
ESCALON_Y_MAX = 0.15

ESCALON_Z_MIN = -0.40
ESCALON_Z_MAX = -0.28
vertices = np.array([
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

    # Puntos de control del Bezier cubico vigente: P0 (origen), P1, P2
    # (interpolados), P3 (destino)
    'p0': np.array([0.0, -a1, -0.30]),
    'p1': np.array([0.0, -a1, -0.30]),
    'p2': np.array([0.0, -a1, -0.30]),
    'p3': np.array([0.0, -a1, -0.30]),

    'tiempo_segmento': 0.50,
    't_segmento': 0.0,


    'indice_actual': 0,
    'siguiendo_trayectoria': False,
    'ruta_terminada': False,
    'segmento_bloqueado': False,   # True si el segmento actual no es viable (colision o fuera de rango)

    'tolerancia': 0.005,
}

#  Registro para analisis de trayectorias 
registro= {
    'historial_actual': [],  
    'historial_objetivo': [], 
    'puntos_alcanzados': [], 
    'indices_alcanzados': [],    
    'fuera_de_alcance': [],   
    'segmentos_bezier': [],       
}
# Cinematica directa
def cinematica_directa(q1, q2, q3, lado):
    c1, s1= np.cos(q1), np.sin(q1)
    c2, s2= np.cos(q2), np.sin(q2)
    c23, s23= np.cos(q2 + q3), np.sin(q2 + q3)

    O0= np.array([0.0, 0.0, 0.0])
    O1= np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2= O1 + np.array([L2 * s2, 0.0, -L2 * c2])
    O3= O2 + np.array([L3 * s23, 0.0, -L3 * c23])

    return O0, O1, O2, O3

# Cinematica inversa
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

def angulos_validos(q1, q2, q3):
    return (
        Q1_MIN <= q1 <= Q1_MAX and
        Q2_MIN <= q2 <= Q2_MAX and
        Q3_MIN <= q3 <= Q3_MAX
    )


def punto_alcanzable(P, lado):
    px, py, pz= P
    q1, q2, q3, fuera_geo= cinematica_inversa(px, py, pz, lado)
    fuera_rango= not angulos_validos(q1, q2, q3)
    return (not fuera_geo) and (not fuera_rango), (q1, q2, q3)


def bezier_cubico(p0, p1, p2, p3, u):
    return (
        (1 - u)**3 * p0 +
        3 * (1 - u)**2 * u * p1 +
        3 * (1 - u) * u**2 * p2 +
        u**3 * p3
    )


def puntos_control_bezier(p0, p3, elevacion=0.0):
    p1= p0 + (p3 - p0) * (1.0 / 3.0)
    p2= p0 + (p3 - p0) * (2.0 / 3.0)

    p1= p1.copy()
    p2= p2.copy()
    p1[2]+= elevacion
    p2[2]+= elevacion

    return p1, p2


def segmento_bezier_valido(p0, p1, p2, p3, lado, muestras=20):
    for u in np.linspace(0.0, 1.0, muestras):
        punto= bezier_cubico(p0, p1, p2, p3, u)
        ok, _= punto_alcanzable(punto, lado)
        if not ok:
            return False
    return True


def distancia_punto_segmento(P, A, B):
    AB= B - A
    AP= P - A
    largo2= np.dot(AB, AB)

    if largo2 < 1e-12:
        return np.linalg.norm(P - A)

    t= np.clip(np.dot(AP, AB) / largo2, 0.0, 1.0)
    proyeccion= A + t * AB

    return np.linalg.norm(P - proyeccion)


def punto_dentro_bloque(P):

    return (
        ESCALON_X_MIN <= P[0] <= ESCALON_X_MAX and
        ESCALON_Y_MIN <= P[1] <= ESCALON_Y_MAX and
        ESCALON_Z_MIN <= P[2] <= ESCALON_Z_MAX
    )


def segmento_colisiona(A, B, muestras=50):

    for t in np.linspace(0, 1, muestras):

        P = A + t * (B - A)

        if punto_dentro_bloque(P):
            return True

    return False

def bezier_colisiona(p0, p1, p2, p3, muestras=40):
    for u in np.linspace(0.0, 1.0, muestras):
        punto_local= bezier_cubico(p0, p1, p2, p3, u)
        if punto_dentro_bloque(CADERA_FR + punto_local):
            return True
    return False


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

# Linea de trayectoria seguida
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
funcion_control,= ax.plot(
    [], [], [],
    '-.', color='darkorange', linewidth=1.3, alpha=0.9,
    label='Bezier vigente'
)
ax.legend(loc='upper left', fontsize=8)



# Texto de progresos

texto_estado= fig.text(0.02, 0.95, "", fontsize=9, family='monospace', va='top')


def actualizar_marcador_objetivo():
    x, y, z= estado['objetivo']
    objetivo_plot.set_data_3d(
        [CADERA_FR[0] + x], [CADERA_FR[1] + y], [CADERA_FR[2] + z]
    )


def preparar_segmento(p0, p3):
    elevacion= 0.0
    elevacion_max= 0.20
    paso= 0.02

    p1, p2= puntos_control_bezier(p0, p3, elevacion)

    while bezier_colisiona(p0, p1, p2, p3):

        elevacion+= paso

        if elevacion > elevacion_max:
            # No fue posible esquivar el escalon solo con Z
            return p1, p2, True

        p1, p2= puntos_control_bezier(p0, p3, elevacion)

    # Curva libre de colision: verificar angulos articulares en todo
    # el recorrido (destino final + puntos intermedios)
    if not segmento_bezier_valido(p0, p1, p2, p3, LADO_FR):
        return p1, p2, True

    return p1, p2, False


def iniciar_trayectoria():

    if len(trayectoria)== 0:
        return

    # Reiniciar registro para una corrida limpia
    registro['historial_actual'].clear()
    registro['historial_objetivo'].clear()
    registro['puntos_alcanzados'].clear()
    registro['indices_alcanzados'].clear()
    registro['fuera_de_alcance'].clear()
    registro['segmentos_bezier'].clear()

    estado['indice_actual']= 0
    estado['ruta_terminada']= False

    estado['objetivo']= trayectoria[0]

    p0= estado['actual'].copy()
    p3= trayectoria[0].copy()
    p1, p2, bloqueado= preparar_segmento(p0, p3)

    estado['trayectoria']= trayectoria
    estado['p0']= p0
    estado['p1']= p1
    estado['p2']= p2
    estado['p3']= p3
    estado['segmento_bloqueado']= bloqueado
    estado['t_segmento']= 0.0

    registro['segmentos_bezier'].append((p0.copy(), p1.copy(), p2.copy(), p3.copy()))

    if bloqueado:
        estado['siguiendo_trayectoria']= False
        print(f"Ruta NO iniciada: el primer segmento (punto 0) no es viable "
              f"(colision sin solucion o angulos fuera de rango).")
    else:
        estado['siguiendo_trayectoria']= True

    actualizar_marcador_objetivo()



# Boton

ax_btn= plt.axes([0.75, 0.03, 0.18, 0.05])
boton= Button(ax_btn, 'Iniciar Ruta', color='lightgray', hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())



# Reporte final de error

def distancia_a_ruta_planeada(P, segmentos_bezier, muestras_por_segmento=60):
    mejor= np.inf

    for (p0, p1, p2, p3) in segmentos_bezier:

        curva= np.array([
            bezier_cubico(p0, p1, p2, p3, u)
            for u in np.linspace(0.0, 1.0, muestras_por_segmento)
        ])

        for i in range(len(curva) - 1):
            d= distancia_punto_segmento(P, curva[i], curva[i + 1])
            if d < mejor:
                mejor= d

    return mejor


def mostrar_reporte_error():

    historial= registro['historial_actual']
    segmentos= registro['segmentos_bezier']

    if len(historial)== 0 or len(segmentos)== 0:
        print("No hay datos suficientes para calcular error (no se alcanzo ningun punto).")
        return

    errores= np.array([
        distancia_a_ruta_planeada(P, segmentos)
        for P in historial
    ])

    rmse= np.sqrt(np.mean(errores ** 2))

    print("Error de trayectoria (distancia perpendicular punto-a-ruta)")
    print(f"RMSE: {rmse:.5f} m")

    #  Grafica de error por frame 
    fig2, ax_err= plt.subplots(1, 1, figsize=(9, 4.5))

    ax_err.plot(errores, '-', color='crimson', linewidth=1.2, label='Error perpendicular')
    ax_err.axhline(estado['tolerancia'], color='gray', linestyle='--',
                   label=f"tolerancia ({estado['tolerancia']} m)")
    ax_err.axhline(rmse, color='dodgerblue', linestyle=':',
                   label=f"RMSE ({rmse:.5f} m)")
    ax_err.set_xlabel("Frame")
    ax_err.set_ylabel("Distancia perpendicular a la ruta (m)")
    ax_err.set_title("Error de seguimiento de trayectoria")
    ax_err.legend(fontsize=8)
    ax_err.grid(alpha=0.3)

    fig2.tight_layout()
    fig2.show()



# Animacion

def actualizar(frame):

    dt= 0.02

    if estado['siguiendo_trayectoria'] and not estado['segmento_bloqueado']:

        estado['t_segmento']+= dt

        T= estado['tiempo_segmento']

        s= np.clip(estado['t_segmento'] / T, 0.0, 1.0)

        # suavizado temporal
        u= 3*s**2 - 2*s**3

        estado['actual']= bezier_cubico(
            estado['p0'], estado['p1'], estado['p2'], estado['p3'], u
        )

    # Registrar historial trayectoria
    registro['historial_actual'].append(estado['actual'].copy())
    registro['historial_objetivo'].append(estado['objetivo'].copy())

    # Cambio al siguiente punto
    if estado['siguiendo_trayectoria'] and not estado['segmento_bloqueado']:

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

                p0= estado['actual'].copy()
                p3= estado['objetivo'].copy()
                p1, p2, bloqueado= preparar_segmento(p0, p3)

                estado['p0']= p0
                estado['p1']= p1
                estado['p2']= p2
                estado['p3']= p3
                estado['segmento_bloqueado']= bloqueado
                estado['t_segmento']= 0.0

                registro['segmentos_bezier'].append((p0.copy(), p1.copy(), p2.copy(), p3.copy()))

                actualizar_marcador_objetivo()

                if bloqueado:
                    estado['siguiendo_trayectoria']= False
                    print(f"Ruta detenida en el punto {estado['indice_actual']}: "
                          f"el siguiente segmento no es viable "
                          f"(colision sin solucion o angulos fuera de rango).")
                    if not estado['ruta_terminada']:
                        estado['ruta_terminada']= True
                        mostrar_reporte_error()

    O0, O1, O2, O3= puntos_pata()
    if (segmento_colisiona(O1, O2) or segmento_colisiona(O2, O3)):
        l12.set_color('orange')
        l23.set_color('orange')
    else:
        l12.set_color('red')
        l23.set_color('red')

    l01.set_data_3d([O0[0], O1[0]], [O0[1], O1[1]], [O0[2], O1[2]])
    l12.set_data_3d([O1[0], O2[0]], [O1[1], O2[1]], [O1[2], O2[2]])
    l23.set_data_3d([O2[0], O3[0]], [O2[1], O3[1]], [O2[2], O3[2]])

    articulaciones._offsets3d= (
        [O0[0], O1[0], O2[0], O3[0]],
        [O0[1], O1[1], O2[1], O3[1]],
        [O0[2], O1[2], O2[2], O3[2]]
    )

    pie.set_data_3d([O3[0]], [O3[1]], [O3[2]])

    # Actualizar la linea de trayectoria seguida 
    if len(registro['historial_actual']) > 1:
        hist= np.array(registro['historial_actual'])
        hist_world= CADERA_FR + hist
        linea_seguida.set_data_3d(hist_world[:, 0], hist_world[:, 1], hist_world[:, 2])

    # Actualizar la curva Bezier vigente (4 puntos de control)
    if estado['siguiendo_trayectoria'] or estado['segmento_bloqueado']:
        curva= np.array([
            bezier_cubico(estado['p0'], estado['p1'], estado['p2'], estado['p3'], v)
            for v in np.linspace(0.0, 1.0, 30)
        ])
        curva_world= CADERA_FR + curva
        funcion_control.set_data_3d(curva_world[:, 0], curva_world[:, 1], curva_world[:, 2])
        funcion_control.set_color('red' if estado['segmento_bloqueado'] else 'darkorange')
    else:
        funcion_control.set_data_3d([], [], [])

    # Texto de progreso en vivo
    if estado['siguiendo_trayectoria'] or estado['ruta_terminada']:
        n_alcanzados= len(registro['indices_alcanzados'])

        if estado['siguiendo_trayectoria']:
            estado_txt= 'en ruta'
        elif estado['segmento_bloqueado']:
            estado_txt= 'detenido (bloqueado)'
        else:
            estado_txt= 'completado'

        texto_estado.set_text(
            f"Punto: {estado['indice_actual']}/{len(estado['trayectoria'])}\n"
            f"Alcanzados: {n_alcanzados}\n"
            f"Estado: {estado_txt}"
        )

    return []



# Ejecutar

ani= FuncAnimation(fig, actualizar, interval=20, blit=False)

plt.tight_layout()
plt.show()