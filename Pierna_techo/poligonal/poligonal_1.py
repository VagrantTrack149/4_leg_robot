import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Parametros
a1=0.0528; L2=0.2142; L3=0.2142
CADERA_FR=np.array([-0.29785,-0.055,0.0]); LADO_FR=-1

Q1_MIN, Q1_MAX= 0.0, np.pi
Q2_MIN, Q2_MAX= -np.pi/2, np.pi/2
Q3_MIN, Q3_MAX= -np.pi, 0.0

trayectoria= []


def generar_espiral_conica(centro=(0,0,-0.20), R0=0.06, Rf=0.03, vueltas=3,n_puntos=30, z_inicial=-0.30, z_final=-0.45):
    puntos=[]
    for i in range(n_puntos):
        t = i / (n_puntos-1)
        radio = R0 - (R0-Rf)*t
        angulo = 2*np.pi * vueltas * t
        x = centro[0] + radio*np.cos(angulo)
        y = centro[1] + radio*np.sin(angulo)
        z = z_inicial + (z_final-z_inicial)*t
        puntos.append(np.array([x,y,z]))
    return puntos

#trayectoria.append(np.array([0.0,0,-0.3]))
#trayectoria.append(np.array([-0.35,0,-0.25]))

trayectoria = generar_espiral_conica()


"""
radio= 0.05

centro_x= 0.0
centro_y= 0.0
centro_z= -0.27

num_puntos= 30

for t in np.linspace(0, 2*np.pi, num_puntos, endpoint=True):

    x= centro_x + radio*np.cos(2*t)
    y= centro_y + radio*np.sin(3*t)
    z= centro_z + radio*np.sin(5*t)

    trayectoria.append(np.array([x, y, z]))
"""

n = 10  # valor por defecto 

def aproximar_poligonal(puntos, n_valor):
    pts = np.asarray(puntos, dtype=float)

    if n_valor < 1:
        raise ValueError("n_valor debe ser >= 1")
    if len(pts) == 1 or n_valor == 1:
        return [pts[0].copy()]

    # longitud de arco acumulada sobre la trayectoria original
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
            vertices.append(p0 + t*(p1 - p0))
    return vertices

poligonal = aproximar_poligonal(trayectoria, n)

# volumen escalon
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

# Cinematica directa
def cinematica_directa(q1,q2,q3,lado):
    c1,s1=np.cos(q1),np.sin(q1); c2,s2=np.cos(q2),np.sin(q2); c23,s23=np.cos(q2+q3),np.sin(q2+q3)
    O0=np.array([0.0,0.0,0.0])
    O1=np.array([0.0,lado*a1*c1,-a1*s1])
    O2=O1+np.array([L2*s2,0.0,-L2*c2])
    O3=O2+np.array([L3*s23,0.0,-L3*c23])
    return O0,O1,O2,O3

# Cinematica inversa
def cinematica_inversa(px,py,pz,lado):
    py_l=py*lado
    ratio_sin_clip=py_l/a1
    ratio=np.clip(ratio_sin_clip,-1.0,1.0)
    q1=np.arccos(ratio)
    O1_z=-a1*np.sin(q1)
    dx=px; dz=pz-O1_z
    R_sin_clip=np.hypot(dx,dz)
    R=min(R_sin_clip,L2+L3-1e-6)
    C3_sin_clip=(R**2-L2**2-L3**2)/(2.0*L2*L3)
    C3=np.clip(C3_sin_clip,-1.0,1.0)
    q3=-np.arctan2(np.sqrt(1.0-C3**2),C3)
    k1=L2+L3*np.cos(q3); k2=L3*np.sin(q3)
    q2=np.arctan2(dx,-dz)-np.arctan2(k2,k1)
    fuera_de_alcance= (
        abs(ratio_sin_clip-ratio) > 1e-9
        or abs(R_sin_clip-R) > 1e-9
        or abs(C3_sin_clip-C3) > 1e-9
    )
    return q1,q2,q3,fuera_de_alcance

def angulos_validos(q1,q2,q3):
    return (
        Q1_MIN <= q1 <= Q1_MAX and
        Q2_MIN <= q2 <= Q2_MAX and
        Q3_MIN <= q3 <= Q3_MAX
    )

def punto_alcanzable(P, lado):
    px,py,pz= P
    q1,q2,q3,fuera_geo= cinematica_inversa(px,py,pz,lado)
    fuera_rango= not angulos_validos(q1,q2,q3)
    return (not fuera_geo) and (not fuera_rango), (q1,q2,q3)

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

# SEGMENTOS RECTOS (poligonales) con evasion de colision
def segmento_recto_colisiona(p0, p3, muestras=40):
    for u in np.linspace(0.0, 1.0, muestras):
        punto_local = p0 + u*(p3-p0)
        if punto_dentro_bloque(CADERA_FR + punto_local):
            return True
    return False

def segmento_recto_valido(p0, p3, lado, muestras=20):
    for u in np.linspace(0.0, 1.0, muestras):
        punto = p0 + u*(p3-p0)
        ok, _ = punto_alcanzable(punto, lado)
        if not ok:
            return False
    return True

def distancia_punto_segmento(P, A, B):
    AB = B - A
    AP = P - A
    largo2 = np.dot(AB, AB)
    if largo2 < 1e-12:
        return np.linalg.norm(P - A)
    t = np.clip(np.dot(AP, AB) / largo2, 0.0, 1.0)
    proyeccion = A + t * AB
    return np.linalg.norm(P - proyeccion)

def preparar_segmento_poligonal(p0, p3, lado):
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

        choca = any(
            segmento_recto_colisiona(cadena[i], cadena[i+1])
            for i in range(len(cadena)-1)
        )

        if not choca:
            valido = all(
                segmento_recto_valido(cadena[i], cadena[i+1], lado)
                for i in range(len(cadena)-1)
            )
            if valido:
                return cadena, False

        elevacion += paso
        if elevacion > elevacion_max:
            return [p0, p3], True

# Reorganizar trayectoria desde la posicion actual
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

# Preparar trayectoria poligonal completa
def preparar_trayectoria_completa_poligonal(puntos):
    puntos_validos = [p for p in puntos if punto_alcanzable(p, LADO_FR)[0]]

    if len(puntos_validos) < 2:
        return [], True

    vertices_totales = [puntos_validos[0]]
    bloqueado_global = False
    for i in range(len(puntos_validos)-1):
        p0 = puntos_validos[i]; p3 = puntos_validos[i+1]
        cadena, bloqueado = preparar_segmento_poligonal(p0, p3, LADO_FR)
        vertices_totales.extend(cadena[1:])
        if bloqueado:
            bloqueado_global = True
            break

    segmentos = [(vertices_totales[i], vertices_totales[i+1])
                 for i in range(len(vertices_totales)-1)]
    return segmentos, vertices_totales, bloqueado_global

# Estado y registro
estado={'actual':np.array([0.0,-a1,-0.30]),'t_segmento':0.0,'indice_segmento':0,
        'siguiendo':False,'terminado':False,'segmentos':[],'vertices':[],
        'segmento_bloqueado':False,'tolerancia':0.005}
registro={'historial':[],'segmentos_reales':[]}

def puntos_pata(pos):
    q1,q2,q3,_=cinematica_inversa(pos[0],pos[1],pos[2],LADO_FR)
    pts=cinematica_directa(q1,q2,q3,LADO_FR)
    return [CADERA_FR+p for p in pts]

def iniciar_trayectoria():
    if len(poligonal)<2: return
    poli_ordenada = reorganizar_trayectoria(poligonal, estado['actual'])
    registro['historial'].clear(); registro['segmentos_reales'].clear()
    segs, verts, bloqueado = preparar_trayectoria_completa_poligonal(poli_ordenada)
    estado['segmentos']=segs
    estado['vertices']=verts
    estado['actual']=poli_ordenada[0].copy()
    estado['indice_segmento']=0
    estado['t_segmento']=0.0
    estado['terminado']=False
    estado['segmento_bloqueado']=bloqueado

    if bloqueado:
        estado['siguiendo']=False
        print(f"Ruta no iniciada, se preparo hasta el segmento "
              f"{len(segs)} (colision sin solucion o angulos fuera de rango).")
    else:
        estado['siguiendo']=True
        print(f"Poligonal preparada con {len(segs)} segmentos rectos "
              f"(n={n} vertices, aproximando la trayectoria original de "
              f"{len(trayectoria)} puntos)")

def avanzar_al_siguiente():
    if estado['indice_segmento'] >= len(estado['segmentos'])-1:
        estado['siguiendo']=False
        estado['terminado']=True
        mostrar_reporte_error()
    else:
        estado['indice_segmento']+=1
        estado['t_segmento']=0.0

def mostrar_reporte_error():
    if len(registro['historial']) == 0: return
    hist_world = np.array([CADERA_FR + p for p in registro['historial']])

    tray_world = np.array([CADERA_FR + p for p in trayectoria])

    errores = []
    for P in hist_world:
        distancia = _distancia_a_polilinea(P, tray_world)
        errores.append(distancia)
 
    errores = np.array(errores)
    rmse = np.sqrt(np.mean(errores**2))
 
    print(f"\nRMSE (distancia a la trayectoria, n={n}): {rmse:.5f} m\n")
    fig2,axs=plt.subplots(2,1,figsize=(9,7))
    axs[0].plot(errores,'o-',color='crimson',markersize=3)
    axs[0].axhline(estado['tolerancia'], color='gray', linestyle='--',
                   label=f"tolerancia ({estado['tolerancia']} m)")
    axs[0].axhline(rmse, color='dodgerblue', linestyle=':',
                   label=f"RMSE ({rmse:.5f} m)")
    axs[0].set_xlabel("Frame"); axs[0].set_ylabel("Error perpendicular")
    axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3)
    hist = np.array(registro['historial'])

    # Paso de tiempo de la simulación
    dt = 0.02
    # Velocidad
    vel = np.diff(hist, axis=0) / dt
    velocidad = np.linalg.norm(vel, axis=1)
    # Aceleración
    ace = np.diff(vel, axis=0) / dt
    aceleracion = np.linalg.norm(ace, axis=1)
    tiempo_v = np.arange(len(velocidad)) * dt
    tiempo_a = np.arange(len(aceleracion)) * dt
    axs[1].plot(tiempo_v, velocidad,
                color='royalblue',
                label='Velocidad')
    axs[1].plot(tiempo_a, aceleracion,
                color='darkorange',
                label='Aceleración')

    axs[1].set_xlabel("Tiempo")
    axs[1].set_ylabel("Magnitud")
    axs[1].set_title(f"Velocidad y aceleración de la trayectoria poligonal (n={n})")
    axs[1].legend()
    axs[1].grid(alpha=0.3)
    fig2.tight_layout(); fig2.show()

# RMSE vs n
def _distancia_a_polilinea(P, ref):
    mejor = np.inf
    for i in range(len(ref)-1):
        d = distancia_punto_segmento(P, ref[i], ref[i+1])
        if d < mejor:
            mejor = d
    return mejor

def rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_valor, muestras_por_segmento=25):
    ref = np.asarray(puntos_trayectoria, dtype=float)
    verts = np.array(aproximar_poligonal(ref, n_valor))

    errores = []
    for i in range(len(verts)-1):
        p0, p1 = verts[i], verts[i+1]
        for u in np.linspace(0.0, 1.0, muestras_por_segmento, endpoint=False):
            punto = p0 + u*(p1-p0)
            errores.append(_distancia_a_polilinea(punto, ref))
    errores.append(_distancia_a_polilinea(verts[-1], ref))

    errores = np.array(errores)
    return np.sqrt(np.mean(errores**2))

RMSE_OBJETIVO = 0.005
N_AUTOMATICO = True     

def encontrar_n_automatico(puntos_trayectoria, rmse_objetivo, n_min=2, n_max=None):
    if n_max is None:
        n_max = len(puntos_trayectoria)
    for n_candidato in range(n_min, n_max + 1):
        rmse_actual = rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_candidato)
        if n_max>= 5:    
            if rmse_actual <= rmse_objetivo and n_candidato > 4:
                return n_candidato, rmse_actual
        if n_max < 5:
            if rmse_actual <= rmse_objetivo:
                return n_candidato, rmse_actual
    return n_max, rmse_poligonal_vs_trayectoria(puntos_trayectoria, n_max)

if N_AUTOMATICO:
    n, rmse_logrado = encontrar_n_automatico(trayectoria, RMSE_OBJETIVO)
    poligonal = aproximar_poligonal(trayectoria, n)
    cumplido = "si" if rmse_logrado <= RMSE_OBJETIVO else "no (se llego al maximo n posible)"
    print(f"n automatico = {n}  (RMSE = {rmse_logrado:.5f} m, objetivo <= {RMSE_OBJETIVO} m, cumplido: {cumplido})")

def graficar_rmse_vs_n(puntos_trayectoria, valores_n, n_usado):
    rmses = [rmse_poligonal_vs_trayectoria(puntos_trayectoria, nv) for nv in valores_n]

    fig3, ax3 = plt.subplots(figsize=(7,5))
    ax3.plot(valores_n, rmses, 'o-', color='darkviolet')
    ax3.axvline(n_usado, color='gray', linestyle='--', alpha=0.6, label=f"n usado en la simulacion = {n_usado}")
    ax3.set_xlabel("n (numero de vertices/segmentos de la poligonal)")
    ax3.set_ylabel("RMSE respecto a la trayectoria original [m]")
    ax3.set_title("Convergencia del RMSE de la poligonal al aumentar n")
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.3)
    fig3.tight_layout()
    fig3.show()
    return rmses

valores_n_prueba = list(range(2, len(trayectoria)+1))
rmses_por_n = graficar_rmse_vs_n(trayectoria, valores_n_prueba, n)

# Figura y animacion
fig=plt.figure(figsize=(12,8))
ax=fig.add_subplot(111,projection='3d')
ax.set_title(f"Trayectoria fija ({len(trayectoria)} pts) aproximada con poligonal n={n}")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
limite=0.3
ax.set_xlim(-limite, limite)
ax.set_ylim(-limite, limite)
ax.set_zlim(-0.50, -0.20)
ax.set_box_aspect([1,1,1])
ax.view_init(elev=30, azim=45)
ax.add_collection3d(cubo)

# Mostrar trayectoria planeada (vertices de la poligonal base)
tray_world=np.array([CADERA_FR+p for p in trayectoria])
ax.plot(tray_world[:,0],tray_world[:,1],tray_world[:,2],'--',color='dodgerblue',linewidth=1.2,label='Waypoints')
ax.scatter(tray_world[:,0],tray_world[:,1],tray_world[:,2],color='blue',s=30)

linea_seguida,=ax.plot([],[],[],'-',color='crimson',linewidth=1.3,alpha=0.85,label='Real')
curva_poligonal,=ax.plot([],[],[],'-',color='lime',linewidth=2,label='Poligonal')

# Pierna inicial
O0,O1,O2,O3=puntos_pata(poligonal[0])
l01,=ax.plot([O0[0],O1[0]],[O0[1],O1[1]],[O0[2],O1[2]],color='red',lw=4)
l12,=ax.plot([O1[0],O2[0]],[O1[1],O2[1]],[O1[2],O2[2]],color='red',lw=4)
l23,=ax.plot([O2[0],O3[0]],[O2[1],O3[1]],[O2[2],O3[2]],color='red',lw=4)
articulaciones=ax.scatter([O0[0],O1[0],O2[0],O3[0]],[O0[1],O1[1],O2[1],O3[1]],[O0[2],O1[2],O2[2],O3[2]],color='red',s=40)
pie,=ax.plot([O3[0]],[O3[1]],[O3[2]],'o',color='gold',markersize=10,markeredgecolor='black')
objetivo_plot,=ax.plot([CADERA_FR[0]],[CADERA_FR[1]],[CADERA_FR[2]],'x',color='green',markersize=12,markeredgewidth=3)
ax.legend(loc='upper left',fontsize=8)
texto_estado=fig.text(0.02,0.95,"",fontsize=9,family='monospace',va='top')
ax_btn=plt.axes([0.75,0.03,0.18,0.05])
boton=Button(ax_btn,'Iniciar Ruta',color='lightgray',hovercolor='lightblue')
boton.on_clicked(lambda event: iniciar_trayectoria())

def actualizar(frame):
    dt=0.02
    if estado['siguiendo']:
        estado['t_segmento']+=dt
        s=np.clip(estado['t_segmento']/0.50,0.0,1.0)
        u=3*s**2-2*s**3
        idx=estado['indice_segmento']
        p0,p1=estado['segmentos'][idx]
        estado['actual']=p0 + u*(p1-p0)
        registro['historial'].append(estado['actual'].copy())
        if estado['t_segmento']>=0.50:
            registro['segmentos_reales'].append((p0.copy(),p1.copy()))
            avanzar_al_siguiente()
    O0,O1,O2,O3=puntos_pata(estado['actual'])

    if (segmento_colisiona(O1, O2) or segmento_colisiona(O2, O3)):
        l12.set_color('orange')
        l23.set_color('orange')
    else:
        l12.set_color('red')
        l23.set_color('red')

    l01.set_data_3d([O0[0],O1[0]],[O0[1],O1[1]],[O0[2],O1[2]])
    l12.set_data_3d([O1[0],O2[0]],[O1[1],O2[1]],[O1[2],O2[2]])
    l23.set_data_3d([O2[0],O3[0]],[O2[1],O3[1]],[O2[2],O3[2]])
    articulaciones._offsets3d=([O0[0],O1[0],O2[0],O3[0]],[O0[1],O1[1],O2[1],O3[1]],[O0[2],O1[2],O2[2],O3[2]])
    pie.set_data_3d([O3[0]],[O3[1]],[O3[2]])
    if len(registro['historial'])>1:
        hist=np.array(registro['historial'])
        hist_world=CADERA_FR+hist
        linea_seguida.set_data_3d(hist_world[:,0],hist_world[:,1],hist_world[:,2])
    if estado['siguiendo'] or estado['terminado']:
        if len(estado['vertices'])>0:
            verts_world=np.array([CADERA_FR+v for v in estado['vertices']])
            curva_poligonal.set_data_3d(verts_world[:,0],verts_world[:,1],verts_world[:,2])
        curva_poligonal.set_color('red' if estado['segmento_bloqueado'] else 'lime')
    else:
        curva_poligonal.set_data_3d([],[],[])
    if estado['siguiendo'] or estado['terminado']:
        if estado['siguiendo']:
            estado_txt='en ruta'
        elif estado['segmento_bloqueado']:
            estado_txt='detenido (bloqueado)'
        else:
            estado_txt='completado'
        texto_estado.set_text(f"Segmento: {estado['indice_segmento']+1}/{len(estado['segmentos'])}\nPuntos: {len(registro['historial'])}\nEstado: {estado_txt}")
    return []

ani=FuncAnimation(fig,actualizar,interval=20,blit=False)
plt.tight_layout()
plt.show()