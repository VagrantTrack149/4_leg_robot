import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

# Parámetros
a1=0.0528; L2=0.2142; L3=0.2142
CADERA_FR=np.array([-0.29785,-0.055,0.0]); LADO_FR=-1

# Trayectoria espiral cónica hacia abajo
def generar_espiral_conica(centro=(0,0,-0.20), R0=0.06, Rf=0.03, vueltas=2.5, n_puntos=30, z_inicial=-0.30, z_final=-0.45):
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

trayectoria = generar_espiral_conica()
"""
trayectoria= []

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

# Cinemática directa
def cinematica_directa(q1,q2,q3,lado):
    c1,s1=np.cos(q1),np.sin(q1); c2,s2=np.cos(q2),np.sin(q2); c23,s23=np.cos(q2+q3),np.sin(q2+q3)
    O0=np.array([0.0,0.0,0.0])
    O1=np.array([0.0,lado*a1*c1,-a1*s1])
    O2=O1+np.array([L2*s2,0.0,-L2*c2])
    O3=O2+np.array([L3*s23,0.0,-L3*c23])
    return O0,O1,O2,O3

# Cinemática inversa
def cinematica_inversa(px,py,pz,lado):
    py_l=py*lado
    ratio=np.clip(py_l/a1,-1.0,1.0)
    q1=np.arccos(ratio)
    O1_z=-a1*np.sin(q1)
    dx=px; dz=pz-O1_z
    R=min(np.hypot(dx,dz),L2+L3-1e-6)
    C3=np.clip((R**2-L2**2-L3**2)/(2.0*L2*L3),-1.0,1.0)
    q3=-np.arctan2(np.sqrt(1.0-C3**2),C3)
    k1=L2+L3*np.cos(q3); k2=L3*np.sin(q3)
    q2=np.arctan2(dx,-dz)-np.arctan2(k2,k1)
    return q1,q2,q3

# Bezier cúbico
def bezier_cubico(p0,p1,p2,p3,t):
    u=1-t
    return u**3*p0+3*u**2*t*p1+3*u*t**2*p2+t**3*p3

# Preparar segmento
def preparar_segmento(p0,p3,tang_in,tang_out):
    factor_curv=1.5
    p1=p0 + (1.0/3.0)*tang_in*factor_curv
    p2=p3 - (1.0/3.0)*tang_out*factor_curv
    return [(p0,p1,p2,p3)], False

# Calcular tangentes
def calcular_tangentes(puntos):
    n=len(puntos)
    tangentes=[]
    for i in range(n):
        if i==0:
            tang=(puntos[1]-puntos[0])
        elif i==n-1:
            tang=(puntos[-1]-puntos[-2])
        else:
            tang=(puntos[i+1]-puntos[i-1])/2.0
        if i<n-1:
            dist=np.linalg.norm(puntos[i+1]-puntos[i])
            if np.linalg.norm(tang)>0:
                tang= tang / np.linalg.norm(tang) * dist
        tangentes.append(tang)
    return tangentes

# Reorganizar trayectoria desde la posición actual
def reorganizar_trayectoria(puntos, pos_inicial):
    if len(puntos)==0: return puntos
    distancias=[np.linalg.norm(p-pos_inicial) for p in puntos]
    idx=distancias.index(max(distancias))
    return puntos[idx:]+puntos[:idx]

# Preparar trayectoria completa
def preparar_trayectoria_completa(puntos):
    tang=calcular_tangentes(puntos)
    segmentos=[]
    for i in range(len(puntos)-1):
        p0=puntos[i]; p3=puntos[i+1]
        segs, _ = preparar_segmento(p0,p3,tang[i],tang[i+1])
        segmentos.extend(segs)
    return segmentos, False

# Estado y registro
estado={'actual':np.array([0.0,-a1,-0.30]),'t_segmento':0.0,'indice_segmento':0,
        'siguiendo':False,'terminado':False,'segmentos':[]}
registro={'historial':[],'segmentos_reales':[]}

def puntos_pata(pos):
    q1,q2,q3=cinematica_inversa(pos[0],pos[1],pos[2],LADO_FR)
    pts=cinematica_directa(q1,q2,q3,LADO_FR)
    return [CADERA_FR+p for p in pts]

def iniciar_trayectoria():
    if len(trayectoria)<2: return
    tray_ordenada = reorganizar_trayectoria(trayectoria, estado['actual'])
    registro['historial'].clear(); registro['segmentos_reales'].clear()
    segs,_=preparar_trayectoria_completa(tray_ordenada)
    estado['segmentos']=segs
    estado['actual']=tray_ordenada[0].copy()
    estado['indice_segmento']=0
    estado['t_segmento']=0.0
    estado['siguiendo']=True
    estado['terminado']=False
    print(f"Trayectoria preparada con {len(segs)} segmentos")

def avanzar_al_siguiente():
    if estado['indice_segmento'] >= len(estado['segmentos'])-1:
        estado['siguiendo']=False
        estado['terminado']=True
        mostrar_reporte_error()
    else:
        estado['indice_segmento']+=1
        estado['t_segmento']=0.0

def mostrar_reporte_error():
    if len(registro['historial'])==0: return
    puntos_planeados=[]
    for seg in registro['segmentos_reales']:
        p0,p1,p2,p3=seg
        for t in np.linspace(0,1,20):
            puntos_planeados.append(bezier_cubico(p0,p1,p2,p3,t))
    puntos_planeados=np.array(puntos_planeados)
    errores=[]
    for P in registro['historial']:
        dists=np.linalg.norm(puntos_planeados-P,axis=1)
        errores.append(np.min(dists))
    rmse=np.sqrt(np.mean(np.array(errores)**2))
    print(f"\nRMSE (distancia perpendicular): {rmse:.5f} m\n")
    fig2,axs=plt.subplots(2,1,figsize=(9,7))
    axs[0].plot(errores,'o-',color='crimson',markersize=3)
    axs[0].set_xlabel("Frame"); axs[0].set_ylabel("Error (m)"); axs[0].grid(alpha=0.3)
    hist=np.array(registro['historial'])
    axs[1].plot(hist[:,0],hist[:,2],'-',color='crimson',label='Real')
    axs[1].plot(puntos_planeados[:,0],puntos_planeados[:,2],'--',color='dodgerblue',label='Planeado')
    axs[1].set_xlabel("X"); axs[1].set_ylabel("Z"); axs[1].legend(); axs[1].grid(alpha=0.3)
    fig2.tight_layout(); fig2.show()

# Figura y animación
fig=plt.figure(figsize=(12,8))
ax=fig.add_subplot(111,projection='3d')
ax.set_title("Bezier cúbico con espiral cónica")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
limite=0.25
ax.set_xlim(-limite*2, 0)
ax.set_ylim(-limite, limite)
ax.set_zlim(-0.50, -0.20)
ax.set_box_aspect([1,1,1])
ax.view_init(elev=30, azim=45)

# Mostrar trayectoria planeada
tray_world=np.array([CADERA_FR+p for p in trayectoria])
ax.plot(tray_world[:,0],tray_world[:,1],tray_world[:,2],'--',color='dodgerblue',linewidth=1.2,label='Waypoints')
ax.scatter(tray_world[:,0],tray_world[:,1],tray_world[:,2],color='blue',s=30)

linea_seguida,=ax.plot([],[],[],'-',color='crimson',linewidth=1.3,alpha=0.85,label='Real')
curva_bezier,=ax.plot([],[],[],'-',color='lime',linewidth=2,label='Bezier')

# Pierna inicial
O0,O1,O2,O3=puntos_pata(trayectoria[0])
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
        p0,p1,p2,p3=estado['segmentos'][idx]
        estado['actual']=bezier_cubico(p0,p1,p2,p3,u)
        registro['historial'].append(estado['actual'].copy())
        if estado['t_segmento']>=0.50:
            registro['segmentos_reales'].append((p0.copy(),p1.copy(),p2.copy(),p3.copy()))
            avanzar_al_siguiente()
    O0,O1,O2,O3=puntos_pata(estado['actual'])
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
        pts=[]
        for seg in estado['segmentos']:
            p0,p1,p2,p3=seg
            for t in np.linspace(0,1,20):
                pts.append(bezier_cubico(CADERA_FR+p0, CADERA_FR+p1, CADERA_FR+p2, CADERA_FR+p3, t))
        pts=np.array(pts)
        curva_bezier.set_data_3d(pts[:,0],pts[:,1],pts[:,2])
    else:
        curva_bezier.set_data_3d([],[],[])
    if estado['siguiendo'] or estado['terminado']:
        texto_estado.set_text(f"Segmento: {estado['indice_segmento']+1}/{len(estado['segmentos'])}\nPuntos: {len(registro['historial'])}\nEstado: {'en ruta' if estado['siguiendo'] else 'completado'}")
    return []

ani=FuncAnimation(fig,actualizar,interval=20,blit=False)
plt.tight_layout()
plt.show()