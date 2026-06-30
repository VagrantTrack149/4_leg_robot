import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

a1=0.0528; L2=0.2142; L3=0.2142
CADERA_FR=np.array([-0.29785,-0.055,0.0]); LADO_FR=-1

trayectoria=[]
radio=0.05
centro_x=0.0; centro_y=0.0; centro_z=-0.27
num_puntos=50
for t in np.linspace(0,2*np.pi,num_puntos,endpoint=False):
    x=centro_x+radio*np.cos(2*t)
    y=centro_y+radio*np.sin(3*t)
    z=centro_z+radio*np.sin(5*t)
    trayectoria.append(np.array([x,y,z]))
"""

trayectoria.append(np.array([0.0,0,-0.4]))
trayectoria.append(np.array([-0.25,0,-0.25]))

"""

ESCALON_X_MIN=-0.70; ESCALON_X_MAX=-0.40
ESCALON_Y_MIN=-0.15; ESCALON_Y_MAX=0.15
ESCALON_Z_MIN=-0.40; ESCALON_Z_MAX=-0.28

vertices=np.array([
    [ESCALON_X_MAX,ESCALON_Y_MAX,ESCALON_Z_MIN],
    [ESCALON_X_MAX,ESCALON_Y_MIN,ESCALON_Z_MIN],
    [ESCALON_X_MIN,ESCALON_Y_MIN,ESCALON_Z_MIN],
    [ESCALON_X_MIN,ESCALON_Y_MAX,ESCALON_Z_MIN],
    [ESCALON_X_MAX,ESCALON_Y_MAX,ESCALON_Z_MAX],
    [ESCALON_X_MAX,ESCALON_Y_MIN,ESCALON_Z_MAX],
    [ESCALON_X_MIN,ESCALON_Y_MIN,ESCALON_Z_MAX],
    [ESCALON_X_MIN,ESCALON_Y_MAX,ESCALON_Z_MAX],
])
caras=[[vertices[0],vertices[1],vertices[2],vertices[3]],
       [vertices[4],vertices[5],vertices[6],vertices[7]],
       [vertices[0],vertices[1],vertices[5],vertices[4]],
       [vertices[2],vertices[3],vertices[7],vertices[6]],
       [vertices[0],vertices[3],vertices[7],vertices[4]],
       [vertices[1],vertices[2],vertices[6],vertices[5]]]
cubo=Poly3DCollection(caras,alpha=0.35,facecolor='gray',edgecolor='black')

TH1_MIN,TH1_MAX=-0.6,0.5; TH2_MIN,TH2_MAX=-1.7,1.7; TH3_MIN,TH3_MAX=-0.45,1.6

def cinematica_directa(q1,q2,q3,lado):
    c1,s1=np.cos(q1),np.sin(q1); c2,s2=np.cos(q2),np.sin(q2); c23,s23=np.cos(q2+q3),np.sin(q2+q3)
    O0=np.array([0.0,0.0,0.0])
    O1=np.array([0.0,lado*a1*c1,-a1*s1])
    O2=O1+np.array([L2*s2,0.0,-L2*c2])
    O3=O2+np.array([L3*s23,0.0,-L3*c23])
    return O0,O1,O2,O3

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

def validar_angulos(q1,q2,q3):
    theta1=q1-np.pi/2.0; theta2=q2; theta3=-q3
    return (TH1_MIN<=theta1<=TH1_MAX and TH2_MIN<=theta2<=TH2_MAX and TH3_MIN<=theta3<=TH3_MAX)

def punto_dentro_bloque(P):
    return (ESCALON_X_MIN<=P[0]<=ESCALON_X_MAX and ESCALON_Y_MIN<=P[1]<=ESCALON_Y_MAX and ESCALON_Z_MIN<=P[2]<=ESCALON_Z_MAX)

def segmento_colisiona(A,B,muestras=50):
    for t in np.linspace(0,1,muestras):
        if punto_dentro_bloque(A+t*(B-A)): return True
    return False

def bezier_cubico(p0,p1,p2,p3,t):
    u=1-t
    return u**3*p0+3*u**2*t*p1+3*u*t**2*p2+t**3*p3

def colision_bezier(p0,p1,p2,p3,muestras=80):
    for t in np.linspace(0,1,muestras):
        if punto_dentro_bloque(bezier_cubico(p0,p1,p2,p3,t)): return True
    return False

def curva_valida(p0,p1,p2,p3,muestras=50):
    for t in np.linspace(0,1,muestras):
        pt=bezier_cubico(p0,p1,p2,p3,t)
        q1,q2,q3=cinematica_inversa(pt[0],pt[1],pt[2],LADO_FR)
        if not validar_angulos(q1,q2,q3): return False
    return True

ELEVACION_MAX=0.35; PASO_ELEV=0.025; DESPLAZAMIENTO_X_MAX=0.20; PASO_X=0.025

def preparar_segmento(p0,p3,tang_in,tang_out):
    factor_curv=1.5
    p1_base=p0 + (1.0/3.0)*tang_in*factor_curv
    p2_base=p3 - (1.0/3.0)*tang_out*factor_curv
    estrategias=[]
    for elev in np.arange(0, ELEVACION_MAX+PASO_ELEV, PASO_ELEV):
        for xoff in np.arange(0, DESPLAZAMIENTO_X_MAX+PASO_X, PASO_X):
            estrategias.append((elev,xoff,1.0))
    for factor in [1.5,2.0,2.5]:
        for elev in np.arange(0, ELEVACION_MAX*0.8+PASO_ELEV, PASO_ELEV):
            for xoff in np.arange(0, DESPLAZAMIENTO_X_MAX*0.8+PASO_X, PASO_X):
                estrategias.append((elev,xoff,factor))
    mejor_elev=0.0; mejor_xoff=0.0; mejor_factor=1.0; encontrado=False
    for elev,xoff,factor in estrategias:
        p1=p1_base.copy(); p2=p2_base.copy()
        signo=1 if p0[0] > p3[0] else -1
        p1[0]+=signo*xoff; p2[0]+=signo*xoff
        p1[2]+=elev; p2[2]+=elev
        if factor>1.0:
            p1=p0 + (1.0/3.0)*tang_in*factor_curv*factor
            p2=p3 - (1.0/3.0)*tang_out*factor_curv*factor
            p1[0]+=signo*xoff; p2[0]+=signo*xoff
            p1[2]+=elev; p2[2]+=elev
        if curva_valida(p0,p1,p2,p3) and not colision_bezier(p0,p1,p2,p3):
            mejor_elev=elev; mejor_xoff=xoff; mejor_factor=factor; encontrado=True
            break
    if encontrado:
        print(f"Segmento ajustado: elev={mejor_elev:.3f}, xoff={mejor_xoff:.3f}, factor={mejor_factor:.1f}")
        return p1,p2,False
    else:
        print("Segmento BLOQUEADO")
        return p1_base,p2_base,True

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

def preparar_trayectoria_completa(puntos):
    tang=calcular_tangentes(puntos)
    segmentos=[]
    bloqueado=False
    for i in range(len(puntos)-1):
        p0=puntos[i]; p3=puntos[i+1]
        p1,p2,bloq=preparar_segmento(p0,p3,tang[i],tang[i+1])
        segmentos.append((p0,p1,p2,p3))
        if bloq: bloqueado=True
    return segmentos,bloqueado

estado={'actual':np.array([0.0,-a1,-0.30]),'t_segmento':0.0,'indice_segmento':0,
        'siguiendo':False,'terminado':False,'bloqueado':False,'segmentos':[]}
registro={'historial':[],'segmentos_reales':[]}

def puntos_pata(pos):
    q1,q2,q3=cinematica_inversa(pos[0],pos[1],pos[2],LADO_FR)
    pts=cinematica_directa(q1,q2,q3,LADO_FR)
    return [CADERA_FR+p for p in pts]

def iniciar_trayectoria():
    if len(trayectoria)<2: return
    registro['historial'].clear(); registro['segmentos_reales'].clear()
    segs,bloq=preparar_trayectoria_completa(trayectoria)
    estado['segmentos']=segs
    estado['bloqueado']=bloq
    estado['actual']=trayectoria[0].copy()
    estado['indice_segmento']=0
    estado['t_segmento']=0.0
    estado['siguiendo']=True
    estado['terminado']=False
    print(f"Trayectoria preparada con {len(segs)} segmentos, bloqueado={bloq}")

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

fig=plt.figure(figsize=(12,8))
ax=fig.add_subplot(111,projection='3d')
ax.set_title("Bezier cúbico con planificación global y evasión de colisiones")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_xlim(-0.8,0.1); ax.set_ylim(-0.350,0.035); ax.set_zlim(-0.50,-0.08)
ax.set_box_aspect([1,1,1]); ax.view_init(elev=25,azim=145)
#ax.add_collection3d(cubo)
tray_world=np.array([CADERA_FR+p for p in trayectoria])
ax.plot(tray_world[:,0],tray_world[:,1],tray_world[:,2],'--',color='dodgerblue',linewidth=1.2,label='Waypoints')
ax.scatter(tray_world[:,0],tray_world[:,1],tray_world[:,2],color='blue',s=30)
linea_seguida,=ax.plot([],[],[],'-',color='crimson',linewidth=1.3,alpha=0.85,label='Real')
curva_bezier,=ax.plot([],[],[],'-',color='lime',linewidth=2,label='Bezier')
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
    col=segmento_colisiona(O1,O2) or segmento_colisiona(O2,O3)
    color='orange' if col else 'red'
    l01.set_data_3d([O0[0],O1[0]],[O0[1],O1[1]],[O0[2],O1[2]])
    l12.set_data_3d([O1[0],O2[0]],[O1[1],O2[1]],[O1[2],O2[2]])
    l23.set_data_3d([O2[0],O3[0]],[O2[1],O3[1]],[O2[2],O3[2]])
    l12.set_color(color); l23.set_color(color)
    articulaciones._offsets3d=([O0[0],O1[0],O2[0],O3[0]],[O0[1],O1[1],O2[1],O3[1]],[O0[2],O1[2],O2[2],O3[2]])
    pie.set_data_3d([O3[0]],[O3[1]],[O3[2]])
    if len(registro['historial'])>1:
        hist=np.array(registro['historial'])
        hist_world=CADERA_FR+hist
        linea_seguida.set_data_3d(hist_world[:,0],hist_world[:,1],hist_world[:,2])
    if estado['siguiendo']:
        pts=[]
        for seg in estado['segmentos']:
            p0,p1,p2,p3=seg
            for t in np.linspace(0,1,20):
                pts.append(bezier_cubico(CADERA_FR+p0, CADERA_FR+p1, CADERA_FR+p2, CADERA_FR+p3, t))
        pts=np.array(pts)
        curva_bezier.set_data_3d(pts[:,0],pts[:,1],pts[:,2])
        curva_bezier.set_color('red' if estado['bloqueado'] else 'lime')
    else:
        curva_bezier.set_data_3d([],[],[])
    if estado['siguiendo'] or estado['terminado']:
        texto_estado.set_text(f"Segmento: {estado['indice_segmento']+1}/{len(estado['segmentos'])}\nPuntos registrados: {len(registro['historial'])}\nEstado: {'en ruta' if estado['siguiendo'] else 'completado'}\nBloqueado: {'SI' if estado['bloqueado'] else 'NO'}")
    return []

ani=FuncAnimation(fig,actualizar,interval=20,blit=False)
plt.tight_layout()
plt.show()