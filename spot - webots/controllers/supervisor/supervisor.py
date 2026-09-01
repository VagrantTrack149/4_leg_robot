from controller import Supervisor
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Parámetros de la cinemática
a1 = 0.0528            
L2 = 0.2142
L3 = 0.2142

ALTURA_NEUTRA = -0.42  
OFFSET_Z = -0.5

# Duración de la simulación
MODO_INFINITO = False
DURACION_PASADA = 16.0 

# SELECCIÓN DE TRAYECTORIA ÚNICA
TIPO_SELECCIONADO = 'espiral'  # 'escalon', 'lissajous', 'espiral'

# VARIABLES DE OFFSET PARA PRUEBAS Y CALIBRACIÓN DE TRAYECTORIA
OFFSET_PRUEBA_X = 0.0   
OFFSET_PRUEBA_Y = 0.0   
OFFSET_PRUEBA_Z = OFFSET_Z   

FOOT_DEF_NAME_FL = None  


# Punto de montaje de cada pata = traslación del nodo SpotLeg en Spot.proto:
#   DEF FRONT_..._LEG SpotLeg { translation  0.3635  0  0.0118 ... }
#   DEF REAR_..._LEG  SpotLeg { translation -0.3084  0  0.0117 ... }
MONTAJE_PATA = {
    'FR': np.array([ 0.3635, 0.0, 0.0118]),
    'FL': np.array([ 0.3635, 0.0, 0.0118]),
    'RR': np.array([-0.3084, 0.0, 0.0117]),
    'RL': np.array([-0.3084, 0.0, 0.0117]),
}

LADO = {
    'RR': -1,
    'RL': +1,
    'FR': -1,
    'FL': +1
}


# Spot.proto rota cada SpotLeg 120° sobre el eje (1,-1,-1)/sqrt(3)
# ("rotation 0.577350 -0.577354 -0.577346 2.094389", igual en las 4 patas).
# Esa rotación reorienta los ejes locales del PROTO de la pata a la
# convención (X adelante, Y izquierda, Z arriba) que ya usan
# cinematica_directa/cinematica_inversa, por lo que px,py,pz salen
# directamente alineados al cuerpo del robot (no hace falta reaplicar
# ninguna rotación extra, solo la traslación cuerpo->hombro).
#
# Dentro de SpotLeg.proto, el HingeJoint de abducción (el origen real de la
# cinemática) está anclado en (sign*a1, 0.0006, 0) en el frame LOCAL de la
# pata, con sign = -1 si left=TRUE, +1 si left=FALSE:
#   HingeJoint { ... anchor %<=sign * 0.0528>% 0.0006 0 }
# Al pasar ese anchor por la rotación de 120° de Spot.proto, el desplazamiento
# del hombro respecto al punto de montaje queda (0, lado*a1, 0.0006).
OFFSET_HOMBRO_LOCAL_Z = 0.0006  # componente Z del anchor de abducción del proto

def matriz_hombro(pata):
    lado = LADO[pata]
    hombro_en_cuerpo = MONTAJE_PATA[pata] + np.array([0.0, lado * a1, OFFSET_HOMBRO_LOCAL_Z])
    T = np.eye(4)
    T[0:3, 3] = hombro_en_cuerpo
    return T

# Se conserva el nombre CADERAS (usado en el resto del script/plot 3D) pero
# ahora se calcula a partir de matriz_hombro en vez de valores hardcodeados.
CADERAS = {p: matriz_hombro(p)[0:3, 3] for p in ['FR', 'FL', 'RR', 'RL']}

motors = {
    "FL": [
        robot.getDevice("front left shoulder abduction motor"),
        robot.getDevice("front left shoulder rotation motor"),
        robot.getDevice("front left elbow motor")
    ],
    "FR": [
        robot.getDevice("front right shoulder abduction motor"),
        robot.getDevice("front right shoulder rotation motor"),
        robot.getDevice("front right elbow motor")
    ],
    "RL": [
        robot.getDevice("rear left shoulder abduction motor"),
        robot.getDevice("rear left shoulder rotation motor"),
        robot.getDevice("rear left elbow motor")
    ],
    "RR": [
        robot.getDevice("rear right shoulder abduction motor"),
        robot.getDevice("rear right shoulder rotation motor"),
        robot.getDevice("rear right elbow motor")
    ]
}

sensors = {}
for pata in motors:
    sensors[pata] = []
    for motor in motors[pata]:
        motor.setVelocity(10.0)
        s = motor.getPositionSensor()
        if s is None:
            s = robot.getDevice(motor.getName() + " sensor")
        s.enable(timestep)
        sensors[pata].append(s)

def cinematica_directa(q1, q2, q3, lado):
    c1, s1 = np.cos(q1), np.sin(q1)
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)

    O0 = np.array([0.0, 0.0, 0.0])
    O1 = np.array([0.0, lado * a1 * c1, -a1 * s1])
    O2 = O1 + np.array([L2 * s2, 0.0, -L2 * c2])       
    O3 = O2 + np.array([L3 * s23, 0.0, -L3 * c23])   

    px = O3[0]
    py = O3[1] + lado * a1
    pz = O3[2]

    return px, py, pz, (O0, O1, O2, O3)

def cinematica_inversa(px, py, pz, lado):
    py_l = py * lado
    ratio = np.clip(py_l / a1, -1.0, 1.0)
    q1 = np.arccos(ratio)

    O1_z = -a1 * np.sin(q1)
    dx = np.clip(px, -0.15, 0.15)
    dz = np.clip(pz - O1_z, -(L2 + L3 - 0.01), -0.05)

    R = np.hypot(dx, dz)
    R = min(R, L2 + L3 - 1e-4)

    C3 = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    C3 = np.clip(C3, -1.0, 1.0)

    q3 = np.arctan2(np.sqrt(1.0 - C3**2), C3)  
    q3 = np.clip(q3, -1.5, 1.5)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)

    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    return q1, q2, q3

def calcular_rmse(real_val, plan_val):
    diff = np.array(real_val) - np.array(plan_val)
    return np.sqrt(np.mean(diff ** 2))

self_node = robot.getSelf()

for pata in ["FL", "FR", "RL", "RR"]:
    lado = LADO[pata]
    px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA
    q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
    motors[pata][0].setPosition(q1)
    motors[pata][1].setPosition(q2)
    motors[pata][2].setPosition(q3)

plt.ion()

fig_3d = plt.figure(figsize=(10, 8))
ax = fig_3d.add_subplot(111, projection='3d')
ax.set_title(f"Robot Spot - Trayectoria Seleccionada: {TIPO_SELECCIONADO.upper()}")

ax.set_xlabel("X (adelante)")
ax.set_ylabel("Y (izquierda)")
ax.set_zlabel("Z (arriba)")

ax.set_xlim(-0.4, 0.4)
ax.set_ylim(-0.4, 0.4)
ax.set_zlim(-1.5, 0.5)

lineas_patas = {}
colores_patas = {'FL': 'orange', 'FR': 'blue', 'RL': 'green', 'RR': 'brown'}
for pata in ["FL", "FR", "RL", "RR"]:
    hombro, = ax.plot([], [], [], 'o-', color=colores_patas[pata], linewidth=2, markersize=4, label=f'Hombro {pata}')
    pierna, = ax.plot([], [], [], 'o-', color=colores_patas[pata], linewidth=3, markersize=6, label=f'Pierna {pata}')
    lineas_patas[pata] = (hombro, pierna)

tray_real, = ax.plot([], [], [], 'r-', linewidth=1.5, label='Trayectoria Real FL')
tray_planeada, = ax.plot([], [], [], 'k--', linewidth=1, label='Planeada FL')
ax.legend(loc='upper left', fontsize=8)

fig_rmse = plt.figure(figsize=(10, 5))
ax_rmse = fig_rmse.add_subplot(111)
ax_rmse.set_title("Evolución del Error RMSE en el Tiempo")
ax_rmse.set_xlabel("Tiempo (s)")
ax_rmse.set_ylabel("RMSE (mm)")
line_rmse, = ax_rmse.plot([], [], 'g-', linewidth=2, label='RMSE (mm)')
ax_rmse.legend(loc='upper right')
ax_rmse.grid(True)

hist_x, hist_y, hist_z = [], [], []
hist_plan_x, hist_plan_y, hist_plan_z = [], [], []
hist_tiempos_rmse = []
hist_valores_rmse = []
hist_real_all = []
hist_plan_all = []

root_children = None
curva_coord_field = None
curva_index_field = None
foot_node_FL = None
TRAIL_EVERY = 2
TRAIL_MAX_PUNTOS = 400
TRAIL_DIST_MIN = 0.001
num_puntos_curva = 0
ultimo_punto_curva = None

def crear_curva_pie(punto_inicial):
    vrml = (
        "Shape {{ appearance Appearance {{ material Material {{ "
        "diffuseColor 1 0.7 0 emissiveColor 1 0.4 0 }} }} "
        "geometry DEF FOOT_TRAIL_LINE IndexedLineSet {{ "
        "coord DEF FOOT_TRAIL_COORD Coordinate {{ point [ {0} {1} {2}, {0} {1} {2} ] }} "
        "coordIndex [ 0, 1, -1 ] }} }}"
    ).format(punto_inicial[0], punto_inicial[1], punto_inicial[2])
    root_children.importMFNodeFromString(-1, vrml)
    coord_field = robot.getFromDef("FOOT_TRAIL_COORD").getField("point")
    index_field = robot.getFromDef("FOOT_TRAIL_LINE").getField("coordIndex")
    return coord_field, index_field

def agregar_punto_curva(pos_mundo):
    global curva_coord_field, curva_index_field, ultimo_punto_curva, num_puntos_curva
    if curva_coord_field is None:
        curva_coord_field, curva_index_field = crear_curva_pie(pos_mundo)
        ultimo_punto_curva = np.array(pos_mundo)
        num_puntos_curva = 2
        return
    if num_puntos_curva >= TRAIL_MAX_PUNTOS:
        return
    if np.linalg.norm(np.array(pos_mundo) - ultimo_punto_curva) < TRAIL_DIST_MIN:
        return
    curva_coord_field.insertMFVec3f(-1, list(pos_mundo))
    nuevo_indice = num_puntos_curva
    curva_index_field.removeMF(curva_index_field.getCount() - 1)
    curva_index_field.insertMFInt32(-1, nuevo_indice)
    curva_index_field.insertMFInt32(-1, -1)
    ultimo_punto_curva = np.array(pos_mundo)
    num_puntos_curva += 1

def buscar_nodo_por_nombre(nodo, palabras_clave, profundidad=0, max_profundidad=15):
    if nodo is None or profundidad > max_profundidad:
        return None
    try:
        campo_nombre = nodo.getField("name")
        if campo_nombre is not None:
            nombre = campo_nombre.getSFString()
            if nombre and any(k.lower() in nombre.lower() for k in palabras_clave):
                return nodo
    except Exception:
        pass
    try:
        campo_endpoint = nodo.getField("endPoint")
        if campo_endpoint is not None:
            hijo = campo_endpoint.getSFNode()
            resultado = buscar_nodo_por_nombre(hijo, palabras_clave, profundidad + 1, max_profundidad)
            if resultado is not None:
                return resultado
    except Exception:
        pass
    try:
        campo_hijos = nodo.getField("children")
        if campo_hijos is not None:
            n = campo_hijos.getCount()
            for i in range(n):
                hijo = campo_hijos.getMFNode(i)
                resultado = buscar_nodo_por_nombre(hijo, palabras_clave, profundidad + 1, max_profundidad)
                if resultado is not None:
                    return resultado
    except Exception:
        pass
    return None

def calcular_punto_pie_mundo(px_real, py_real, pz_real, pata):
    if pata == "FL" and foot_node_FL is not None:
        pos_base = np.array(foot_node_FL.getPosition())
        return pos_base + np.array([OFFSET_PRUEBA_X, OFFSET_PRUEBA_Y, OFFSET_PRUEBA_Z])

    pos_cuerpo = np.array(self_node.getPosition())
    R_cuerpo = np.array(self_node.getOrientation()).reshape(3, 3)

    # Coordenadas del pie en el frame del hombro (px,py,pz de la cinemática),
    # con los offsets de prueba/calibración aplicados igual que antes.
    p_pie_hombro = np.array([
        px_real + OFFSET_PRUEBA_X,
        py_real + OFFSET_PRUEBA_Y,
        (pz_real - ALTURA_NEUTRA) + OFFSET_PRUEBA_Z,
        1.0
    ])

    # hombro -> cuerpo (matriz derivada del proto) -> mundo (pose real de self_node)
    p_relativa_cuerpo = (matriz_hombro(pata) @ p_pie_hombro)[0:3]
    pos_mundo = pos_cuerpo + R_cuerpo.dot(p_relativa_cuerpo)
    return pos_mundo

def obtener_punto_vectorial(t_val, tipo, lado):
    t_mod = t_val % 4.0
    if tipo == 'escalon':
        py = lado * a1 + 0.08
        tercio = 4.0 / 3.0
        if t_mod < tercio:
            px = -0.05
            pz = ALTURA_NEUTRA
        elif t_mod < 2 * tercio:
            px = 0.0
            pz = ALTURA_NEUTRA + 0.05
        else:
            px = 0.05
            pz = ALTURA_NEUTRA + 0.05
    elif tipo == 'lissajous':
        px = 0.04 * np.sin(2 * np.pi * t_val)
        py = lado * a1 + 0.16 * np.sin(3 * np.pi * t_val)  
        pz = ALTURA_NEUTRA + 0.02 * np.sin(5 * np.pi * t_val)
    else:  
        vueltas = 4
        altura = 0.05
        s = t_mod / 4.0
        theta = 2 * np.pi * vueltas * s
        radio = 0.02 + 0.03 * s
        px = radio * np.cos(theta)
        py = lado * a1 + (radio * 4.0) * np.sin(theta)  
        pz = ALTURA_NEUTRA - altura * s
    return px, py, pz

t = 0.0
step_count = 0
DRAW_EVERY = 5
RMSE_EVERY = 20
MUESTREO_ERROR_EVERY = 1

px_real_FL = py_real_FL = pz_real_FL = 0.0
px_plan_FL = py_plan_FL = pz_plan_FL = 0.0

while robot.step(timestep) != -1:
    t += timestep / 1000.0

    if not MODO_INFINITO and t >= DURACION_PASADA:
        break

    if root_children is None:
        root_children = robot.getRoot().getField("children")
        self_node = robot.getSelf()
        if FOOT_DEF_NAME_FL is not None:
            foot_node_FL = robot.getFromDef(FOOT_DEF_NAME_FL)
        else:
            foot_node_FL = buscar_nodo_por_nombre(self_node, ["foot", "toe", "pie", "tip", "ankle", "fl_"])

    for pata in ["FL", "FR", "RL", "RR"]:
        lado = LADO[pata]

        if pata == "FL":
            px, py, pz = obtener_punto_vectorial(t, TIPO_SELECCIONADO, lado)
            px_plan_FL, py_plan_FL, pz_plan_FL = px, py, pz
        else:
            px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA

        q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
        motors[pata][0].setPosition(q1)
        motors[pata][1].setPosition(q2)
        motors[pata][2].setPosition(q3)

        if pata == "FL":
            q1_real = sensors["FL"][0].getValue()
            q2_real = sensors["FL"][1].getValue()
            q3_real = sensors["FL"][2].getValue()
            px_real, py_real, pz_real, _ = cinematica_directa(q1_real, q2_real, q3_real, lado)
            px_real_FL, py_real_FL, pz_real_FL = px_real, py_real, pz_real
            
            if step_count % TRAIL_EVERY == 0:
                pos_mundo = calcular_punto_pie_mundo(px_real, py_real, pz_real, pata)
                agregar_punto_curva(pos_mundo)

    if step_count % MUESTREO_ERROR_EVERY == 0:
        pos_cuerpo = np.array(self_node.getPosition())
        R_cuerpo = np.array(self_node.getOrientation()).reshape(3, 3)
        
        T_hombro_FL = matriz_hombro('FL')

        p_real_hombro = np.array([
            px_real_FL + OFFSET_PRUEBA_X,
            py_real_FL + OFFSET_PRUEBA_Y,
            (pz_real_FL - ALTURA_NEUTRA) + OFFSET_PRUEBA_Z,
            1.0
        ])
        pos_mundo_real = pos_cuerpo + R_cuerpo.dot((T_hombro_FL @ p_real_hombro)[0:3])

        p_plan_hombro = np.array([
            px_plan_FL + OFFSET_PRUEBA_X,
            py_plan_FL + OFFSET_PRUEBA_Y,
            (pz_plan_FL - ALTURA_NEUTRA) + OFFSET_PRUEBA_Z,
            1.0
        ])
        pos_mundo_plan = pos_cuerpo + R_cuerpo.dot((T_hombro_FL @ p_plan_hombro)[0:3])

        hist_x.append(pos_mundo_real[0])
        hist_y.append(pos_mundo_real[1])
        hist_z.append(pos_mundo_real[2])
        
        hist_plan_x.append(pos_mundo_plan[0])
        hist_plan_y.append(pos_mundo_plan[1])
        hist_plan_z.append(pos_mundo_plan[2])

        if len(hist_x) > 150:
            hist_x.pop(0); hist_y.pop(0); hist_z.pop(0)
            hist_plan_x.pop(0); hist_plan_y.pop(0); hist_plan_z.pop(0)

        hist_real_all.append([px_real_FL, py_real_FL, pz_real_FL])
        hist_plan_all.append([px_plan_FL, py_plan_FL, pz_plan_FL])

        if step_count % RMSE_EVERY == 0:
            rmse_actual = calcular_rmse(hist_real_all, hist_plan_all)
            val_mm = rmse_actual * 1000
            print(f"[t={t:.2f}s | Tipo: {TIPO_SELECCIONADO}] RMSE: {val_mm:.4f} mm")
            hist_tiempos_rmse.append(t)
            hist_valores_rmse.append(val_mm)

    if step_count % DRAW_EVERY == 0 and step_count > 0:
        for pata in ["FL", "FR", "RL", "RR"]:
            lado = LADO[pata]
            q1_r = sensors[pata][0].getValue()
            q2_r = sensors[pata][1].getValue()
            q3_r = sensors[pata][2].getValue()

            _, _, _, (O0, O1, O2, O3) = cinematica_directa(q1_r, q2_r, q3_r, lado)
            O0 = CADERAS[pata] + O0
            O1 = CADERAS[pata] + O1
            O2 = CADERAS[pata] + O2
            O3 = CADERAS[pata] + O3

            linea_hombro, linea_pierna = lineas_patas[pata]
            linea_hombro.set_data([O0[0], O1[0]], [O0[1], O1[1]])
            linea_hombro.set_3d_properties([O0[2], O1[2]])

            linea_pierna.set_data([O1[0], O2[0], O3[0]], [O1[1], O2[1], O3[1]])
            linea_pierna.set_3d_properties([O1[2], O2[2], O3[2]])

        tray_real.set_data(hist_x, hist_y)
        tray_real.set_3d_properties(hist_z)
        tray_planeada.set_data(hist_plan_x, hist_plan_y)
        tray_planeada.set_3d_properties(hist_plan_z)

        if len(hist_tiempos_rmse) > 0:
            line_rmse.set_data(hist_tiempos_rmse, hist_valores_rmse)
            ax_rmse.relim()
            ax_rmse.autoscale_view()

        fig_3d.canvas.draw_idle()
        fig_3d.canvas.flush_events()
        fig_rmse.canvas.draw_idle()
        fig_rmse.canvas.flush_events()

    step_count += 1

plt.ioff()
plt.show()