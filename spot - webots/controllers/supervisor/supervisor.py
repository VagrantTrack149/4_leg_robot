from controller import Supervisor
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Parametros
a1 = 0.08       # Offset lateral de cadera (L_hip)
L2 = 0.34       # Largo del muslo (L_thigh)
L3 = 0.35       # Largo de la pantorrilla (L_calf)
ALTURA_NEUTRA = -0.7   # altura de la pata en reposo

# duración de la simulación
MODO_INFINITO = False
DURACION_PASADA = 10.0   # segundos

# Offsets geométricos de la pata FL
LARGO_ROBOT = 0.8
OFFSET_X_CUERPO = LARGO_ROBOT
OFFSET_Y_HOMBRO = a1*2
OFFSET_Z_HOMBRO = -0.5      
AUTO_CALIBRAR_OFFSETS = True
#
#Se tiene que modificar el offset geometrico de cada trayectoria de la pata

ESCALA_TRAYECTORIA_VISUAL = 1

PALABRAS_CLAVE_PIE_FL = ["foot", "toe", "pie", "tip", "ankle", "fl_"]

FOOT_DEF_NAME_FL = None  

# CONFIGURACIÓN DE PATAS
LADO = {
    'RR': -1,
    'RL':  1,
    'FR': -1,
    'FL':  1
}

# MOTORES
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

# Sensores
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

# CINEMÁTICA INVERSA
def cinematica_inversa(px, py, pz, lado):
    x = px
    y = py - lado * a1
    z = pz
    q1 = np.arctan2(y, -z)
    L = np.hypot(y, z)
    d = np.hypot(x, L)
    cos_knee = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    q3 = np.pi - np.arccos(cos_knee)
    alpha = np.arctan2(x, L)
    beta = np.arccos(np.clip((L2**2 + d**2 - L3**2) / (2 * L2 * d), -1.0, 1.0))
    q2 = alpha + beta
    return q1, q2, q3

# CINEMÁTICA DIRECTA
def cinematica_directa(q1, q2, q3, lado):
    d = np.sqrt(max(L2**2 + L3**2 + 2 * L2 * L3 * np.cos(q3), 0.0))
    cos_beta = np.clip((L2**2 + d**2 - L3**2) / (2 * L2 * d + 1e-12), -1.0, 1.0)
    beta = np.arccos(cos_beta)
    alpha = q2 - beta
    x = d * np.sin(alpha)
    L = d * np.cos(alpha)
    y = L * np.sin(q1)
    z = -L * np.cos(q1)
    px = x
    py = y + lado * a1
    pz = z
    return px, py, pz

# RMSE
def calcular_rmse(hist_real, hist_planeada):
    if len(hist_real) == 0 or len(hist_planeada) == 0:
        return None
    real = np.array(hist_real)
    plan = np.array(hist_planeada)
    n = min(len(real), len(plan))
    real, plan = real[:n], plan[:n]
    return np.sqrt(np.mean(np.sum((real - plan) ** 2, axis=1)))

def calibrar_offsets_hombro():
    global OFFSET_X_CUERPO, OFFSET_Y_HOMBRO, OFFSET_Z_HOMBRO
    try:
        nodo_cadera_fl = robot.getFromDevice(motors["FL"][0])
        if nodo_cadera_fl is None:
            raise RuntimeError("getFromDevice(FL abduction) devolvió None")
        pos_cadera = np.array(nodo_cadera_fl.getPosition())
        pos_cuerpo = np.array(self_node.getPosition())
        R = np.array(self_node.getOrientation()).reshape(3, 3)
        local = R.T.dot(pos_cadera - pos_cuerpo)
        if abs(local[0]) < 1e-4 and abs(local[1]) < 1e-4:
            raise RuntimeError("posición local nula/no válida")
        OFFSET_X_CUERPO = local[0]   # con signo
        OFFSET_Y_HOMBRO = local[1]   # con signo
        OFFSET_Z_HOMBRO = local[2]   # con signo nuevo
        print(f"[calibración] Offsets reales medidos (con signo): "
              f"X={OFFSET_X_CUERPO:.4f} m, Y={OFFSET_Y_HOMBRO:.4f} m, "
              f"Z={OFFSET_Z_HOMBRO:.4f} m")
    except Exception as e:
        print(f"[calibración] No se pudo medir automáticamente ({e}). "
              f"Se usan valores de respaldo: X={OFFSET_X_CUERPO:.4f} m, "
              f"Y={OFFSET_Y_HOMBRO:.4f} m, Z={OFFSET_Z_HOMBRO:.4f} m.")

# Postura inicial
for pata in ["FL", "FR", "RL", "RR"]:
    lado = LADO[pata]
    px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA
    q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
    motors[pata][0].setPosition(q1)
    motors[pata][1].setPosition(q2)
    motors[pata][2].setPosition(q3)

# Configuración de matplotlib (gráfica en tiempo real)
plt.ion()
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Robot Spot - Cinemática Pata FL")
ax.set_xlabel("X (adelante)"); ax.set_ylabel("Y (izquierda)"); ax.set_zlabel("Z (arriba)")
ax.set_xlim(-0.3, 0.3); ax.set_ylim(-0.3, 0.3); ax.set_zlim(-0.9, -0.1)

line_hombro, = ax.plot([], [], [], 'o-', color='purple', linewidth=3, markersize=6, label='Hombro')
line_pierna, = ax.plot([], [], [], 'o-', color='orange', linewidth=4, markersize=8, label='Pierna')
tray_real, = ax.plot([], [], [], 'r-', linewidth=1.5, label='Trayectoria Real')
tray_planeada, = ax.plot([], [], [], 'k--', linewidth=1, label='Planeada')
ax.legend()
plt.tight_layout()

# Historiales para la ventana deslizante 
hist_x, hist_y, hist_z = [], [], []
hist_plan_x, hist_plan_y, hist_plan_z = [], [], []

# Historial completo para RMSE final
hist_full_t = []
hist_full_real = []
hist_full_plan = []

# Variables para la estela en Webots (IndexedLineSet)
root_children = None
self_node = None
curva_coord_field = None
curva_index_field = None
foot_node_FL = None
TRAIL_EVERY = 2
TRAIL_MAX_PUNTOS = 300
TRAIL_DIST_MIN = 0.001
num_puntos_curva = 0
ultimo_punto_curva = None

def crear_curva_pie(punto_inicial):
    vrml = (
        "Shape {{ appearance Appearance {{ material Material {{ "
        "diffuseColor 2 1.4 0 emissiveColor 1 0.4 0 }} }} "
        "geometry DEF FOOT_TRAIL_LINE IndexedLineSet {{ "
        "coord DEF FOOT_TRAIL_COORD Coordinate {{ point [ {0} {1} {2} ] }} "
        "coordIndex [ 0 -1 ] }} }}"
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
        num_puntos_curva = 1
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

def punto_local_a_mundo(local_pos):
    pos = np.array(self_node.getPosition())
    R = np.array(self_node.getOrientation()).reshape(3, 3)
    return pos + R.dot(np.array(local_pos))

#busqueda pie
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

    # Si es un joint (HingeJoint, Hinge2Joint, SliderJoint), bajar por endPoint
    try:
        campo_endpoint = nodo.getField("endPoint")
        if campo_endpoint is not None:
            hijo = campo_endpoint.getSFNode()
            resultado = buscar_nodo_por_nombre(hijo, palabras_clave, profundidad + 1, max_profundidad)
            if resultado is not None:
                return resultado
    except Exception:
        pass

    # Si tiene "children" (Solid, Transform, Group, Robot), recorrer cada hijo
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

def calcular_punto_pie_mundo(px_real, py_real, pz_real, lado):
    if foot_node_FL is not None:
        # Camino robusto: posición física real del nodo del pie en el mundo
        return np.array(foot_node_FL.getPosition())

    # Desviaciones respecto al punto neutro de la pata
    x_relativo = px_real - 0.0
    y_relativo = py_real - lado * a1
    z_relativo = pz_real - ALTURA_NEUTRA

    punto_cuerpo = [
        OFFSET_X_CUERPO + x_relativo * ESCALA_TRAYECTORIA_VISUAL,
        OFFSET_Y_HOMBRO + y_relativo * ESCALA_TRAYECTORIA_VISUAL,
        OFFSET_Z_HOMBRO + z_relativo * ESCALA_TRAYECTORIA_VISUAL,
    ]
    return punto_local_a_mundo(punto_cuerpo)

# Parámetros de trayectoria
OFFSET_X = -0.25
OFFSET_Y = 0.0
OFFSET_Z = 0.20
TIPO = 'espiral'  # 'escalon', 'lissajous', o 'n' (espiral)

def obtener_punto(t, tipo, lado):
    t_mod = t % 4.0
    if tipo == 'escalon':

        py = lado * a1 + 0.05
        tercio = 4.0 / 3.0
        if t_mod < tercio:
            # Punto 1: bajo, atrás
            px = -0.1
            pz = ALTURA_NEUTRA
        elif t_mod < 2 * tercio:
            # Punto 2: sube al escalón, centro
            px = 0.0
            pz = ALTURA_NEUTRA + 0.1
        else:
            # Punto 3: se mantiene arriba, avanza
            px = 0.1
            pz = ALTURA_NEUTRA + 0.1
    elif tipo == 'lissajous':
        px = 0.08 * np.sin(2 * np.pi * t)
        py = lado * a1 + 0.08 * np.sin(3 * np.pi * t)
        pz = ALTURA_NEUTRA + 0.04 * np.sin(5 * np.pi * t)
    else:  # espiral
        
        vueltas = 4
        altura = 0.10
        s = t_mod / 4.0
        theta = 2 * np.pi * vueltas * s
        radio = 0.03 + 0.05 * s
        px = OFFSET_X + radio * np.cos(theta)
        py = lado * a1 + OFFSET_Y + radio * np.sin(theta)
        pz = ALTURA_NEUTRA + OFFSET_Z - altura * s
    return px, py, pz

# Bucle principal
t = 0.0
step_count = 0
DRAW_EVERY = 5      # cada cuánto se redibuja la gráfica 3D en vivo
RMSE_EVERY = 20      # cada cuánto se imprime el RMSE acumulado en consola
MUESTREO_ERROR_EVERY = 1  # cada cuánto se guarda el error instantáneo para la gráfica rmse

# Variables para almacenar la última posición real y planeada de FL
px_real_FL = py_real_FL = pz_real_FL = 0.0
px_plan_FL = py_plan_FL = pz_plan_FL = 0.0

while robot.step(timestep) != -1:
    t += timestep / 1000.0

    if not MODO_INFINITO and t >= DURACION_PASADA:
        break

    # Inicialización de Supervisor
    if root_children is None:
        root_children = robot.getRoot().getField("children")
        self_node = robot.getSelf()
        if AUTO_CALIBRAR_OFFSETS:
            calibrar_offsets_hombro()

        # Localizar el nodo físico del pie FL: primero por DEF explícito
        if FOOT_DEF_NAME_FL is not None:
            foot_node_FL = robot.getFromDef(FOOT_DEF_NAME_FL)
            if foot_node_FL is not None:
                print(f"[trail] Usando nodo con DEF '{FOOT_DEF_NAME_FL}' para el pie FL.")
            else:
                print(f"[trail] ERROR: no existe DEF '{FOOT_DEF_NAME_FL}' en el .wbt. "
                      f"Cayendo a búsqueda automática por nombre.")
                foot_node_FL = buscar_nodo_por_nombre(self_node, PALABRAS_CLAVE_PIE_FL)
        else:
            foot_node_FL = buscar_nodo_por_nombre(self_node, PALABRAS_CLAVE_PIE_FL)

        if foot_node_FL is not None:
            try:
                nombre_encontrado = foot_node_FL.getField("name").getSFString()
            except Exception:
                nombre_encontrado = "(sin campo name)"
            print(f"[trail] Nodo del pie FL encontrado: '{nombre_encontrado}'. "
                  f"Se usará su posición real en el mundo.")
        else:
            print("[trail] No se encontró un nodo de pie por nombre ni por DEF. "
                  "Se usará el cálculo manual (menos preciso). Revisa ")

    # Iterar sobre todas las patas
    for pata in ["FL", "FR", "RL", "RR"]:
        lado = LADO[pata]

        if pata == "FL":
            px, py, pz = obtener_punto(t, TIPO, lado)
            # Guardar la posición planeada para usarla en el dibujo de matplotlib
            px_plan_FL, py_plan_FL, pz_plan_FL = px, py, pz
        else:
            px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA

        q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
        motors[pata][0].setPosition(q1)
        motors[pata][1].setPosition(q2)
        motors[pata][2].setPosition(q3)

        # Si es FL, leer sensores y actualizar estela en Webots
        if pata == "FL":
            q1_real = sensors["FL"][0].getValue()
            q2_real = sensors["FL"][1].getValue()
            q3_real = sensors["FL"][2].getValue()
            px_real, py_real, pz_real = cinematica_directa(q1_real, q2_real, q3_real, lado)
            px_real_FL, py_real_FL, pz_real_FL = px_real, py_real, pz_real
            if TIPO=="escalon":
                LARGO_ROBOT = 0.8
                OFFSET_X_CUERPO = LARGO_ROBOT*0.7
                OFFSET_Y_HOMBRO = a1*2
                OFFSET_Z_HOMBRO = -0.5      
                AUTO_CALIBRAR_OFFSETS = True
            
                ESCALA_TRAYECTORIA_VISUAL=2.5
            elif TIPO=="espiral" or "n":
                LARGO_ROBOT = 0.8
                OFFSET_X_CUERPO = LARGO_ROBOT*1.1
                OFFSET_Y_HOMBRO = a1*2
                OFFSET_Z_HOMBRO = -0.5      
                AUTO_CALIBRAR_OFFSETS = True
                ESCALA_TRAYECTORIA_VISUAL=1.5
            elif TIPO=="lissajous": 
                LARGO_ROBOT = 0.8
                OFFSET_X_CUERPO = LARGO_ROBOT*(-1*0.1)
                OFFSET_Y_HOMBRO = a1*2
                OFFSET_Z_HOMBRO = 0.15      
                #AUTO_CALIBRAR_OFFSETS = True
                ESCALA_TRAYECTORIA_VISUAL=1
                
            # Actualizar estela en Webots 
            if step_count % TRAIL_EVERY == 0:
                pos_mundo = calcular_punto_pie_mundo(px_real, py_real, pz_real, lado)
                agregar_punto_curva(pos_mundo)

    # registro error
    if step_count % MUESTREO_ERROR_EVERY == 0:
        hist_x.append(px_real_FL)
        hist_y.append(py_real_FL)
        hist_z.append(pz_real_FL)
        hist_plan_x.append(px_plan_FL)
        hist_plan_y.append(py_plan_FL)
        hist_plan_z.append(pz_plan_FL)
        if len(hist_x) > 100:
            hist_x.pop(0); hist_y.pop(0); hist_z.pop(0)
            hist_plan_x.pop(0); hist_plan_y.pop(0); hist_plan_z.pop(0)

        # Historial completo para el RMSE y la gráfica de error final
        hist_full_t.append(t)
        hist_full_real.append((px_real_FL, py_real_FL, pz_real_FL))
        hist_full_plan.append((px_plan_FL, py_plan_FL, pz_plan_FL))

        # Imprimir RMSE (euclidiana) cada RMSE_EVERY registros
        if len(hist_full_t) % RMSE_EVERY == 0:
            rmse_actual = calcular_rmse(hist_full_real, hist_full_plan)
            if rmse_actual is not None:
                print(f"[t={t:.2f}s] RMSE  euclidiana): "
                      f"{rmse_actual*1000:.4f} mm")

    # Actualización de matplotlib
    if step_count % DRAW_EVERY == 0 and step_count > 0:
        # Calcular puntos del wireframe (usando ángulos reales de FL)
        q1_real = sensors["FL"][0].getValue()
        q2_real = sensors["FL"][1].getValue()
        q3_real = sensors["FL"][2].getValue()
        # Ya tenemos px_real_FL, py_real_FL, pz_real_FL

        B = np.array([0, 0, 0])
        H = np.array([0, LADO["FL"] * a1, 0])
        K = H + np.array([-L2 * np.sin(q2_real), 0, -L2 * np.cos(q2_real)])
        P = np.array([px_real_FL, py_real_FL, pz_real_FL])

        # Actualizar líneas
        line_hombro.set_data([B[0], H[0]], [B[1], H[1]])
        line_hombro.set_3d_properties([B[2], H[2]])

        line_pierna.set_data([H[0], K[0], P[0]], [H[1], K[1], P[1]])
        line_pierna.set_3d_properties([H[2], K[2], P[2]])

        tray_real.set_data(hist_x, hist_y)
        tray_real.set_3d_properties(hist_z)
        tray_planeada.set_data(hist_plan_x, hist_plan_y)
        tray_planeada.set_3d_properties(hist_plan_z)

        plt.draw()
        plt.pause(0.001)

    step_count += 1


# RMSE final con todo el historial
rmse_final = calcular_rmse(hist_full_real, hist_full_plan)
if rmse_final is not None:
    print(f"RMSE final: {rmse_final*1000:.4f} mm ({rmse_final*1e6:.2f} um)")

# Gráfica de RMSE final
if len(hist_full_t) > 0:
    real_arr = np.array(hist_full_real)
    plan_arr = np.array(hist_full_plan)
    error_instantaneo_mm = np.linalg.norm(real_arr - plan_arr, axis=1) * 1000.0

    fig_rmse, ax_rmse = plt.subplots(figsize=(10, 5))
    ax_rmse.plot(hist_full_t, error_instantaneo_mm, color='tab:red', linewidth=1.0,
                 label='Distancia euclidiana (error real)')
    if rmse_final is not None:
        ax_rmse.axhline(rmse_final * 1000.0, color='blue', linestyle='--', linewidth=2,
                         label=f'RMSE global = {rmse_final*1000:.4f} mm')
    ax_rmse.set_title(f"Error de seguimiento (pata FL) — {len(hist_full_t)} muestras")
    ax_rmse.set_xlabel("Tiempo (s)")
    ax_rmse.set_ylabel("Distancia euclidiana (mm)")
    ax_rmse.legend()
    ax_rmse.grid(True, alpha=0.3)
    plt.tight_layout()

plt.ioff()
plt.show()