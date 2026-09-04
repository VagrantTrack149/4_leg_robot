# import matplotlib
# matplotlib.use('TkAgg')   # matplotlib vivo
from controller import Supervisor
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

TIME_STEP = 32
N_PUNTOS = 100
N_REF_RMSE = 20
FACTOR_ESCALA = 40.0             # amplitud de la trayectoria del pie, en mm
TIPO_TRAYECTORIA = 'espiral'   # 'lissajous' 'helice'  'circulo'  'espiral' 
PAUSA_GRAFICO = 0.001            # segundos que matplotlib cede al event loop en cada paso
PASOS_ASENTAMIENTO = 4           # pasos de simulacion extra para que los motores lleguen al objetivo


# Clases para el manejo de las patas
class SpotLeg:
    def __init__(self, robot, leg_name):
        self.name = leg_name
        self.robot = robot

        motor_names = {
            'abduction': f"{leg_name} shoulder abduction motor",
            'rotation':  f"{leg_name} shoulder rotation motor",
            'elbow':     f"{leg_name} elbow motor"
        }
        sensor_names = {
            'abduction': f"{leg_name} shoulder abduction sensor",
            'rotation':  f"{leg_name} shoulder rotation sensor",
            'elbow':     f"{leg_name} elbow sensor"
        }

        self.motors = {}
        for key, name in motor_names.items():
            motor = robot.getDevice(name)
            if motor is None:
                print(f"ERROR: No se encontro el motor '{name}' para {leg_name}")
                sys.exit(1)
            self.motors[key] = motor  # control de POSICION

        self.sensors = {}
        for key, name in sensor_names.items():
            sensor = robot.getDevice(name)
            if sensor is None:
                print(f"ADVERTENCIA: No se encontro el sensor '{name}' para {leg_name}")
                continue
            sensor.enable(TIME_STEP)
            self.sensors[key] = sensor

        self.limits = {
            'abduction': (-0.6, 0.5),
            'rotation':  (-1.7, 1.7),
            'elbow':     (-0.45, 1.6)
        }
        self.default_velocity = 3.5

    def set_angles(self, abduction=None, rotation=None, elbow=None, velocity=None):
        if velocity is None:
            velocity = self.default_velocity
        targets = {'abduction': abduction, 'rotation': rotation, 'elbow': elbow}
        for key, motor in self.motors.items():
            if targets[key] is not None:
                low, high = self.limits[key]
                val = min(max(targets[key], low), high)
                motor.setPosition(val)
                motor.setVelocity(velocity)

    def get_sensor_values(self):
        return {key: sensor.getValue() for key, sensor in self.sensors.items()}

    def error_maximo(self, objetivo):
        valores = self.get_sensor_values()
        claves = ['abduction', 'rotation', 'elbow']
        return max(abs(valores.get(k, objetivo[i]) - objetivo[i]) for i, k in enumerate(claves))


class SpotRobot:
    def __init__(self, robot):
        self.robot = robot
        self.leg_names = ['front left', 'front right', 'rear left', 'rear right']
        self.legs = {name: SpotLeg(robot, name) for name in self.leg_names}

    def set_leg(self, leg_name, abduction=None, rotation=None, elbow=None, velocity=None):
        if leg_name not in self.legs:
            raise ValueError(f"Pata '{leg_name}' no reconocida. Usa: {self.leg_names}")
        self.legs[leg_name].set_angles(abduction, rotation, elbow, velocity)

    def set_all_legs(self, angles_dict, velocity=None):
        for leg_name, leg_angles in angles_dict.items():
            if leg_name in self.legs:
                self.legs[leg_name].set_angles(**leg_angles, velocity=velocity)

    def get_all_sensors(self):
        return {name: leg.get_sensor_values() for name, leg in self.legs.items()}



# Transformaciones homogeneas
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def Tx(d):
    T = np.eye(4); T[0, 3] = d; return T

def Ty(d):
    T = np.eye(4); T[1, 3] = d; return T

def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T

def rot3_from_axis_angle(axis, angle):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c],
    ])



# Cinematica directa de una pata
D1 = 0.0528
D1_Y = 0.0006
D2_Y = -0.00053
D3 = 0.1122
FEMUR_Y = -0.319729
FEMUR_Z = 0.182338
FOOT_OFFSET = np.array([0.001038, -0.271321, -0.225038])

#Cinematica directa para el wireframe
def cinematica_directa(q1, q2, q3, left=True, puntos_intermedios=False):
    """FK de una pata. q1: abduccion (eje Z). q2: cadera (eje X).
    q3: rodilla (eje X). Devuelve la posicion del pie en el marco °°°°°LOCAL°°°°°
    de la pata (antes de montarla al cuerpo)."""
    s = -1 if left else 1

    T1 = Tx(s * D1) @ Ty(D1_Y) @ Rz(q1)
    T2 = T1 @ Ty(D2_Y) @ Rx(q2)
    T_rodilla = T2 @ Tx(s * D3) @ Ty(FEMUR_Y) @ Tz(FEMUR_Z)
    T3 = T_rodilla @ Rx(q3)

    p_forearm = np.array([s * FOOT_OFFSET[0], FOOT_OFFSET[1], FOOT_OFFSET[2], 1.0])
    p_pie = T3 @ p_forearm

    if not puntos_intermedios:
        return p_pie[:3]

    p_hombro = T1 @ np.array([0.0, 0.0, 0.0, 1.0])
    p_rodilla = T_rodilla @ np.array([0.0, 0.0, 0.0, 1.0])
    return p_hombro[:3], p_rodilla[:3], p_pie[:3]

#Cinematica inversa para webots
def cinematica_inversa(target_pos, q_init, left=True, max_iter=100, tol=1e-4):
    q = np.array(q_init, dtype=float)
    for _ in range(max_iter):
        p = cinematica_directa(q[0], q[1], q[2], left)
        error = target_pos - p
        if np.linalg.norm(error) < tol:
            break
        J = np.zeros((3, 3))
        delta = 1e-6
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += delta
            p_plus = cinematica_directa(q_plus[0], q_plus[1], q_plus[2], left)
            J[:, i] = (p_plus - p) / delta
        try:
            dq = np.linalg.solve(J, error)
        except np.linalg.LinAlgError:
            dq = np.linalg.pinv(J) @ error
        q += dq
        q[0] = np.clip(q[0], -0.6, 0.5)
        q[1] = np.clip(q[1], -1.7, 1.7)
        q[2] = np.clip(q[2], -0.45, 1.6)
    return q



# Montaje real de cada pata al cuerpo

MOUNT_AXIS = (0.577350, -0.577354, -0.577346)
MOUNT_ANGLE = 2.094389
MOUNT_ROTATION = rot3_from_axis_angle(MOUNT_AXIS, MOUNT_ANGLE)

MOUNT_TRANSLATION = {
    'front left':  np.array([0.3635, 0.0, 0.0118]),
    'front right': np.array([0.3635, 0.0, 0.0118]),
    'rear left':   np.array([-0.3084, 0.0, 0.0117]),
    'rear right':  np.array([-0.3084, 0.0, 0.0117]),
}

LEG_KEY = {'front left': 'FL', 'front right': 'FR', 'rear left': 'RL', 'rear right': 'RR'}
COLORES = {'front left': 'b', 'front right': 'r', 'rear left': 'g', 'rear right': 'm'}
COLORES_RGB = {'front left': (0, 0, 1), 'front right': (1, 0, 0),
               'rear left': (0, 1, 0), 'rear right': (1, 0, 1)}


def a_marco_cuerpo(p_local, leg_name):
    #Convierte un punto del marco LOCAL de la pata al marco del cuerpo (X=avance, Y=lateral izq+, Z=vertical arriba+).
    return MOUNT_TRANSLATION[leg_name] + MOUNT_ROTATION @ p_local



#Trayectorias 
def generar_trayectoria(tipo, n_puntos):
    if tipo == 'lissajous':
        t = np.linspace(0, 2 * np.pi, n_puntos)
        x = np.sin(2 * t)
        y = np.sin(3 * t)
        z = np.cos(t)
    elif tipo == 'helice':
        t = np.linspace(0, 2 * np.pi, n_puntos)
        x = np.cos(t)
        y = np.sin(t)
        z = np.linspace(0, 1, n_puntos)
    elif tipo == 'circulo':
        t = np.linspace(0, 2 * np.pi, n_puntos)
        x = np.cos(t)
        y = np.sin(t)
        pendiente_x = 0.5
        pendiente_y = 0.2
        z = (pendiente_x * x) + (pendiente_y * y)
    elif tipo == 'espiral':
        t = np.linspace(0, 4 * np.pi, n_puntos)
        r = np.linspace(0.1, 1, n_puntos)
        x = r * np.cos(t)
        y = r * np.sin(t)
        z = np.linspace(0, 1, n_puntos)
    else:
        t = np.linspace(0, 2 * np.pi, n_puntos)
        x = np.cos(t)
        y = np.sin(2 * t) / 2
        z = np.sin(t)

    trayectoria = np.stack([x, y, z], axis=-1) * FACTOR_ESCALA
    return np.round(trayectoria)


def espejear(traj):
    #Espejea la trayectoria del pie sobre el eje lateral local (columna 0). RL = espejo(FR), RR = espejo(FL)
    #Solo apra pruebas luego lo eliminaré, primero que funcione
    esp = traj.copy()
    esp[:, 0] = -esp[:, 0]
    return esp


def calcular_rmse(planeada, realizada, n_ref=N_REF_RMSE):
    n = planeada.shape[0]
    idx = np.linspace(0, n - 1, n_ref).astype(int)
    diff = realizada[idx] - planeada[idx]
    rmse = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
    return rmse, idx



# Trayectoria de la pata en el mundo: se crea un IndexedLineSet en Webots y se va actualizando en cada paso(por eso son necesarios)

def crear_rastro_webots(robot, def_name, color_rgb, punto_inicial, n_puntos):
    """Crea en la escena de Webots una linea (IndexedLineSet) con
    n_puntos+1 vertices, todos inicialmente en punto_inicial, y
    devuelve el campo 'point' (MFVec3f) para irlos actualizando."""
    #por eso es cerrada la linea
    x0, y0, z0 = punto_inicial
    puntos_str = " ".join(f"{x0:.4f} {y0:.4f} {z0:.4f}" for _ in range(n_puntos + 1))
    indices_str = " ".join(f"{i} {i + 1} -1" for i in range(n_puntos))
    r, g, b = color_rgb

    vrml = (
        f"DEF {def_name} Shape {{\n"
        f"  appearance Appearance {{\n"
        f"    material Material {{ diffuseColor {r} {g} {b} emissiveColor {r} {g} {b} }}\n"
        f"  }}\n"
        f"  geometry IndexedLineSet {{\n"
        f"    coord Coordinate {{ point [ {puntos_str} ] }}\n"
        f"    coordIndex [ {indices_str} ]\n"
        f"  }}\n"
        f"  castShadows FALSE\n"
        f"}}\n"
    )

    children_field = robot.getRoot().getField('children')
    children_field.importMFNodeFromString(-1, vrml)
    nodo = robot.getFromDef(def_name)
    coord_nodo = nodo.getField('geometry').getSFNode().getField('coord').getSFNode()
    return coord_nodo.getField('point')


def actualizar_rastro(point_field, indice, punto_mundo):
    point_field.setMFVec3f(indice, [float(punto_mundo[0]), float(punto_mundo[1]), float(punto_mundo[2])])


def pie_a_mundo(self_node, p_pie_local, leg_name):
    #Convierte la posicion LOCAL del pie (de esa pata) a coordenadas absolutas del mundo, usando la pose actual del cuerpo del robot.
    body_pos = np.array(self_node.getPosition())
    body_rot = np.array(self_node.getOrientation()).reshape(3, 3)
    p_cuerpo = a_marco_cuerpo(p_pie_local, leg_name)
    return body_pos + body_rot @ p_cuerpo



#Figura de matplotlib: se crea una vez y se actualiza en cada paso

def crear_figura(leg_names, planeadas, neutral):
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    orden = ['front left', 'front right', 'rear right', 'rear left', 'front left']
    esquinas = np.array([MOUNT_TRANSLATION[p] for p in orden])
    ax.plot(esquinas[:, 0], esquinas[:, 1], esquinas[:, 2], 'k-', linewidth=2, label='Cuerpo')

    artistas = {}
    for leg_name in leg_names:
        color = COLORES[leg_name]

        traj_p_local = neutral[leg_name] + planeadas[leg_name]
        traj_p = np.array([a_marco_cuerpo(p, leg_name) for p in traj_p_local])
        ax.plot(traj_p[:, 0], traj_p[:, 1], traj_p[:, 2], '--', color=color, alpha=0.6,
                label=f'{LEG_KEY[leg_name]} planeada')

        base_c = a_marco_cuerpo(np.zeros(3), leg_name)
        pie0_c = a_marco_cuerpo(neutral[leg_name], leg_name)
        wf_line, = ax.plot([base_c[0], pie0_c[0]], [base_c[1], pie0_c[1]], [base_c[2], pie0_c[2]],
                            color=color, marker='o', linewidth=2, label=f'Pata {LEG_KEY[leg_name]}')

        real_line, = ax.plot([], [], [], '-', color=color, alpha=0.9,
                              label=f'{LEG_KEY[leg_name]} realizada')

        artistas[leg_name] = {'wireframe': wf_line, 'realizada': real_line, 'traj_p': traj_p}

    ax.set_xlabel('X - avance [m]')
    ax.set_ylabel('Y - lateral (izq +) [m]')
    ax.set_zlabel('Z - vertical (arriba +) [m]')
    ax.set_title('Spot: animacion en vivo (wireframe + trayectorias)')
    ax.legend(loc='upper left', fontsize=6, ncol=2)
    plt.tight_layout()
    fig.canvas.draw()
    plt.pause(PAUSA_GRAFICO)
    return fig, ax, artistas

#Corregir figura para darle más forma de robot y corregir la trayectoria que aparece en el cuello o cabeza ya ni se
def actualizar_figura(artistas, leg_name, p_hombro, p_rodilla, p_pie, realizada_local_hasta_ahora):
    base_c = a_marco_cuerpo(np.zeros(3), leg_name)
    p_hombro_c = a_marco_cuerpo(p_hombro, leg_name)
    p_rodilla_c = a_marco_cuerpo(p_rodilla, leg_name)
    p_pie_c = a_marco_cuerpo(p_pie, leg_name)

    pts = np.array([base_c, p_hombro_c, p_rodilla_c, p_pie_c])
    wf = artistas[leg_name]['wireframe']
    wf.set_data(pts[:, 0], pts[:, 1])
    wf.set_3d_properties(pts[:, 2])

    if len(realizada_local_hasta_ahora) > 0:
        traj_r_local = np.array(realizada_local_hasta_ahora)
        traj_r_c = np.array([a_marco_cuerpo(p, leg_name) for p in traj_r_local])
        rl = artistas[leg_name]['realizada']
        rl.set_data(traj_r_c[:, 0], traj_r_c[:, 1])
        rl.set_3d_properties(traj_r_c[:, 2])


def main():
    robot = Supervisor()
    spot = SpotRobot(robot)
    self_node = robot.getSelf()

    neutral = {}
    for leg_name in spot.leg_names:
        left = 'left' in leg_name
        neutral[leg_name] = cinematica_directa(0.0, 0.0, 0.0, left)

    puntos_mm = generar_trayectoria(TIPO_TRAYECTORIA, N_PUNTOS)
    puntos_m = puntos_mm / 1000.0

    traj_FR = puntos_m.copy()
    traj_FL = puntos_m.copy()
    traj_RL = espejear(traj_FR)   # RL espeja a FR
    traj_RR = espejear(traj_FL)   # RR espeja a FL

    planeadas = {
        'front right': traj_FR,
        'front left':  traj_FL,
        'rear left':   traj_RL,
        'rear right':  traj_RR,
    }
    realizadas = {leg: [] for leg in spot.leg_names}
    q_init = {leg: [0.0, 0.0, 0.0] for leg in spot.leg_names}

    fig, ax, artistas = crear_figura(spot.leg_names, planeadas, neutral)

    # Dejar rastro dentro de webos
    robot.step(TIME_STEP)  # asegura que getPosition/getOrientation ya sean validos
    rastros = {}
    for leg_name in spot.leg_names:
        punto0 = pie_a_mundo(self_node, neutral[leg_name], leg_name)
        def_name = f"RASTRO_{LEG_KEY[leg_name]}"
        rastros[leg_name] = crear_rastro_webots(robot, def_name, COLORES_RGB[leg_name], punto0, N_PUNTOS)

    print("Iniciando controlador")
    terminado = False
    for i in range(N_PUNTOS):
        if terminado:
            break

        objetivos = {}
        for leg_name in spot.leg_names:
            left = 'left' in leg_name
            objetivo = neutral[leg_name] + planeadas[leg_name][i]
            q = cinematica_inversa(objetivo, q_init[leg_name], left)
            q_init[leg_name] = q
            objetivos[leg_name] = q
            spot.set_leg(leg_name, abduction=q[0], rotation=q[1], elbow=q[2])

        # Asentamiento: varios pasos hasta acercarse al objetivo 
        # NO CONTROL REAL, UNICAMENTE PASOS PARA LLEGAR AL OBJETIVO REAL
        for _ in range(PASOS_ASENTAMIENTO):
            if robot.step(TIME_STEP) == -1:
                terminado = True
                break
        if terminado:
            break

        for leg_name in spot.leg_names:
            left = 'left' in leg_name
            valores = spot.legs[leg_name].get_sensor_values()
            angulos_reales = (valores.get('abduction', q_init[leg_name][0]),
                               valores.get('rotation', q_init[leg_name][1]),
                               valores.get('elbow', q_init[leg_name][2]))
            p_hombro, p_rodilla, p_pie = cinematica_directa(*angulos_reales, left=left,
                                                                  puntos_intermedios=True)
            realizadas[leg_name].append(p_pie - neutral[leg_name])

            # actualiza la figura de matplotlib
            actualizar_figura(artistas, leg_name, p_hombro, p_rodilla, p_pie, realizadas[leg_name])

            # actualiza el rastro dentro de Webots
            punto_mundo = pie_a_mundo(self_node, p_pie, leg_name)
            actualizar_rastro(rastros[leg_name], i + 1, punto_mundo)

        fig.canvas.draw_idle()
        plt.pause(PAUSA_GRAFICO)

    for leg_name in realizadas:
        realizadas[leg_name] = np.array(realizadas[leg_name])

    print(f"RMSE (m) usando {N_REF_RMSE} puntos de referencia:") #falta volver a crear la grafica
    idx_ref = None
    for leg_name in spot.leg_names:
        n_calc = min(planeadas[leg_name].shape[0], realizadas[leg_name].shape[0])
        if n_calc == 0:
            continue
        rmse, idx_ref = calcular_rmse(planeadas[leg_name][:n_calc], realizadas[leg_name][:n_calc])
        print(f"  Pata {LEG_KEY[leg_name]} ({leg_name}): {rmse:.5f} m")

    if idx_ref is not None:
        for leg_name in spot.leg_names:
            traj_p = artistas[leg_name]['traj_p']
            idx_validos = idx_ref[idx_ref < traj_p.shape[0]]
            ax.scatter(traj_p[idx_validos, 0], traj_p[idx_validos, 1], traj_p[idx_validos, 2],
                       color=COLORES[leg_name], marker='x', s=40)

    fig.canvas.draw()
    plt.savefig('spot_trayectorias.png', dpi=150) #Luego quitar
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()