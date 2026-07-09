from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

robot = Robot()
timestep = int(robot.getBasicTimeStep())
# Parametros
a1 = 0.08       # Offset lateral de cadera (L_hip)
L2 = 0.34       # Largo del muslo (L_thigh)
L3 = 0.35       # Largo de la pantorrilla (L_calf)
ALTURA_NEUTRA = -0.8   # altura de la pata en reposo

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

# Velocidad para movimientos suaves
for pata in motors:
    for motor in motors[pata]:
        motor.setVelocity(10.0)


# CINEMÁTICA INVERSA ya no sé ni cual es
def cinematica_inversa(px, py, pz, lado):
    # Posición relativa a la cadera
    x = px
    y = py - lado * a1   # lado: 1=izquierda, -1=derecha
    z = pz
    
    # Ángulo de abducción de cadera (q1)
    q1 = np.arctan2(y, -z)
    
    # Distancia desde la cadera hasta el pie en el plano sagital
    L = np.hypot(y, z)
    d = np.hypot(x, L)
    
    # Ángulo de la rodilla (q3) - ley de cosenos
    cos_knee = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    q3 = np.pi - np.arccos(cos_knee)
    
    # Ángulo de flexión de cadera (q2)
    alpha = np.arctan2(x, L)
    beta = np.arccos(np.clip((L2**2 + d**2 - L3**2) / (2 * L2 * d), -1.0, 1.0))
    q2 = alpha + beta
    
    return q1, q2, q3


# Parametros circulo trauectoria

RADIO = 0.04          # radio del círculo
FREC_CIRC = 1.0       # vueltas por segundo



# possture inicial

for pata in ["FL", "FR", "RL", "RR"]:
    lado = LADO[pata]
    px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA
    q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
    motors[pata][0].setPosition(q1)
    motors[pata][1].setPosition(q2)
    motors[pata][2].setPosition(q3)

# grafico tiempo real

plt.ion()
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Robot Spot - Cinemática Pata FL")
ax.set_xlabel("X (adelante)"); ax.set_ylabel("Y (izquierda)"); ax.set_zlabel("Z (arriba)")
ax.set_xlim(-0.3, 0.3); ax.set_ylim(-0.3, 0.3); ax.set_zlim(-0.9, -0.1)

# Objetos para visualización
line_pierna, = ax.plot([], [], [], 'o-', color='orange', linewidth=4, markersize=8, label='Pierna (Wireframe)')
tray_real, = ax.plot([], [], [], 'r-', linewidth=1.5, label='Trayectoria Real')
tray_planeada, = ax.plot([], [], [], 'k--', linewidth=1, label='Planeada')

ax.legend()
plt.tight_layout()

# Historial para trayectoria
hist_x, hist_y, hist_z = [], [], []



t = 0.0
step_count = 0
DRAW_EVERY = 5


# DEFINICIÓN DE TRAYECTORIAS

TIPO = 'lissajous' #  'escalon', 'lissajous' o 'espiral'

def obtener_punto(t, tipo, lado):
    # Parámetros base
    t_mod = t % 4.0 # Ciclo de 4 segundos
    
    if tipo == 'escalon':
        px = 0.1 if t_mod < 2.0 else -0.1
        py = lado * a1 + 0.05
        pz = ALTURA_NEUTRA if t_mod < 1.0 or t_mod > 3.0 else ALTURA_NEUTRA + 0.1
        
    elif tipo == 'lissajous':
        px = 0.08 * np.sin(2 * np.pi * t)
        py = lado * a1 + 0.08 * np.sin(3 * np.pi * t)
        pz = ALTURA_NEUTRA + 0.04 * np.sin(5 * np.pi * t)
        
    else: # Espiral
        espiral_r = 0.02 + 0.05 * (t_mod / 4.0)
        px = espiral_r * np.cos(2 * np.pi * t)
        py = lado * a1 + espiral_r * np.sin(2 * np.pi * t)
        pz = ALTURA_NEUTRA + 0.02 * (t_mod / 4.0)
        
    return px, py, pz


while robot.step(timestep) != -1:
    t += timestep / 1000.0

    for pata in ["FL", "FR", "RL", "RR"]:
        lado = LADO[pata]
        
        if pata == "FL":
            px, py, pz = obtener_punto(t, TIPO, lado)
        else:
            px, py, pz = 0.0, lado * a1, ALTURA_NEUTRA

        q1, q2, q3 = cinematica_inversa(px, py, pz, lado)
        motors[pata][0].setPosition(q1)
        motors[pata][1].setPosition(q2)
        motors[pata][2].setPosition(q3)

        if pata == "FL" and step_count % DRAW_EVERY == 0:
            hist_x.append(px); hist_y.append(py); hist_z.append(pz)
            if len(hist_x) > 100: hist_x.pop(0); hist_y.pop(0); hist_z.pop(0)

            # Cálculo de eslabones para el wireframe
            H = np.array([0, lado * a1, 0])
            K = H + np.array([-L2 * np.sin(q2), 0, -L2 * np.cos(q2)])
            P = np.array([px, py, pz])

            line_pierna.set_data([H[0], K[0], P[0]], [H[1], K[1], P[1]])
            line_pierna.set_3d_properties([H[2], K[2], P[2]])
            tray_real.set_data(hist_x, hist_y)
            tray_real.set_3d_properties(hist_z)

            plt.draw()
            plt.pause(0.001)
    step_count += 1

# Mantener la figura abierta al final
plt.ioff()
plt.show()