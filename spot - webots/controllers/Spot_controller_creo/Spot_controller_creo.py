from controller import Robot
import numpy as np


# ROBOT


robot = Robot()
timestep = int(robot.getBasicTimeStep())


# PARAMETROS DEL ROBOT


a1 = 0.0528
L2 = 0.2142
L3 = 0.2142

ALTURA_NEUTRA = -0.60


# CONFIGURACION DE PATAS


LADO = {
    'RR': -1,
    'RL':  1,
    'FR': -1,
    'FL':  1
}

# Fases para caminata
"""
FASES = {
    'FL': np.pi,
    'FR': 2*np.pi,
    'RL': np.pi,
    'RR': 2*np.pi
}
"""

FASES = {
    'FL' :2*np.pi,
    'FR': np.pi,
    'RL': np.pi,
    'RR': 2*np.pi 
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

# velocidad máxima para que la animación se vea suave

for pata in motors:
    for motor in motors[pata]:
        motor.setVelocity(10.0)


# CINEMATICA INVERSA


def cinematica_inversa(px, py, pz, lado):

    py_l = py * lado

    ratio = np.clip(py_l / a1, -1.0, 1.0)

    q1 = np.arccos(ratio)

    O1_z = -a1 * np.sin(q1)

    dx = px
    dz = pz - O1_z

    R = np.hypot(dx, dz)
    R = min(R, L2 + L3 - 1e-6)

    C3 = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    C3 = np.clip(C3, -1.0, 1.0)

    q3 = -np.arctan2(np.sqrt(1.0 - C3**2), C3)

    k1 = L2 + L3 * np.cos(q3)
    k2 = L3 * np.sin(q3)

    q2 = np.arctan2(dx, -dz) - np.arctan2(k2, k1)

    return q1, q2, q3


# GENERADOR DE PASO


FRECUENCIA = 2
AMP_PASO = 0.15
AMP_ELEVACION = 0.2

def objetivo_pie(nombre, t):

    lado = LADO[nombre]

    base = np.array([
        0.0,
        lado * a1,
        ALTURA_NEUTRA - 0.25
    ])

    phi = 2.0 * np.pi * FRECUENCIA * t + FASES[nombre]

    dx = -AMP_PASO * np.sin(phi)

    vuelo = np.sin(phi + np.pi / 2.0)
    vuelo=vuelo*vuelo

    dz = AMP_ELEVACION * max(0.0, vuelo)

    return base + np.array([dx, 0.0, dz])


# POSTURA INICIAL


for pata in ["FL", "FR", "RL", "RR"]:

    lado = LADO[pata]

    px, py, pz = (
        0.0,
        lado * a1,
        ALTURA_NEUTRA
    )

    q1, q2, q3 = cinematica_inversa(
        px,
        py,
        pz,
        lado
    )

    motors[pata][0].setPosition(q1)
    motors[pata][1].setPosition(q2)
    motors[pata][2].setPosition(q3)


# BUCLE PRINCIPAL


t = 0.0

while robot.step(timestep) != -1:

    t += timestep / 1000.0

    for pata in ["FL", "FR", "RL", "RR"]:

        lado = LADO[pata]

        px, py, pz = objetivo_pie(pata, t)

        q1, q2, q3 = cinematica_inversa(
            px,
            py,
            pz,
            lado
        )


        motors[pata][0].setPosition(q1)
        motors[pata][1].setPosition(q2)
        motors[pata][2].setPosition(q3)
