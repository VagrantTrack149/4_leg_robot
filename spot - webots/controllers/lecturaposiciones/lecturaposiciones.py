"""Controlador que solo lee la posición real de los motores y calcula la posición del pie."""

import math
import numpy as np
from controller import Robot
from pata_common import cinematica_directa  # Asegúrate de que pata_common.py esté en el mismo directorio

# Configuración: mismo lado que en tu controlador original (pata delantera izquierda)
LADO = 1

# Inicializar robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Nombres de los motores (deben coincidir con los del modelo Spot)
motor_names = [
    "front left shoulder abduction motor",
    "front left shoulder rotation motor",
    "front left elbow motor"
]

# Obtener los dispositivos motores
motors = [robot.getDevice(name) for name in motor_names]

# Habilitar sensores de posición para cada motor
position_sensors = []
for motor in motors:
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)
    position_sensors.append(sensor)

# Pequeña espera para que los sensores se actualicen
for _ in range(5):
    robot.step(timestep)

# Leer posiciones reales de los motores (radianes)
q1_real = position_sensors[0].getValue()
q2_real = position_sensors[1].getValue()
q3_real = position_sensors[2].getValue()

# Calcular cinemática directa para obtener la posición del pie
O0, O1, O2, O3 = cinematica_directa(q1_real, q2_real, q3_real, LADO)

# Mostrar resultados
print("=== POSICIÓN ACTUAL DEL PIE (desde sensores) ===")
print(f"Ángulos de motor: q1={q1_real:.4f} rad, q2={q2_real:.4f} rad, q3={q3_real:.4f} rad")
print(f"Posición del pie (x, y, z): ({O3[0]:.4f}, {O3[1]:.4f}, {O3[2]:.4f}) metros")

# Fin del controlador