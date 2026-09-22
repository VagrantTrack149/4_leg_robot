estado: escribiendo


fuentes:
  - "Craig, J. J. - Robótica"
  - "Reyes Cortés, F. - Robótica: Control de Robots Manipuladores"
  - "Reyes Cortés, F. - MATLAB Aplicado a Robótica y Mecatrónica"

#  Cinemática y Dinámica de Actuadores

## 1. Cinemática de Actuadores y Transmisiones
La cinemática estudia el movimiento sin considerar las *fuerzas*. En actuadores, se refiere a cómo se traduce el movimiento del motor a la articulación.
### Espacio de Actuador vs. Espacio de Articulación (Craig, Cap. 3.9)
Diferencia Clave
No siempre hay una correspondencia 1:1 entre el motor y la articulación.

- *Espacio de Actuador*: Posiciones medidas por los sensores (encoders) en los motores.
- Espacio de Articulación:* Variables reales de la articulación del robot.
- Mecanismos de acoplamiento:* Un actuador lineal puede mover una articulación rotacional, o dos motores pueden trabajar juntos en un diferencial.
###  Convención Denavit-Hartenberg (DH) 
(Reyes - MATLAB Cap. 5; Reyes - Control Cap. 4)

Herramienta estándar para modelar la geometría del robot y sus actuadores.
- *Parámetros DH:
	- $l_i$ (longitud), 
	- $\alpha_i$ (torsión),
	- $d_i$ (desplazamiento, incluye ancho del servomotor $\beta_i$), 
	- $\theta_i$ (ángulo de articulación).
- Matrices de Transformación Homogénea: Describen la posición y orientación del actuador en el espacio tridimensional.
###  Transmisiones y Reducción (Craig, Cap. 8.6)
- Ubicación de Actuadores: Montaje directo (reduce fricción) y el montaje remoto (reduce inercia).
- Sistemas de Reducción:* Engranajes, bandas, cables, correas, tornillos de bolas.
- Relación de Transmisión ($\eta$):* 
  - Velocidad: $\dot{\theta}_o = \frac{1}{\eta} \dot{\theta}_i$
  - Par: $\tau_o = \eta \tau_i$

## 2. Dinámica de Actuadores y Transmisiones
Estudia las fuerzas y pares necesarios para generar el movimiento. Integra el modelo eléctrico del motor con la carga mecánica.
###  Modelado de una Sola Articulación y Motor CD (Craig, Cap. 9.9)
La inercia del rotor del motor se multiplica por el cuadrado de la relación de engranajes.
- **Circuito de Armadura:** Inductancia ($l_a$), resistencia ($r_a$), fuerza contraelectromotriz ($v = k_e \dot{\theta}_m$).
- **Par del Motor:** $\tau_m = k_m i_a$ (proporcional a la corriente).
- **Inercia Efectiva:** $I_{efectiva} = I + \eta^2 I_m$. La inercia del motor domina en robots con engranajes altos.
- **Flexibilidad no modelada:** Engranajes, ejes y cojinetes tienen rigidez finita $\rightarrow$ Resonancias que limitan las ganancias del controlador.
###  Modelo Dinámico de Robots (Reyes - Control Cap. 5; Craig Cap. 6)
Ecuación general de Euler-Lagrange:
$$ \tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + f(\dot{q}) $$
- **Fricción:** Fenómeno disipativo crítico en actuadores.
  - *Viscosa:* $b \dot{q}$ (proporcional a la velocidad).
  - *Coulomb:* $f_c \text{sgn}(\dot{q})$ (constante, opuesta al movimiento).
  - *Estática:* Fuerza umbral para iniciar el movimiento.
- **Propiedades:** $M(q)$ es simétrica y definida positiva. La linealidad en los parámetros permite identificar masas, inercias y fricción.
###  Cuerpos No Rígidos y Rigidez (Craig, Cap. 6.11 y Cap. 8.7)
- **Fricción Real:** Puede representar hasta el 25% del par total en robots con engranajes.
- **Rigidez Torsional de Ejes:** $k = \frac{G \pi d^4}{32 l}$
- **Efecto de Engranajes:** Aumentan la rigidez efectiva vista desde la salida por un factor de $\eta^2$.
- **Servomotores de Transmisión Directa (Reyes - Control Cap. 2):** Funcionan como fuente de par ideal, mínima fricción, alta resolución de encoders, pero menor capacidad de par.
---
## 3. Simulación, Identificación y Control
###  Simulación de Sistemas Dinámicos (Reyes - MATLAB Cap. 6)
- **Estructura de Estado:** Convertir EDO de 2do orden a sistema de 1er orden: $\dot{x} = f(x)$.
- **Métodos Numéricos:** Uso de `ode45` (Runge-Kutta 4/5) para integrar ecuaciones de motores, péndulos y robots de 2/3 GDL.
- **Código Fuente:** Scripts de MATLAB para simular par, fricción y dinámica del motor.
### Identificación Paramétrica (Reyes - MATLAB Cap. 7; Reyes - Control Cap. 5)
- **Mínimos Cuadrados Recursivo:** Estimar parámetros desconocidos (masas, inercias, fricción) a partir de datos experimentales (posición, velocidad, par).
- **Linealidad:** Expresar el modelo como $y = \Psi \theta$.
###  Control de Posición (Reyes - Control Cap. 6; Craig Cap. 9 y 10)
- **Control PD/PID:** Sintonización de ganancias considerando amortiguamiento y frecuencia natural.
- **Moldeo de Energía:** Técnica moderna que inyecta energía al sistema moldeando una función de potencial, asegurando estabilidad asintótica global incluso con no linealidades.
---
## Resumen de Capítulos Clave por Libro
| Libro                | Capítulo | Tema Relacionado con Actuadores                                      |
| :------------------- | :------- | :------------------------------------------------------------------- |
| **Craig (Robótica)** | Cap. 3   | Espacio de actuador vs. espacio de articulación.                     |
|                      | Cap. 6   | Dinámica de manipuladores (Fricción, rigidez, efectos no rígidos).   |
|                      | Cap. 8   | Diseño: Actuadores, transmisiones, rigidez.                          |
|                      | Cap. 9   | Control lineal: Modelado de motor CD, inercia efectiva, resonancias. |
| **Reyes (MATLAB)**   | Cap. 5   | Cinemática directa cartesiana y matrices DH.                         |
|                      | Cap. 6   | Simulación dinámica de sistemas.                                     |
|                      | Cap. 7   | Identificación paramétrica (Mínimos cuadrados).                      |
| **Reyes (Control)**  | Cap. 2   | Servomotores y sensores (Transmisión directa vs engranajes).         |
|                      | Cap. 4   | Cinemática de robots manipuladores.                                  |
|                      | Cap. 5   | Dinámica de robots (Euler-Lagrange, fricción).                       |


### Cap. 6  Control de posición 
(PD, PID, moldeo de energía). 
## Notas
[[Cinemática Cuadrupedo Resumen]]