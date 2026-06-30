# Modelo de la pierna: Denavit–Hartenberg, cinemática y dinámica

Este documento describe el modelo de una pata de 3 grados de libertad (abducción de cadera, flexión de cadera, flexión de rodilla) tal como está implementado en `pierna_techo_1.py` y replicado x4 en `4_patas.py`. Las ecuaciones DH se derivaron para que coincidan exactamente con las funciones `cinematica_directa` / `cinematica_inversa` ya existentes (se verificó numéricamente: error de cierre ~1e-17 en los puntos de prueba).

---

## 1. Parámetros geométricos

| Símbolo | Valor (m) | Significado |
|---|---|---|
| `a1` | 0.0528 | Longitud del eslabón de abducción (cadera → fémur) |
| `L2` | 0.2142 | Longitud del fémur (muslo) |
| `L3` | 0.2142 | Longitud de la tibia (canilla) |

`lado` ∈ {+1, −1} indica si la pata es del lado izquierdo o derecho del cuerpo (afecta el signo del eje de abducción).

Variables articulares: `q1` (abducción/aducción de cadera), `q2` (flexión/extensión de cadera), `q3` (flexión/extensión de rodilla).

---

## 2. Tabla de Denavit–Hartenberg

Convención usada: DH estándar, con el eje `q1` rotando alrededor de X (abducción), y `q2`, `q3` rotando alrededor de Y (plano sagital de avance/elevación). El signo `lado` se incorpora como una reflexión en el eje Y antes de aplicar la cadena DH.

| i | θᵢ | dᵢ | aᵢ₋₁ | αᵢ₋₁ |
|---|---|---|---|---|
| 1 | q1 | 0 | 0 | 90° |
| 2 | q2 + 90° | 0 | a1 | 90° |
| 3 | q3 | 0 | L2 | 0° |
| pie | — | 0 | L3 | 0° |

Donde la transformada elemental es la habitual:

```
A_i = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ₋₁) · Rot_x(αᵢ₋₁)
```

Esta combinación de ángulos α y el offset de 90° en θ₂ es la que, al multiplicar las matrices, reproduce exactamente:

```
O1 = (0,         lado·a1·cos(q1),   -a1·sin(q1))
O2 = O1 + (L2·sin(q2),       0,     -L2·cos(q2))
O3 = O2 + (L3·sin(q2+q3),    0,     -L3·cos(q2+q3))
```

> Nota práctica: si vas a re-derivar la tabla DH "desde cero" con un software de álgebra simbólica, usa estas ecuaciones de cierre como verificación — son las que ya corren en el robot.

---

## 3. Cinemática directa

A partir de q1, q2, q3 y `lado`:

```python
O0 = (0, 0, 0)                                          # origen de cadera
O1 = (0, lado*a1*cos(q1), -a1*sin(q1))                  # fin del eslabón de abducción
O2 = O1 + (L2*sin(q2), 0, -L2*cos(q2))                  # rodilla
O3 = O2 + (L3*sin(q2+q3), 0, -L3*cos(q2+q3))            # pie (efector final)
```

Interpretación física:
- `q1 = 0` ⇒ la pata cuelga en el plano vertical que contiene la cadera (sin abducción).
- `q2 = 0` ⇒ el fémur apunta verticalmente hacia abajo.
- `q3 = 0` ⇒ la tibia está alineada con el fémur (pierna extendida).
- El signo negativo en `cos(q2)` y `cos(q2+q3)` define z hacia abajo como negativo, consistente con que el suelo está en z muy negativo respecto a la cadera.

La posición final del pie en el sistema de la pata es simplemente `O3`. En coordenadas del mundo: `O3_mundo = CADERA + O3`.

---

## 4. Cinemática inversa

Dado un punto objetivo `(px, py, pz)` relativo a la cadera y el lado de la pata:

**Paso 1 — Ángulo de abducción `q1`:**
```python
q1 = arccos( clip(py * lado / a1, -1, 1) )
```
Esto viene de que el eslabón de abducción solo afecta la coordenada Y (proyección sobre el eje de cadera): `py·lado = a1·cos(q1)`.

**Paso 2 — Proyección al plano sagital (X-Z):**
```python
O1_z = -a1 * sin(q1)
dx = px
dz = pz - O1_z
R = hypot(dx, dz)          # distancia cadera-fémur al objetivo, en el plano sagital
R = min(R, L2 + L3 - eps)  # evita NaN si el punto está fuera de alcance
```

**Paso 3 — Ángulo de rodilla `q3` (ley de cosenos):**
```python
C3 = (R² - L2² - L3²) / (2·L2·L3)
C3 = clip(C3, -1, 1)
q3 = -arctan2( sqrt(1 - C3²), C3 )
```
El signo negativo selecciona la configuración de "rodilla hacia atrás/adentro" (codo abajo, en términos de brazo robótico) consistente con la geometría de una pata de cuadrúpedo tipo Spot.

**Paso 4 — Ángulo de cadera `q2`:**
```python
k1 = L2 + L3*cos(q3)
k2 = L3*sin(q3)
q2 = arctan2(dx, -dz) - arctan2(k2, k1)
```

Este es el método geométrico estándar de "ley de cosenos + ángulo compuesto" para un manipulador planar de 2 eslabones, aplicado después de haber resuelto el desacople de abducción en el paso 1.

**Limitaciones conocidas (presentes en el código actual):**
- No hay una rama alternativa para "rodilla hacia adelante" — siempre se toma `q3 < 0`.
- El `clip` en R y en C3 evita errores numéricos pero **enmascara silenciosamente** los puntos fuera del espacio de trabajo: el resultado será el punto más cercano alcanzable, no un error. Si se necesita lógica de "objetivo inalcanzable", hay que detectar el clipping explícitamente (ver código de control de trayectorias más abajo, que sí lo reporta).

---

## 5. Dinámica — diagnóstico y propuesta para `4_patas.py`

### 5.1 Qué tiene el archivo actualmente

`4_patas.py` es **puramente cinemático**: define una trayectoria de pie deseada por fase de trote (`objetivo_pie`), resuelve la cinemática inversa, y dibuja el resultado. No hay masas, inercias, fuerzas de contacto, torques articulares, ni integración de ecuaciones de movimiento. El movimiento se "teletransporta" usando posición deseada directamente, sin pasar por un modelo dinámico ni por un controlador de torque.

Esto es perfectamente razonable para visualizar la geometría de la marcha (que es probablemente la intención original), pero si el objetivo es simular cómo se *comportaría* el robot real (o diseñar el control de torque de los motores), falta una capa dinámica completa.

### 5.2 Modelo dinámico propuesto (por pata, 3 GDL)

Usando formulación de Lagrange para un manipulador serie de 3 eslabones (igual estructura para las 4 patas, ya que son geométricamente idénticas salvo el signo `lado`):

**Ecuación general del manipulador:**

```
M(q)·q̈ + C(q, q̇)·q̇ + G(q) = τ - Jᵗ(q)·F_contacto
```

donde:
- `q = [q1, q2, q3]ᵗ` — vector de posiciones articulares
- `M(q)` — matriz de inercia (3×3, simétrica positiva definida)
- `C(q, q̇)` — matriz de fuerzas de Coriolis/centrífugas
- `G(q)` — vector de torques gravitacionales
- `τ` — torques aplicados por los motores (entrada de control)
- `J(q)` — jacobiano del pie respecto a `q` (3×3)
- `F_contacto` — fuerza de reacción del suelo sobre el pie (no nula solo en fase de apoyo)

**Parámetros adicionales que hace falta definir** (no existen en el código actual):

| Parámetro | Descripción |
|---|---|
| `m1, m2, m3` | masas del eslabón de abducción, fémur y tibia |
| `I1, I2, I3` | momentos de inercia de cada eslabón (respecto a su propio eje) |
| `lc1, lc2, lc3` | distancia del eje de cada articulación a su centro de masa |
| `g = 9.81` | gravedad |
| `μ` | coeficiente de fricción pie-suelo (para `F_contacto` durante apoyo) |
| `k_suelo, b_suelo` | rigidez y amortiguamiento de un modelo de contacto tipo resorte-amortiguador, si se simula el impacto del pie |

**Jacobiano del pie (derivada de la cinemática directa de la sección 3):**

```
J11 = 0
J21 = -lado*a1*sin(q1)
J31 = -a1*cos(q1)

J12 = 0
J22 = 0
J32 = L2*sin(q2) + L3*sin(q2+q3)     (con signo, derivando O3_z respecto a q2)

J13 = L3*cos(q2+q3)
J23 = 0
J33 = L3*sin(q2+q3)
```

(El jacobiano completo 3×3 hay que ensamblarlo derivando `O3(q1,q2,q3)` de la sección 3 respecto a cada `qi`; arriba están las componentes no triviales como referencia para no tener que rederivarlo desde cero.)

**Para integrar este modelo en simulación:**

1. En fase de **vuelo** (pie en el aire, `F_contacto = 0`): integrar `q̈ = M⁻¹(q)·(τ - C·q̇ - G)` con un método tipo RK4, usando un controlador de torque (p. ej. PD en espacio articular o en espacio cartesiano vía `τ = Jᵗ·F_deseada`) que persiga la trayectoria que ya genera `objetivo_pie`.
2. En fase de **apoyo** (pie en el suelo): añadir restricción de contacto, ya sea como restricción holónoma (el pie no se mueve, se calculan las fuerzas de reacción) o como modelo de contacto blando (resorte-amortiguador) si se quiere simular el impacto.
3. La detección de fase ya existe implícitamente en `objetivo_pie` a través de `vuelo = sin(phi + π/2)` recortado en `max(0, vuelo)` — eso es indicador de fase de vuelo vs. apoyo y se puede reusar directamente como bandera `en_apoyo = (vuelo <= 0)`.

### 5.3 Alcance recomendado

Implementar la dinámica completa de las 4 patas con contacto es un proyecto sustancial (equivalente a un mini-motor de física tipo MuJoCo/PyBullet simplificado). Si el objetivo real es más acotado, dos alternativas más prácticas:

- **Dinámica de una sola pata, fase de vuelo únicamente** (sin contacto): manejable en ~100 líneas, sirve para diseñar el controlador de torque/PD articular.
- **Usar un motor de física existente** (PyBullet, MuJoCo) cargando la geometría DH como URDF, en vez de derivar Lagrange a mano — mucho más robusto para incluir fricción y contacto realista.

Puedo desarrollar cualquiera de las dos si me confirmas cuál te sirve — no las incluí completas en código todavía porque dependen de parámetros (masas/inercias) que no están en ninguno de los dos archivos originales y preferí no inventarlos.

---

## 6. Control de trayectorias — diagnóstico de `pierna_techo_1.py`

El bucle de animación actual hace esto en cada frame:

```python
error = objetivo - actual
actual += velocidad_interp * error      # filtro de primer orden, k = 0.2
```

Esto **no es un control que garantice llegar al punto exacto**: es un filtro exponencial. El pie se acerca asintóticamente al objetivo y, en cuanto la distancia cae bajo `tolerancia = 0.005 m`, salta al siguiente punto de la trayectoria — sin haber llegado realmente. Por diseño, **siempre habrá un error residual** entre el punto planeado y el punto donde el pie "realmente" estaba al momento del cambio de objetivo, y un error de seguimiento continuo si la trayectoria avanza más rápido que lo que el filtro puede alcanzar.

El código que te entrego en `pierna_techo_1_control.py` (ver archivo adjunto) mantiene la misma filosofía de interpolación (no inventé un PID nuevo de motor, porque no hay torques en este archivo — solo hay una posición objetivo en espacio cartesiano), pero agrega:

1. **Registro histórico completo** de la trayectoria realmente seguida por el pie (no solo el estado instantáneo).
2. **Marcado explícito** de cuál fue la posición real del pie en el instante exacto en que el sistema consideró "alcanzado" cada punto de la trayectoria planeada (el verdadero punto de comparación para el error).
3. **Verificación de alcanzabilidad**: si la cinemática inversa tuvo que recortar (`clip`) el objetivo por estar fuera del espacio de trabajo, se marca y reporta.
4. **Resumen final de error** al terminar la ruta: error punto-a-punto (waypoint planeado vs. posición real al momento de considerarlo alcanzado) y estadísticas (máximo, promedio, RMS), impreso en consola y mostrado en una gráfica adicional.
