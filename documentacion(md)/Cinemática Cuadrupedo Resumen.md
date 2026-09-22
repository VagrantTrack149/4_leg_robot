estado: escribiendo

fuentes:

- "Craig, J. J. - Robótica"
- "Reyes Cortés, F. - Robótica: Control de Robots Manipuladores"
- "Reyes Cortés, F. - MATLAB Aplicado a Robótica y Mecatrónica"

## Datos

| Parámetro                 | Valor                            | Uso                                          |
| :------------------------ | :------------------------------- | :------------------------------------------- |
| `D1`                      | 0.0528                           | Desplazamiento lateral del hombro            |
| `D1_Y`                    | 0.0006                           | Offset Y en abducción                        |
| `D2_Y`                    | -0.00053                         | Offset Y en rotación de hombro               |
| `D3`                      | 0.1122                           | Desplazamiento lateral entre hombro y codo   |
| `FEMUR_Y`                 | -0.319729                        | Offset Y del fémur                           |
| `FEMUR_Z`                 | 0.182338                         | Offset Z del fémur                           |
| `FOOT_OFFSET`             | [0.001038, -0.271321, -0.225038] | Offset final del pie (relativo a la rodilla) |
| Límite abducción          | (-0.6, 0.5)                      | q1                                           |
| Límite rotación           | (-1.7, 1.7)                      | q2                                           |
| Límite codo               | (-0.45, 1.6)                     | q3                                           |
| Velocidad por defecto     | 3.5                              | `setVelocity`                                |
| `MOUNT_AXIS`              | (0.577350, -0.577354, -0.577346) | Eje de montaje                               |
| `MOUNT_ANGLE`             | 2.094389                         | Ángulo de montaje                            |
| `MOUNT_TRANSLATION` FL/FR | [0.3635, 0.0, 0.0118]            | Montaje delantero                            |
| `MOUNT_TRANSLATION` RL/RR | [-0.3084, 0.0, 0.0117]           | Montaje trasero                              |

## Matrices homogéneas elementales

Toda la cadena se construye componiendo rotaciones puras y traslaciones puras de 4x4, como en Craig (cap. 2-3) y en Reyes Cortés (cap. 2). Cada matriz va en su propio bloque para que el webots no las mezcle:

$$ R_z(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta&0&0\\sin\theta&\cos\theta&0&0 \\ 0 &0&1&0 \\ 0&0&0&1\end{bmatrix} $$

$$ R_x(\theta)=\begin{bmatrix}1&0&0&0 \\ 0&\cos\theta&-\sin\theta&0 \\ 0&\sin\theta&\cos\theta&0 \\ 0&0&0&1\end{bmatrix} $$

$$ T_x(d)=\begin{bmatrix}1&0&0&d \\ 0&1&0&0 \\ 0&0&1&0 \\ 0&0&0&1\end{bmatrix} $$

$$ T_y(d)=\begin{bmatrix}1&0&0&0 \\ 0&1&0&d \\ 0&0&1&0 \\ 0&0&0&1\end{bmatrix} $$

$$ T_z(d)=\begin{bmatrix}1&0&0&0 \\ 0&1&0&0 \\ 0&0&1&d \\ 0&0&0&1\end{bmatrix} $$

Estas seis (`Rx`, `Ry`, `Rz`, `Tx`, `Ty`, `Tz`); `Ry` no se usa en la cadena de la pata (solo `Rz` para abducción y `Rx` para cadera/rodilla).

## Cadena cinemática

- `q1`: abducción del hombro (eje **Z**).
- `q2`: rotación de cadera (eje **X**).
- `q3`: rotación de rodilla (eje **X**).
- `s = -1` para pata izquierda, `s = +1` para pata derecha (mismo signo que el campo `left` del .proto).

Cada articulación se arma con el patrón trasladar al anchor, luego rotar en ese punto (así es como Webots encadena Transform->HingeJoint, y es equivalente a la convención de Craig de ubicar el sistema i en la unión y orientarlo con el ángulo articular):

$$ T_1 = T_x(sD_1),T_y(D1_Y),R_z(q_1) $$

$$ T_2 = T_1,T_y(D2_Y),R_x(q_2) $$

$$ T_{rodilla} = T_2,T_x(sD_3),T_y(FEMUR_Y),T_z(FEMUR_Z) $$

$$ T_3 = T_{rodilla},R_x(q_3) $$

Offset final del pie (vector homogéneo, en el marco de la rodilla antes de aplicar $q_3$):

$$ p_{forearm} = \begin{bmatrix} s\cdot FOOT_OFFSET_x \ FOOT_OFFSET_y \ FOOT_OFFSET_z \ 1 \end{bmatrix} $$

Posición homogénea del pie:

$$ p_{pie} = T_3, p_{forearm} $$

Puntos intermedios (si `puntos_intermedios=True`), cada uno en su propio bloque:

$$ p_{hombro} = T_1\begin{bmatrix}0 \ 0 \ 0 \ 1\end{bmatrix} $$

$$ p_{rodilla} = T_{rodilla}\begin{bmatrix}0 \ 0 \ 0\ 1\end{bmatrix} $$

$$ p_{pie} = T_3, p_{forearm} $$

## Forma cerrada (ecuación de conversión pata -> pie)

Como `Tx`, `Ty`, `Tz` son traslaciones puras (bloque de rotación = $I$), todo producto $T_{traslación}\cdot R(\theta)$ tiene la forma canónica

$$ \begin{bmatrix}R(\theta) & \vec{d}\ 0 & 1\end{bmatrix} $$

es decir: el bloque de rotación es solo el de la rotación, y el vector de traslación es solo el offset (no se afectan entre sí). Aplicando esto en cadena a $T_1$, $T_2$, $T_{rodilla}$, $T_3$, cada una queda en forma de bloque $\begin{bmatrix}R_i & \vec{d_i} \ 0&1\end{bmatrix}$, con:

$$ R_1=R_z(q_1) $$

$$ \vec{d_1}=(sD_1,; D1_Y,; 0) $$

$$ R_2=R_z(q_1),R_x(q_2) $$

$$ \vec{d_2}=\vec{d_1}+R_1,(0,; D2_Y,; 0)^{T} $$

$$ R_{rodilla}=R_2 $$

$$ \vec{d_{rodilla}}=\vec{d_2}+R_2,(sD_3,; FEMUR_Y,; FEMUR_Z)^{T} $$

$$ R_3=R_z(q_1),R_x(q_2),R_x(q_3) $$

$$ \vec{d_3}=\vec{d_{rodilla}} $$

(la rotación de rodilla $R_x(q_3)$ no traslada, por eso $\vec{d_3}=\vec{d_{rodilla}}$: es el mismo punto que $p_{rodilla}$).

Con esto la posición del pie tiene ecuación cerrada (sin necesidad de multiplicar matrices en tiempo de ejecución):

$$ p_{pie}(q_1,q_2,q_3) = \vec{d_{rodilla}} ;+; R_z(q_1),R_x(q_2),R_x(q_3)\cdot FOOT_OFFSET(s) $$

la rotación total es el producto $R_z(q_1)R_x(q_2)R_x(q_3)$ (no conmutativo) y la traslación total es $\vec{d_{rodilla}}$, que no es una simple suma de los offsets crudos: cada offset se ve rotado por todas las rotaciones que ya se aplicaron antes que él ($D2_Y$ rotado por $R_1$, y el offset del fémur rotado por $R_2$).

## Cinemática directa

Función `cinematica_directa(q1, q2, q3, left=True, puntos_intermedios=False)`.

- Entrada: ángulos articulares `q1, q2, q3` y bandera `left`.
- Salida: posición 3D del pie en el marco local de la pata (ecuación cerrada de arriba). Si `puntos_intermedios=True`, retorna también hombro y rodilla.
- Formulación matemática: La matriz de transformación total desde la base de la pata hasta el pie es: $$ T_{total} = T_1 \cdot T_2 \cdot T_{rodilla} \cdot T_3 $$ Por lo tanto, la ecuación de conversión de coordenadas es: $$ P_{pie} = T_{total} \cdot P_{forearm} $$ Donde $T_{total}$ es una matriz de $4 \times 4$ que contiene la rotación y traslación acumulada.
- Procedimiento en código: arma $T_1$, $T_2$, $T_{rodilla}$, $T_3$ por multiplicación de matrices homogéneas y evalúa $p_{pie}=T_3,p_{forearm}$; matemáticamente equivale a la forma cerrada de la sección anterior.

## Cinemática inversa

Hay dos formulaciones implementadas para el mismo problema. Las dos reciben lo mismo (`target_pos` en el marco local de la pata, `q_init` como semilla/referencia, y la bandera `left`), pero resuelven por caminos distintos. `Prueba_cerrada.py` usa la **cerrada**; versiones anteriores usaban la **iterativa por Jacobiano**.

### Forma algebraica (cerrada) en `cinematica_inversa` de `Prueba_cerrada.py`

Se despeja $q_1,q_2,q_3$ directamente de la ecuación cerrada de $p_{pie}$. La idea es desacoplar $q_1$ (que gira alrededor de Z y no toca $p_z$) y luego resolver el triángulo planar fémur-tibia por ley de cosenos.

Constantes auxiliares (se calculan una sola vez, fuera del ciclo):

$$ L_1=\sqrt{FEMUR_Y^2+FEMUR_Z^2},\qquad \phi_1=\operatorname{atan2}(FEMUR_Z,,FEMUR_Y) $$

$$ L_2=\sqrt{FOOT_OFFSET_y^2+FOOT_OFFSET_z^2},\qquad \phi_2=\operatorname{atan2}(FOOT_OFFSET_z,,FOOT_OFFSET_y) $$

$L_1$ es la "longitud fémur" (rodilla a pie antes de aplicar $q_3$) y $L_2$ la del offset del pie respecto a la rodilla. Los $\phi_i$ son el ángulo que esos offsets forman respecto al eje Y **en su propio plano**; aparecen porque ni el fémur ni el pie son perfectamente axiales.

**Paso 1: despejar $q_1$.** Proyectando en el plano XY (el único donde $q_1$ gira), la posición horizontal del pie cumple:

$$ p_x=sD_1+v_x\cos q_1-v_y\sin q_1,\qquad p_y=D1_Y+v_x\sin q_1+v_y\cos q_1 $$

con $v_x=s(D_3+FOOT_OFFSET_x)$ constante. Con $a=sD_1$, $b=D1_Y$ esto es una circunferencia de radio $v_x$ centrada en $(a,b)$; el punto $(p_x,p_y)$ solo es alcanzable si:

$$ r^2=(p_x-a)^2+(p_y-b)^2\ge v_x^2,\qquad v_y=\pm\sqrt{r^2-v_x^2} $$

Para cada signo de $v_y$:

$$ q_1=\operatorname{atan2}(p_y-b,;p_x-a)-\operatorname{atan2}(v_y,;v_x) $$

El doble signo de $v_y$ da las dos soluciones de hombro "espejadas" (codo adelante / codo atrás), que se filtran recién al final.

**Paso 2: plano sagital.** Restando la contribución del hombro:

$$ Z_y=v_y-D2_Y,\qquad Z_z=p_z $$

El vector $(Z_y,Z_z)$ va del origen del plano sagital al pie, y se descompone como suma de dos eslabones de longitudes $L_1$ y $L_2$:

$$ Z_y=L_1\cos u+L_2\cos v,\qquad Z_z=L_1\sin u+L_2\sin v $$

con $u=q_2+\phi_1$ y $v=q_2+q_3+\phi_2$. Por ley de cosenos:

$$ \cos\delta=\frac{Z_y^2+Z_z^2-L_1^2-L_2^2}{2L_1L_2},\qquad \delta=v-u=\pm\arccos(\cos\delta) $$

El signo de $\delta$ selecciona codo arriba / codo abajo. Con $\alpha=\operatorname{atan2}(Z_z,Z_y)$ y $\beta=\operatorname{atan2}(L_2\sin\delta,;L_1+L_2\cos\delta)$:

$$ u=\alpha-\beta,\qquad v=u+\delta $$

y de ahí:

$$ q_2=u-\phi_1,\qquad q_3=v-\phi_2-q_2 $$

**Paso 3: normalizar y seleccionar.** Las cuatro combinaciones (2 por $v_y$ × 2 por $\delta$) se normalizan a $[-\pi,\pi]$ con `(q + pi) % (2pi) - pi` (importante: los ángulos articulares son equivalentes mód $2\pi$, y sin normalizar los `LIMITS_IK` rechazan soluciones que físicamente sí caen dentro). Luego se descartan las que caen fuera de `LIMITS_IK` y entre las válidas se elige la más cercana a `q_init`:

$$ q^*=\arg\min_{q\in\mathcal{Q}_{válidas}}|q-q_{init}| $$

Esto es exactamente `_candidatas_ik` + `cinematica_inversa`. No hay iteración: es evaluación directa por cada candidata.

Notas:

- **Ventaja**: determinista, sin `max_iter` ni tolerancia. Siempre la misma solución para el mismo `q_init`. Si le pasamos el $q$ del paso anterior (como hace el `main`), la pata mantiene la **misma rama** durante toda la trayectoria.
- **Costo**: un puñado de `atan2`/`acos` por candidata. Contra las `max_iter × 4` evaluaciones de FK del método numérico, es prácticamente gratis.
- **Fallos**: si $r^2<v_x^2$, el objetivo cae dentro del agujero del círculo de hombro. Si $|\cos\delta|>1$, el objetivo queda fuera del alcance del fémur+tibia. Si ninguna candidata pasa los límites articulares, se levanta `ValueError("Objetivo inalcanzable dentro de los limites articulares")`.

### Forma iterativa por Jacobiano (versiones anteriores)

Función `cinematica_inversa(target_pos, q_init, left=True, max_iter=100, tol=1e-4)`.

- Entrada: posición objetivo `target_pos`, ángulos iniciales `q_init`, bandera `left`.
- Salida: ángulos articulares `q = [q1, q2, q3]`.
- Método: Newton-Raphson con Jacobiano numérico por diferencias finitas, porque de la ecuación cerrada de $p_{pie}(q_1,q_2,q_3)$ no se despeja $q_1,q_2,q_3$ en forma analítica simple (los tres ángulos entran acoplados dentro de $R_z(q_1)R_x(q_2)R_x(q_3)$).

Formulación:

1. Calcular $p_k = f(q_k)$.
2. Error $e_k = p_{objetivo} - p_k$. Si $|e_k| < tol$, terminar.
3. Jacobiano $J(q_k) \in \mathbb{R}^{3\times 3}$ por diferencias finitas ($\delta=10^{-6}$): $$ J_{:,i} \approx \dfrac{f(q+\delta e_i)-f(q)}{\delta} $$
4. Resolver $J(q_k),\Delta q = e_k$ (`np.linalg.solve`, o pseudo-inversa si es singular).
5. Actualizar $q_{k+1} = q_k + \Delta q$.
6. Recortar a límites articulares con `np.clip`.

Desventajas frente a la forma cerrada:

- La convergencia depende de `q_init`; puede caer en otra rama o quedarse oscilando.
- Hasta `max_iter=100` evaluaciones de FK por llamada, y cada una hace 3 perturbaciones, o sea hasta ~300 `cinematica_directa` en el peor caso.
- No distingue "objetivo inalcanzable": simplemente devuelve lo que haya después de `max_iter`.

### Comparación rápida

|                           | Cerrada                         | Jacobiano          |
| ------------------------- | ------------------------------- | ------------------ |
| Iteraciones               | 0                               | hasta 100          |
| Evals FK por punto        | ~4                              | hasta ~400         |
| Detección de inalcanzable | sí (`ValueError`)               | no                 |
| Continuidad de rama       | forzada por `q_init`            | depende del paso   |
| Límites articulares       | filtro duro (rechaza candidata) | `np.clip` al final |

En `Prueba_cerrada.py` se usa la cerrada. El `q_init` que se pasa es el `q` del paso anterior (ver el `for i in range(N_PUNTOS)` del `main`), y ese detalle es lo que mantiene la pata en la misma rama durante toda la trayectoria.

## Cinemática en formato Denavit-Hartenberg (DH)

Todo lo de arriba (`T_1`, `T_2`, `T_rodilla`, `T_3` y la forma cerrada) se puede reescribir como una tabla DH clásica de 3 grados de libertad, con la misma convención que usa Reyes Cortés para el robot antropomórfico RRR (cap. 4, tabla 4.5): ejes $Z,X,X$ para hombro-cadera-rodilla, y

$$ H^i_{i-1}=R_z(\theta_i),T_z(d_i),T_x(l_i),R_x(\alpha_i) $$

(misma fórmula que sus ecuaciones 4.26-4.28, con $l_i\equiv a_i$). La pata usa exactamente la misma estructura de ejes que ese robot (ver PDF adjunto, RRR.pdf), solo que con tres offsets extra ($D2_Y$, $D_3+FOOT_OFFSET_x$ y el propio fémur/pie no axiales) que en el robot de 3 gdl del libro son cero.

**Idea clave (verificada numéricamente, error $\sim10^{-16}$ contra `cinematica_directa`):** los offsets $D_1$/$D1_Y$ no son parte de la cadena DH, son el montaje fijo de la pata (como el "T_base" de Craig); y los ángulos auxiliares $u,v$ que ya usa la IK cerrada **son literalmente** los ángulos DH $\theta_2$ y $\theta_2+\theta_3$. No hace falta re-derivar nada, solo renombrar.

### Transformación de base (no es articulación)

$$ T_{base}=T_x(sD_1),T_y(D1_Y),R_z!\left(\tfrac{\pi}{2}\right) $$

El $R_z(\pi/2)$ solo re-etiqueta los ejes locales de la pata (el eje $x_0$ de la tabla DH queda apuntando donde antes apuntaba $y$) para que $D2_Y$ pueda escribirse como el parámetro $a_1$ estándar (offset a lo largo de $x$); no mueve el pie, es puro cambio de nombre de ejes.

### Tabla DH

|Eslabón $i$|$l_i\ (a_i)$|$\alpha_i$|$d_i$|$\theta_i$|
|:-:|:--|:--|:--|:--|
|1 (hombro, abducción)|$D2_Y$|$\pi/2$|$0$|$q_1$|
|2 (cadera)|$L_1$|$0$|$s,(D_3+FOOT_OFFSET_x)$|$q_2+\phi_1$|
|3 (rodilla)|$L_2$|$0$|$0$|$q_3+\phi_2-\phi_1$|

con $L_1,\phi_1,L_2,\phi_2$ exactamente los mismos definidos en la sección de cinemática inversa (fémur y offset del pie puestos en forma polar). Nótese el parecido directo con la tabla 4.5 del libro: fila 1 es igual ($l_1,\pi/2,0,q_1$) salvo que aquí $l_1=D2_Y$ en vez de $0$; filas 2 y 3 son el mismo brazo planar 2R del libro ($l_2,0,0,q_2$ / $l_3,0,0,q_3$), solo que la articulación real $q_2,q_3$ está desfasada por $\phi_1,\phi_2$ porque el fémur y el pie no son axiales.

$D_3+FOOT_OFFSET_x$ aparece como $d_2$ (traslación a lo largo de $z_1$, el eje de la articulación de cadera) porque, al ser una traslación paralela al eje sobre el que giran $q_2$ y $q_3$, es invariante a ambas rotaciones — por eso en la forma cerrada original $v_x$ no depende de $q_2$ ni $q_3$.

### Matrices homogéneas elementales (cada una en su bloque)

$$ H^1_0 = R_z(q_1),T_x(D2_Y),R_x!\left(\tfrac{\pi}{2}\right) $$

$$ H^2_1 = R_z(q_2+\phi_1),T_z\big(s(D_3+FOOT_OFFSET_x)\big),T_x(L_1) $$

$$ H^3_2 = R_z(q_3+\phi_2-\phi_1),T_x(L_2) $$

### Cinemática directa en DH

$$ T^3_0 = T_{base};H^1_0,H^2_1,H^3_2 $$

$$ p_{pie} = T^3_0\begin{bmatrix}0 \\ 0 \\ 0 \\ 1\end{bmatrix} $$

Es la misma $p_{pie}(q_1,q_2,q_3)$ de la forma cerrada de la sección anterior (comprobado por sustitución numérica), solo que factorizada como tabla DH en vez de como offsets sueltos; $l_3=L_2$ ya incluye el `FOOT_OFFSET` completo (componentes $y,z$), así que no hace falta una transformación de "herramienta" aparte al final de la cadena.

### Cinemática inversa en DH

Como $\theta_2=u$ y $\theta_2+\theta_3=v$ por construcción, los pasos 1 y 2 de la IK cerrada **ya calculan directamente los ángulos DH**, sin ningún paso extra:

$$ \theta_1=q_1=\operatorname{atan2}(p_y-b,;p_x-a)-\operatorname{atan2}(v_y,;v_x) $$

$$ \theta_2=u=\alpha-\beta,\qquad \theta_3=\delta=v-u=\pm\arccos(\cos\delta) $$

(con $a,b,v_x,v_y,\alpha,\beta,\delta,\cos\delta$ definidos igual que en la sección de IK cerrada). Solo falta el paso de vuelta a los ángulos articulares reales que se mandan a los motores (`abduction`, `rotation`, `elbow`), que es la resta de los offsets constantes:

$$ q_1=\theta_1,\qquad q_2=\theta_2-\phi_1,\qquad q_3=\theta_3+\phi_1-\phi_2 $$

El resto del procedimiento (normalizar a $[-\pi,\pi]$, filtrar por `LIMITS_IK`, elegir la candidata más cercana a `q_init`) es idéntico, no cambia por escribirlo en DH.

## Montaje al cuerpo

Cada pata se monta con la misma rotación (todas comparten `MOUNT_AXIS`/`MOUNT_ANGLE`) y una traslación que solo depende de si es delantera o trasera:

$$ P_{cuerpo} = P_{mount} + R_{mount} \cdot P_{local} $$

En forma de matriz homogénea:

$$ T_{cuerpo} = \begin{bmatrix} R_{mount} & P_{mount} \\ 0 & 1 \end{bmatrix} \cdot T_{local} $$

Donde:

- $P_{mount} = MOUNT_TRANSLATION[leg]$
- $R_{mount}$ es la matriz 3x3 obtenida de `MOUNT_AXIS`/`MOUNT_ANGLE` con la fórmula de Rodrigues (`rot3_from_axis_angle`), no con una composición $R_zR_xR_y$ de Euler.

## Notas

- La cinemática directa es _analítica_: se obtiene por composición de transformaciones homogéneas.
- La cinemática inversa puede resolverse de forma **analítica/cerrada** (ver `Forma algebraica (cerrada)`) o de forma **numérica** (Jacobiano, ver `Forma iterativa por Jacobiano`). En `Prueba_cerrada.py` se usa la cerrada.
- El Jacobiano se aproxima numéricamente (diferencias finitas) para no derivar a mano $\partial p_{pie}/\partial q_i$; sería posible hacerlo de forma analítica derivando la ecuación cerrada, falta justificar.
- La secuencia de transformaciones (trasladar y luego rotar en cada articulación) y los offsets están verificados contra `SpotLeg.proto`: los anchors y las traslaciones del `endPoint Solid` coinciden exactamente con `D1`, `D1_Y`, `D2_Y`, `D3`, `FEMUR_Y`, `FEMUR_Z`, y el eje de cada `HingeJointParameters` (Z para abducción, X por defecto para cadera/rodilla) coincide con `Rz(q1)`/`Rx(q2)`/`Rx(q3)`.
- La tabla DH de la sección anterior es equivalente bit a bit a la forma cerrada (verificada numéricamente, error $\sim10^{-16}$); no es una re-derivación independiente, es la misma ecuación reetiquetada con $\theta_i,l_i,\alpha_i,d_i$.


[[Cinemática Cuadrupedo Resumen]]