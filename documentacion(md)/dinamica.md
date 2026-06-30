
### Notacion compacta

A lo largo del documento se usa:

$$c_i = \cos\theta_i, \quad s_i = \sin\theta_i$$

$$c_{ij} = \cos(\theta_i + \theta_j), \quad s_{ij} = \sin(\theta_i + \theta_j)$$

$$c_{ijk} = \cos(\theta_i + \theta_j + \theta_k), \quad s_{ijk} = \sin(\theta_i + \theta_j + \theta_k)$$

---

## Parametros Denavit-Hartenberg

### Convencion D-H estandar

Para cada eslabon $i$ se definen 4 parametros:

| Parametro | Simbolo | Significado fisico |
|---|:---:|---|
| Longitud del eslabon | $a_i$ | Distancia entre $Z_{i-1}$ y $Z_i$ medida a lo largo de $X_{i-1}$ |
| Torsion del eslabon | $\alpha_i$ | Angulo entre $Z_{i-1}$ y $Z_i$ girado alrededor de $X_{i-1}$ |
| Desplazamiento | $d_i$ | Distancia entre $X_{i-1}$ y $X_i$ medida a lo largo de $Z_{i-1}$ |
| Angulo articular | $\theta_i$ | Angulo entre $X_{i-1}$ y $X_i$ girado alrededor de $Z_{i-1}$ (variable) |

### 2.2 Tabla D-H de la pata del Spot

| Eslabon $i$ | $a_i$ (m) | $\alpha_i$ | $d_i$ (m) | $\theta_i$ (variable) | Rango articular |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 — Abduccion | 0.0528 | −90° | 0 | $\theta_1$ | [−0.6, +0.5] rad |
| 2 — Hombro | 0.2142 | 0° | 0 | $\theta_2$ | [−1.7, +1.7] rad |
| 3 — Codo | 0.2142 | 0° | 0.3197 | $\theta_3$ | [−0.45, +1.6] rad |

### 2.3 Justificacion de cada parametro

Por que $\alpha_1 = -90°$?  
J1 rota alrededor de Z (vertical). J2 rota alrededor de X (horizontal). Para pasar de un eje al otro se necesita una rotacion de −90° alrededor de X, que es exactamente $\alpha_1$.

Por que $\alpha_2 = \alpha_3 = 0°$?  
J2 y J3 tienen el mismo eje de rotacion (ambos en X).

Por que $d_3 \neq 0$ pero $d_1 = d_2 = 0$?  
El parametro $d$ mide desplazamiento a lo largo del eje $Z_{i-1}$. El forearm del Spot tiene una longitud estructural de 0.3197 m en esa direccion. Los primeros eslabones se desplazan en X, no en Z.

Nota sobre $\theta_2$: En el modelo se aplica un offset de $+\pi/2$ rad para que la postura neutral ($\theta_2 = 0$) corresponda a la pata colgando verticalmente.

---

## Cinematica Directa

### Matriz de transformacion homogenea D-H

Cada articulacion genera una transformacion ${}^{i-1}T_i \in \mathbb{R}^{4\times4}$:

$${}^{i-1}T_i = \begin{bmatrix} c\theta_i & -s\theta_i\,c\alpha_i & s\theta_i\,s\alpha_i & a_i\,c\theta_i \\ s\theta_i & c\theta_i\,c\alpha_i & -c\theta_i\,s\alpha_i & a_i\,s\theta_i \\ 0 & s\alpha_i & c\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Matrices individuales

${}^0T_1$ — Abduccion ($a_1 = 0.0528$, $\alpha_1 = -90°$, $d_1 = 0$):

$${}^0T_1 = \begin{bmatrix} c_1 & 0 & -s_1 & 0.0528\,c_1 \\ s_1 & 0 & c_1 & 0.0528\,s_1 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

${}^1T_2$ — Hombro ($a_2 = 0.2142$, $\alpha_2 = 0°$, $d_2 = 0$):

$${}^1T_2 = \begin{bmatrix} c_2 & -s_2 & 0 & 0.2142\,c_2 \\ s_2 & c_2 & 0 & 0.2142\,s_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

${}^2T_3$ — Codo ($a_3 = 0.2142$, $\alpha_3 = 0°$, $d_3 = 0.3197$):

$${}^2T_3 = \begin{bmatrix} c_3 & -s_3 & 0 & 0.2142\,c_3 \\ s_3 & c_3 & 0 & 0.2142\,s_3 \\ 0 & 0 & 1 & 0.3197 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Composicion — transformacion total

La pose del pie respecto al cuerpo:

$${}^0T_3 = {}^0T_1 \cdot {}^1T_2 \cdot {}^2T_3$$

Composicion ${}^0T_1 \cdot {}^1T_2 = {}^0T_2$:

$${}^0T_2 = \begin{bmatrix} c_1 c_2 & -c_1 s_2 & -s_1 & 0.0528\,c_1 + 0.2142\,c_1 c_2 \\ s_1 c_2 & -s_1 s_2 & c_1 & 0.0528\,s_1 + 0.2142\,s_1 c_2 \\ -s_2 & -c_2 & 0 & -0.2142\,s_2 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Posicion del pie (efector final)

La columna de traslacion de ${}^0T_3$ da la posicion del pie:

$$\boxed{p_x = c_1\bigl(0.0528 + 0.2142\,c_2 + 0.2142\,c_{23}\bigr) - 0.3197\,s_1}$$

$$\boxed{p_y = s_1\bigl(0.0528 + 0.2142\,c_2 + 0.2142\,c_{23}\bigr) + 0.3197\,c_1}$$

$$\boxed{p_z = -0.2142\,s_2 - 0.2142\,s_{23}}$$

Por que aparece $c_{23}$ y $s_{23}$?  
Porque J2 y J3 tienen el mismo eje (X), su composicion rotacional colapsa como suma de angulos:  
$R_X(\theta_2) \cdot R_X(\theta_3) = R_X(\theta_2 + \theta_3)$

### Matriz de orientacion del efector

La submatriz de rotacion $3 \times 3$ de ${}^0T_3$:

$$T_{03} = \begin{bmatrix} c_1 c_{23} & -c_1 s_{23} & -s_1 \\ s_1 c_{23} & -s_1 s_{23} & c_1 \\ -s_{23} & -c_{23} & 0 \end{bmatrix}$$

### Posiciones de los origenes de cada marco

| Marco | $X$ | $Y$ | $Z$ |
|:---:|---|---|---|
| $O_0$ | $0$ | $0$ | $0$ |
| $O_1$ | $a_1 c_1$ | $a_1 s_1$ | $0$ |
| $O_2$ | $a_1 c_1 + L_2 c_1 c_2$ | $a_1 s_1 + L_2 s_1 c_2 + d_3 c_1$ | $-L_2 s_2$ |
| $O_3$ | $p_x$ | $p_y$ | $p_z$ |

---

## Cinematica Inversa

### Planteamiento

Dado el punto objetivo $(p_x, p_y, p_z)$, encontrar $(\theta_1, \theta_2, \theta_3)$.

Las ecuaciones de posicion son el sistema a invertir:

$$p_x = c_1\bigl(\rho\bigr) - d_3\,s_1 \tag{1}$$

$$p_y = s_1\bigl(\rho\bigr) + d_3\,c_1 \tag{2}$$

$$p_z = -L_2 s_2 - L_3 s_{23} \tag{3}$$

donde $\rho = 0.0528 + 0.2142\,c_2 + 0.2142\,c_{23}$ agrupa la parte que no depende de $\theta_1$.

### Solucion de θ₁

Paso 1 — Eliminar ρ multiplicando en cruz:

Multiplicar (1) por $s_1$ y (2) por $c_1$, y restar:

$$p_y\,c_1 - p_x\,s_1 = d_3\,(c_1^2 + s_1^2) = d_3$$

Esta ecuacion es independiente de $\theta_2$ y $\theta_3$:

$$p_y\,c_1 - p_x\,s_1 = 0.3197 \tag{4}$$

Paso 2 — Resolver la ecuacion trigonometrica:

La forma $A\cos\theta - B\sin\theta = C$ se resuelve escribiendo:

$$\sqrt{p_x^2 + p_y^2}\,\cos\!\left(\theta_1 + \text{atan2}(p_x, p_y)\right) = d_3$$

Despejando:

$$\boxed{\theta_1 = \text{atan2}(p_y,\, p_x) - \text{atan2}\!\left(0.3197,\, \sqrt{p_x^2 + p_y^2 - 0.3197^2}\right)} \tag{5}$$

El signo positivo de la raiz corresponde a la solucion fisica (pata apuntando hacia afuera).

### Reduccion al problema planar

Elevando al cuadrado y sumando (1) y (2):

$$p_x^2 + p_y^2 = \rho^2 + d_3^2 \implies \rho = \sqrt{p_x^2 + p_y^2 - d_3^2}$$

Absorbiendo el offset $a_1 = 0.0528$, definimos el radio efectivo para el subsistema planar:

$$\boxed{R = \sqrt{p_x^2 + p_y^2 - 0.3197^2} - 0.0528} \tag{6}$$

El sistema se reduce al problema planar 2R en el plano $(R, p_z)$:

$$R = L_2\,c_2 + L_3\,c_{23} \tag{7}$$

$$p_z = -L_2\,s_2 - L_3\,s_{23} \tag{8}$$

### Solucion de θ₃

Paso 1 — Elevar al cuadrado y sumar (7) y (8):

$$(R)^2 + p_z^2 = L_2^2 + L_3^2 + 2L_2 L_3\underbrace{(c_2\,c_{23} + s_2\,s_{23})}_{=\,\cos\theta_3}$$

Por que? Porque $c_2\,c_{23} + s_2\,s_{23} = \cos(\theta_2+\theta_3-\theta_2) = \cos\theta_3$ por la identidad del coseno de la diferencia.

Paso 2 — Despejar $\cos\theta_3$:

$$\cos\theta_3 = \frac{(R)^2 + p_z^2 - L_2^2 - L_3^2}{2L_2 L_3}$$

Con $L_2 = L_3 = 0.2142$ m:

$$\boxed{C_3 = \frac{(R)^2 + p_z^2 - 0.09178}{0.09178}} \tag{9}$$

Paso 3 — Calcular θ₃:

$$\boxed{\theta_3 = \text{atan2}\!\left(+\sqrt{1 - C_3^2},\; C_3\right)} \tag{10}$$

El signo $+$ de la raiz corresponde al codo hacia adelante (postura natural del Spot).  
El signo $-$ corresponde a codo invertido (viola los limites articulares en condiciones normales).

### Solucion de θ₂

Con $\theta_3$ conocido, se definen los coeficientes auxiliares:

$$k_1 = L_2 + L_3\cos\theta_3 = 0.2142 + 0.2142\cos\theta_3 \tag{11}$$

$$k_2 = L_3\sin\theta_3 = 0.2142\sin\theta_3 \tag{12}$$

Significado geometrico: $k_1$ es la proyeccion del brazo total (con el codo a angulo $\theta_3$) en la direccion del brazo superior. $k_2$ es la componente perpendicular introducida por la flexion del codo.

Expandiendo $c_{23} = c_2 c_3 - s_2 s_3$ y $s_{23} = s_2 c_3 + c_2 s_3$ en (7) y (8):

$$R = k_1\,c_2 - k_2\,s_2$$

$$-p_z = k_1\,s_2 + k_2\,c_2$$

El sistema en forma matricial es:

$$\begin{bmatrix} R \\ -p_z \end{bmatrix} = \begin{bmatrix} k_1 & -k_2 \\ k_2 & k_1 \end{bmatrix} \begin{bmatrix} c_2 \\ s_2 \end{bmatrix}$$

La solucion por inversion directa (la matriz tiene determinante $k_1^2 + k_2^2 \neq 0$):

$$\boxed{\theta_2 = \text{atan2}(-p_z,\; R) - \text{atan2}(0.2142\sin\theta_3,\; 0.2142 + 0.2142\cos\theta_3)} \tag{13}$$

Lectura geometrica: El primer atan2 da la direccion al objetivo desde J2. El segundo atan2 descuenta el angulo que el codo ya "consume" de esa direccion.

### Formulas cerradas genericas — resumen

Dado cualquier $(p_x, p_y, p_z)$ alcanzable:

$$R = \sqrt{p_x^2 + p_y^2 - 0.3197^2} - 0.0528$$

$$\theta_1 = \text{atan2}(p_y,\, p_x) - \text{atan2}\!\left(0.3197,\, \sqrt{p_x^2+p_y^2-0.3197^2}\right)$$

$$C_3 = \frac{(R)^2 + p_z^2 - 0.09178}{0.09178}, \qquad \theta_3 = \text{atan2}\!\left(\sqrt{1-C_3^2},\, C_3\right)$$

$$\theta_2 = \text{atan2}(-p_z,\; R) - \text{atan2}(0.2142\sin\theta_3,\; 0.2142 + 0.2142\cos\theta_3)$$

### Orden de solucion y por que

El orden obligatorio es $\theta_1 \to \theta_3 \to \theta_2$ porque:

- θ₁ primero: La ecuacion (4) lo aisla completamente de $\theta_2$ y $\theta_3$.
- θ₃ antes que θ₂: La ley del coseno elimina $\theta_2$ en la ecuacion cuadratica, dando θ₃ directamente.
- θ₂ ultimo: Con θ₃ conocido, el sistema lineal $2\times2$ se resuelve trivialmente.

---
