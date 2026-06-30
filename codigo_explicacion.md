# Explicación del funcionamiento del código

## Objetivo general

Este programa implementa la **planeación y simulación del movimiento de una pierna robótica** en un entorno tridimensional. Su propósito es generar una trayectoria suave mediante curvas de Bézier, verificar que todos los puntos de la trayectoria sean alcanzables por la pierna, evitar colisiones con un obstáculo y mostrar el movimiento en una simulación 3D.

El sistema integra conceptos de:

- Cinemática directa.
- Cinemática inversa.
- Planeación de trayectorias.
- Curvas de Bézier cúbicas.
- Detección de colisiones.
- Validación de límites articulares.
- Visualización y análisis del error de seguimiento.

---

# 1. Definición de parámetros

Al inicio del programa se definen las dimensiones físicas de la pierna.

```python
a1
L2
L3
```

Estas representan:

- a1 → longitud del primer eslabón (cadera)
- L2 → longitud del fémur
- L3 → longitud de la tibia

También se define la posición de la cadera en el espacio.

```python
CADERA_FR
```

Además se establecen los límites permitidos para cada articulación:

```python
Q1_MIN
Q2_MIN
Q3_MIN
```

Estos límites evitan que la pierna adopte posiciones físicamente imposibles.

---

# 2. Definición de la trayectoria

La variable

```python
trayectoria
```

almacena los puntos que deberá seguir el pie.

Actualmente el ejemplo utiliza únicamente dos puntos:

```python
trayectoria.append(...)
trayectoria.append(...)
```

Sin embargo, el código incluye ejemplos comentados para generar trayectorias más complejas como:

- espiral cónica
- curvas tridimensionales
- trayectorias senoidales

Esto permite sustituir fácilmente la ruta por cualquier conjunto de puntos.

---

# 3. Construcción del obstáculo

El obstáculo se modela como un volumen rectangular.

Se definen sus límites:

```python
ESCALON_X_MIN
ESCALON_X_MAX

ESCALON_Y_MIN
ESCALON_Y_MAX

ESCALON_Z_MIN
ESCALON_Z_MAX
```

Posteriormente se construyen:

- vértices
- caras
- representación gráfica mediante `Poly3DCollection`

Este volumen representa un escalón u objeto que la pierna debe evitar durante su desplazamiento.

---

# 4. Cinemática directa

La función

```python
cinematica_directa()
```

calcula la posición de cada articulación conociendo los ángulos articulares.

Obtiene las coordenadas de:

- origen
- cadera
- rodilla
- pie

Estas posiciones son utilizadas para dibujar la pierna durante la animación.

---

# 5. Cinemática inversa

La función

```python
cinematica_inversa()
```

resuelve el problema inverso:

A partir de una posición deseada del pie calcula los tres ángulos articulares necesarios para alcanzarla.

Durante este proceso también verifica:

- puntos fuera del espacio de trabajo
- errores numéricos
- soluciones físicamente inválidas

Si el punto no puede alcanzarse se marca como fuera de alcance.

---

# 6. Validación del espacio de trabajo

Antes de aceptar un punto de la trayectoria se realizan dos verificaciones.

## Alcance geométrico

La función

```python
punto_alcanzable()
```

comprueba que:

- exista solución de cinemática inversa
- los ángulos permanezcan dentro de sus límites

Solo los puntos válidos continúan en el proceso de planeación.

---

## Colisión con el obstáculo

La función

```python
punto_dentro_bloque()
```

determina si un punto se encuentra dentro del volumen del escalón.

Posteriormente

```python
segmento_colisiona()
```

verifica si alguno de los eslabones de la pierna atraviesa dicho obstáculo durante la simulación.

Cuando ocurre una colisión los segmentos cambian de color a naranja.

---

# 7. Planeación mediante curvas de Bézier

La trayectoria entre dos puntos consecutivos no se realiza mediante líneas rectas.

En su lugar se utiliza una curva de Bézier cúbica.

La función

```python
bezier_cubico()
```

calcula la posición sobre la curva usando cuatro puntos:

- punto inicial
- primer punto de control
- segundo punto de control
- punto final

Este método produce movimientos continuos y suaves, eliminando cambios bruscos de dirección.

---

# 8. Verificación de la curva

Una vez construida la curva se realizan dos comprobaciones.

## Colisión

```python
bezier_colisiona()
```

recorre múltiples muestras de la curva para verificar que ningún punto atraviese el obstáculo.

---

## Alcanzabilidad

```python
segmento_bezier_valido()
```

evalúa cada punto de la curva para asegurar que toda la trayectoria pueda ser seguida por la pierna.

No basta con que el inicio y el final sean válidos; toda la curva debe cumplir las restricciones cinemáticas.

---

# 9. Ajuste automático de la trayectoria

La función

```python
preparar_segmento()
```

es el núcleo del algoritmo.

Su función consiste en modificar automáticamente la forma de la curva cuando existe una colisión.

El procedimiento es:

1. construir la curva inicial
2. comprobar colisiones
3. aumentar gradualmente la elevación de los puntos de control
4. volver a verificar la colisión
5. validar nuevamente los ángulos articulares

Si después de varios intentos no se obtiene una solución válida, el segmento se considera bloqueado.

De esta manera el algoritmo intenta rodear el obstáculo sin intervención del usuario.

---

# 10. Cálculo de tangentes

La función

```python
calcular_tangentes()
```

estima la dirección del movimiento en cada waypoint.

Estas tangentes permiten construir curvas de Bézier con continuidad, evitando cambios bruscos de orientación entre segmentos consecutivos.

---

# 11. Preparación completa de la trayectoria

La función

```python
preparar_trayectoria_completa()
```

realiza todo el proceso de planeación.

Para cada segmento:

- elimina puntos inalcanzables
- calcula tangentes
- genera curvas de Bézier
- verifica colisiones
- verifica límites articulares

El resultado es una lista de curvas listas para ser ejecutadas.

---

# 12. Animación

La animación utiliza

```python
FuncAnimation
```

de Matplotlib.

En cada cuadro se realiza el siguiente procedimiento:

1. avanzar sobre la curva de Bézier
2. calcular la nueva posición del pie
3. resolver la cinemática inversa
4. obtener la nueva postura mediante cinemática directa
5. actualizar la visualización
6. comprobar colisiones
7. registrar la trayectoria seguida

La transición entre puntos se suaviza utilizando una interpolación cúbica.

---

# 13. Registro de trayectoria

Durante toda la simulación se almacenan dos conjuntos de datos.

## Trayectoria planeada

Corresponde a las curvas de Bézier generadas por el algoritmo.

---

## Trayectoria recorrida

Corresponde a las posiciones realmente ejecutadas durante la animación.

Ambas trayectorias se utilizan posteriormente para evaluar el desempeño del sistema.

---

# 14. Evaluación del error

Al finalizar el recorrido se ejecuta

```python
mostrar_reporte_error()
```

Esta función calcula la distancia entre la trayectoria ejecutada y la trayectoria planeada.

Posteriormente obtiene el indicador:

**RMSE (Root Mean Square Error)**

Este parámetro cuantifica el error promedio de seguimiento durante todo el recorrido.

Finalmente se generan dos gráficas:

- Error perpendicular en función del tiempo.
- Comparación entre trayectoria planeada y trayectoria ejecutada.

---

# Flujo general del algoritmo

El funcionamiento completo puede resumirse de la siguiente manera:

1. Definir dimensiones de la pierna.
2. Definir la trayectoria objetivo.
3. Construir el obstáculo.
4. Filtrar puntos alcanzables.
5. Calcular tangentes.
6. Generar curvas de Bézier.
7. Verificar colisiones.
8. Ajustar automáticamente la trayectoria si existe una colisión.
9. Simular el movimiento mediante cinemática inversa y directa.
10. Detectar colisiones durante la ejecución.
11. Registrar la trayectoria seguida.
12. Calcular el error RMSE.
13. Mostrar los resultados finales.

---

# Conclusión

El código implementa un sistema completo de planeación y seguimiento de trayectorias para una pierna robótica de tres grados de libertad. La solución combina modelos cinemáticos, curvas de Bézier y algoritmos de detección de colisiones para generar movimientos suaves y físicamente realizables. Además, incorpora una evaluación cuantitativa mediante el cálculo del RMSE, lo que permite analizar la precisión del seguimiento y validar el desempeño del algoritmo frente a obstáculos presentes en el entorno.