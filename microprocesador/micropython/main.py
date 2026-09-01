import math
import random
import time
import gc


# PARÁMETROS GLOBALES

FACTOR_ESCALA = 1000.0
PESO_RMSE = 1.0
PESO_PENALIZACION_CAJA = 0.2
PESO_PENALIZACION_VECINOS = 0.5
FACTOR_UMBRAL_SEPARACION = 2.5
MARGEN_CAJA_LOCAL = 15.0
N_PUNTOS_REF = 300

# Parámetros fijos solicitados
N_PUNTOS = 10
POP_SIZE = 300
GENERACIONES = 600
TRAYECTORIAS = ['circulo', 'helice', 'lissajous', 'espiral']


# GENERACIÓN DE TRAYECTORIAS DE REFERENCIA

def generar_trayectoria(tipo, n_puntos):
    trayectoria = []
    pi = math.pi
    
    for i in range(n_puntos):
        if tipo == 'lissajous':
            t = 2 * pi * i / (n_puntos - 1)
            x = math.sin(2 * t)
            y = math.sin(3 * t)
            z = math.cos(t)
        elif tipo == 'helice':
            t = 2 * pi * i / (n_puntos - 1)
            x = math.cos(t)
            y = math.sin(t)
            z = i / (n_puntos - 1)
        elif tipo == 'circulo':
            t = 2 * pi * i / (n_puntos - 1)
            x = math.cos(t)
            y = math.sin(t)
            z = 0.0
        elif tipo == 'espiral':
            t = 4 * pi * i / (n_puntos - 1)
            r = 0.1 + 0.9 * (i / (n_puntos - 1))
            x = r * math.cos(t)
            y = r * math.sin(t)
            z = i / (n_puntos - 1)
        
        trayectoria.append([
            round(x * FACTOR_ESCALA),
            round(y * FACTOR_ESCALA),
            round(z * FACTOR_ESCALA)
        ])
    return trayectoria


# FUNCIONES MATEMÁTICAS Y DE FITNESS

def dist_sq_3d(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2

def norm_3d(p):
    return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)

def ordenamiento_dinamico(pts, trayectoria_ref):
    # Ordena los puntos generados según el índice más cercano en la trayectoria de referencia
    puntos_con_idx = []
    for p in pts:
        min_d2 = float('inf')
        best_idx = 0
        for i, ref in enumerate(trayectoria_ref):
            d2 = dist_sq_3d(p, ref)
            if d2 < min_d2:
                min_d2 = d2
                best_idx = i
        puntos_con_idx.append((best_idx, p))
    
    puntos_con_idx.sort(key=lambda x: x[0])
    return [p[1] for p in puntos_con_idx]

def calcular_rmse_verdadero(vertices, trayectoria_ref):
    sum_d2_min = 0.0
    num_seg = len(vertices) - 1
    
    for pref in trayectoria_ref:
        d_min_sq = float('inf')
        for i in range(num_seg):
            A = vertices[i]
            B = vertices[i+1]
            AB = [B[0]-A[0], B[1]-A[1], B[2]-A[2]]
            den = AB[0]**2 + AB[1]**2 + AB[2]**2
            if den < 1e-12:
                u = 0.0
            else:
                AP = [pref[0]-A[0], pref[1]-A[1], pref[2]-A[2]]
                u = (AP[0]*AB[0] + AP[1]*AB[1] + AP[2]*AB[2]) / den
                u = max(0.0, min(1.0, u))
            
            Q = [A[0] + u * AB[0], A[1] + u * AB[1], A[2] + u * AB[2]]
            d2 = dist_sq_3d(pref, Q)
            if d2 < d_min_sq:
                d_min_sq = d2
        sum_d2_min += d_min_sq
        
    return math.sqrt(sum_d2_min / len(trayectoria_ref))

def calcular_penalizaciones(vertices, umbral_separacion, longitud_ideal_segmento):
    pen_caja = 0.0
    pen_vecinos = 0.0
    dist_max_ideal = 2.5 * longitud_ideal_segmento
    
    for i in range(len(vertices) - 1):
        seg = [vertices[i+1][0] - vertices[i][0], vertices[i+1][1] - vertices[i][1], vertices[i+1][2] - vertices[i][2]]
        longitud = norm_3d(seg)
        
        if longitud > umbral_separacion:
            pen_caja += (longitud - umbral_separacion)
        if longitud > dist_max_ideal:
            pen_vecinos += (longitud - dist_max_ideal)
            
    return pen_caja, pen_vecinos

def calcular_fitness(ind, trayectoria_ref, umbral_sep, long_ideal):
    pts = [ind[i:i+3] for i in range(0, len(ind), 3)]
    pts_ord = ordenamiento_dinamico(pts, trayectoria_ref)
    vertices = [trayectoria_ref[0]] + pts_ord + [trayectoria_ref[-1]]
    
    rmse = calcular_rmse_verdadero(vertices, trayectoria_ref)
    pen_caja, pen_vecinos = calcular_penalizaciones(vertices, umbral_sep, long_ideal)
    
    fitness = (PESO_RMSE * rmse) + (PESO_PENALIZACION_CAJA * pen_caja) + (PESO_PENALIZACION_VECINOS * pen_vecinos)
    return fitness, rmse


# DEFINICIÓN DE LIMITES (BOUNDS) POR PUNTO

def obtener_bounds(trayectoria_ref, n_puntos):
    # Calcula longitud total
    long_total = 0.0
    for i in range(len(trayectoria_ref)-1):
        long_total += norm_3d([trayectoria_ref[i+1][j] - trayectoria_ref[i][j] for j in range(3)])
    
    espaciado_esperado = long_total / (len(trayectoria_ref) - 1)
    
    lb, ub = [], []
    for i in range(n_puntos):
        idx_centro = int(((i + 1) / (n_puntos + 1)) * (len(trayectoria_ref) - 1))
        centro = trayectoria_ref[idx_centro]
        
        radio_x = MARGEN_CAJA_LOCAL * espaciado_esperado * random.uniform(0.3, 2.5)
        radio_y = MARGEN_CAJA_LOCAL * espaciado_esperado * random.uniform(0.3, 2.5)
        radio_z = MARGEN_CAJA_LOCAL * espaciado_esperado * random.uniform(0.3, 2.5)
        
        lb.extend([centro[0] - radio_x, centro[1] - radio_y, centro[2] - radio_z])
        ub.extend([centro[0] + radio_x, centro[1] + radio_y, centro[2] + radio_z])
        
    return lb, ub, long_total


# EJECUCIÓN DEL ALGORITMO GENÉTICO (GA)

def ejecutar_ga(tipo):
    print("Conectado,")
    print("Iniciando GA - Trayectoria: " + tipo.upper())
    print("Config: " + str(N_PUNTOS) + " pts | Pop: " + str(POP_SIZE) + " | Gens: " + str(GENERACIONES))
    
    t0 = time.ticks_ms()
    trayectoria_ref = generar_trayectoria(tipo, N_PUNTOS_REF)
    lb, ub, long_total = obtener_bounds(trayectoria_ref, N_PUNTOS)
    
    long_ideal = long_total / (N_PUNTOS + 1)
    umbral_sep = long_ideal * FACTOR_UMBRAL_SEPARACION
    
    # Crear población inicial
    poblacion = []
    dim = N_PUNTOS * 3
    for _ in range(POP_SIZE):
        ind = [random.uniform(lb[i], ub[i]) for i in range(dim)]
        poblacion.append(ind)
        
    mejor_global_ind = None
    mejor_global_fit = float('inf')
    mejor_global_rmse = float('inf')

    # Bucle evolutivo
    for gen in range(GENERACIONES):
        evaluaciones = []
        for ind in poblacion:
            fit, rmse = calcular_fitness(ind, trayectoria_ref, umbral_sep, long_ideal)
            evaluaciones.append((fit, rmse, ind))
            
            if fit < mejor_global_fit:
                mejor_global_fit = fit
                mejor_global_rmse = rmse
                mejor_global_ind = ind[:]

        # Ordenar por Fitness
        evaluaciones.sort(key=lambda x: x[0])

        # Selección (Torneo / Elitismo): Conservar el top 20%
        nueva_poblacion = [e[2] for e in evaluaciones[:int(POP_SIZE * 0.2)]]

        # Reproducción y Mutación para completar la población
        while len(nueva_poblacion) < POP_SIZE:
            p1 = evaluaciones[random.randint(0, int(POP_SIZE * 0.4))][2]
            p2 = evaluaciones[random.randint(0, int(POP_SIZE * 0.4))][2]
            
            # Crossover (promedio)
            hijo = [(p1[i] + p2[i]) / 2.0 for i in range(dim)]
            
            # Mutación (20% de probabilidad por gen)
            for i in range(dim):
                if random.random() < 0.2:
                    hijo[i] += random.gauss(0, (ub[i] - lb[i]) * 0.05)
                    hijo[i] = max(lb[i], min(ub[i], hijo[i]))  # Clipping
                    
            nueva_poblacion.append(hijo)

        poblacion = nueva_poblacion
        print(".")
        # Reportar avance cada 100 generaciones
        if (gen + 1) % 100 == 0 or gen == 0:
            print("\n")
            print("Gen " + str(gen + 1) + "/" + str(GENERACIONES) + " | Mejor Fitness: " + str(round(mejor_global_fit, 3)) + " | RMSE: " + str(round(mejor_global_rmse, 3)))
            gc.collect()

    t_ejecucion = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
    print("\n Resultados Finales [" + tipo + "]:")
    print("   Tiempo total: " + str(round(t_ejecucion, 2)) + " s")
    print("   RMSE final: " + str(round(mejor_global_rmse, 3)))
    print("   Fitness final: " + str(round(mejor_global_fit, 3)))


# EJECUCIÓN PRINCIPAL

for tray in TRAYECTORIAS:
    ejecutar_ga(tray)
    gc.collect()