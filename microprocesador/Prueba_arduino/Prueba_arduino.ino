#include <Arduino.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <float.h>


// ESTRUCTURAS DE DATOS

struct Punto3D {
    float x, y, z;
};

struct Individuo {
    std::vector<float> genes; // 10 puntos * 3 coords (x,y,z) = 30 genes
    float fitness;
    float rmse;
};


// PARÁMETROS GLOBALES

const float FACTOR_ESCALA = 1000.0f;
const float PESO_RMSE = 1.0f;
const float PESO_PENALIZACION_CAJA = 0.2f;
const float PESO_PENALIZACION_VECINOS = 0.5f;
const float FACTOR_UMBRAL_SEPARACION = 2.5f;
const float MARGEN_CAJA_LOCAL = 15.0f;

const int N_PUNTOS_REF = 300;
const int N_PUNTOS = 20;
const int POP_SIZE = 300;
const int GENERACIONES = 600;

const std::vector<String> TRAYECTORIAS = {"circulo", "helice", "lissajous", "espiral"};


// FUNCIONES AUXILIARES MATEMÁTICAS

float dist_sq_3d(const Punto3D& p1, const Punto3D& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

float norm_3d(const Punto3D& p) {
    return sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
}

// Generar número aleatorio float entre min y max
float randomFloat(float minVal, float maxVal) {
    return minVal + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (maxVal - minVal)));
}

// Distribución gaussiana aproximada (Box-Muller)
float randomGauss(float mean, float stddev) {
    float u1 = randomFloat(0.0001f, 1.0f);
    float u2 = randomFloat(0.0001f, 1.0f);
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return mean + z0 * stddev;
}


// GENERACIÓN DE TRAYECTORIAS DE REFERENCIA

std::vector<Punto3D> generar_trayectoria(const String& tipo, int n_puntos) {
    std::vector<Punto3D> trayectoria(n_puntos);
    
    for (int i = 0; i < n_puntos; ++i) {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float t = 0.0f;

        if (tipo == "lissajous") {
            t = 2.0f * M_PI * i / (n_puntos - 1);
            x = sinf(2.0f * t);
            y = sinf(3.0f * t);
            z = cosf(t);
        } else if (tipo == "helice") {
            t = 2.0f * M_PI * i / (n_puntos - 1);
            x = cosf(t);
            y = sinf(t);
            z = static_cast<float>(i) / (n_puntos - 1);
        } else if (tipo == "circulo") {
            t = 2.0f * M_PI * i / (n_puntos - 1);
            x = cosf(t);
            y = sinf(t);
            z = 0.0f;
        } else if (tipo == "espiral") {
            t = 4.0f * M_PI * i / (n_puntos - 1);
            float r = 0.1f + 0.9f * (static_cast<float>(i) / (n_puntos - 1));
            x = r * cosf(t);
            y = r * sinf(t);
            z = static_cast<float>(i) / (n_puntos - 1);
        }

        trayectoria[i] = {
            roundf(x * FACTOR_ESCALA),
            roundf(y * FACTOR_ESCALA),
            roundf(z * FACTOR_ESCALA)
        };
    }
    return trayectoria;
}


// ORDENAMIENTO DINÁMICO Y EVALUACIÓN

std::vector<Punto3D> ordenamiento_dinamico(const std::vector<Punto3D>& pts, const std::vector<Punto3D>& trayectoria_ref) {
    struct PuntoConIndice {
        int idx;
        Punto3D p;
    };
    std::vector<PuntoConIndice> lista(pts.size());

    for (size_t i = 0; i < pts.size(); ++i) {
        float min_d2 = FLT_MAX;
        int best_idx = 0;
        for (size_t j = 0; j < trayectoria_ref.size(); ++j) {
            float d2 = dist_sq_3d(pts[i], trayectoria_ref[j]);
            if (d2 < min_d2) {
                min_d2 = d2;
                best_idx = static_cast<int>(j);
            }
        }
        lista[i] = {best_idx, pts[i]};
    }

    std::sort(lista.begin(), lista.end(), [](const PuntoConIndice& a, const PuntoConIndice& b) {
        return a.idx < b.idx;
    });

    std::vector<Punto3D> ordenados(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        ordenados[i] = lista[i].p;
    }
    return ordenados;
}

float calcular_rmse_verdadero(const std::vector<Punto3D>& vertices, const std::vector<Punto3D>& trayectoria_ref) {
    float sum_d2_min = 0.0f;
    size_t num_seg = vertices.size() - 1;

    for (const auto& pref : trayectoria_ref) {
        float d_min_sq = FLT_MAX;
        for (size_t i = 0; i < num_seg; ++i) {
            Punto3D A = vertices[i];
            Punto3D B = vertices[i + 1];
            Punto3D AB = {B.x - A.x, B.y - A.y, B.z - A.z};

            float den = AB.x * AB.x + AB.y * AB.y + AB.z * AB.z;
            float u = 0.0f;
            if (den >= 1e-12f) {
                Punto3D AP = {pref.x - A.x, pref.y - A.y, pref.z - A.z};
                u = (AP.x * AB.x + AP.y * AB.y + AP.z * AB.z) / den;
                u = std::max(0.0f, std::min(1.0f, u));
            }

            Punto3D Q = {A.x + u * AB.x, A.y + u * AB.y, A.z + u * AB.z};
            float d2 = dist_sq_3d(pref, Q);
            if (d2 < d_min_sq) {
                d_min_sq = d2;
            }
        }
        sum_d2_min += d_min_sq;
    }
    return sqrtf(sum_d2_min / trayectoria_ref.size());
}

void calcular_penalizaciones(const std::vector<Punto3D>& vertices, float umbral_sep, float long_ideal, float& pen_caja, float& pen_vecinos) {
    pen_caja = 0.0f;
    pen_vecinos = 0.0f;
    float dist_max_ideal = 2.5f * long_ideal;

    for (size_t i = 0; i < vertices.size() - 1; ++i) {
        Punto3D seg = {vertices[i+1].x - vertices[i].x, vertices[i+1].y - vertices[i].y, vertices[i+1].z - vertices[i].z};
        float longitud = norm_3d(seg);

        if (longitud > umbral_sep) {
            pen_caja += (longitud - umbral_sep);
        }
        if (longitud > dist_max_ideal) {
            pen_vecinos += (longitud - dist_max_ideal);
        }
    }
}

void calcular_fitness(Individuo& ind, const std::vector<Punto3D>& trayectoria_ref, float umbral_sep, float long_ideal) {
    std::vector<Punto3D> pts(N_PUNTOS);
    for (int i = 0; i < N_PUNTOS; ++i) {
        pts[i] = {ind.genes[i * 3], ind.genes[i * 3 + 1], ind.genes[i * 3 + 2]};
    }

    std::vector<Punto3D> pts_ord = ordenamiento_dinamico(pts, trayectoria_ref);

    std::vector<Punto3D> vertices;
    vertices.reserve(N_PUNTOS + 2);
    vertices.push_back(trayectoria_ref.front());
    vertices.insert(vertices.end(), pts_ord.begin(), pts_ord.end());
    vertices.push_back(trayectoria_ref.back());

    ind.rmse = calcular_rmse_verdadero(vertices, trayectoria_ref);

    float pen_caja = 0.0f, pen_vecinos = 0.0f;
    calcular_penalizaciones(vertices, umbral_sep, long_ideal, pen_caja, pen_vecinos);

    ind.fitness = (PESO_RMSE * ind.rmse) + (PESO_PENALIZACION_CAJA * pen_caja) + (PESO_PENALIZACION_VECINOS * pen_vecinos);
}


// LÍMITES Y LÓGICA GA

void obtener_bounds(const std::vector<Punto3D>& trayectoria_ref, int n_puntos, std::vector<float>& lb, std::vector<float>& ub, float& long_total) {
    long_total = 0.0f;
    for (size_t i = 0; i < trayectoria_ref.size() - 1; ++i) {
        Punto3D diff = {trayectoria_ref[i+1].x - trayectoria_ref[i].x, trayectoria_ref[i+1].y - trayectoria_ref[i].y, trayectoria_ref[i+1].z - trayectoria_ref[i].z};
        long_total += norm_3d(diff);
    }

    float espaciado_esperado = long_total / (trayectoria_ref.size() - 1);
    lb.resize(n_puntos * 3);
    ub.resize(n_puntos * 3);

    for (int i = 0; i < n_puntos; ++i) {
        int idx_centro = static_cast<int>(((float)(i + 1) / (n_puntos + 1)) * (trayectoria_ref.size() - 1));
        Punto3D centro = trayectoria_ref[idx_centro];

        float rx = MARGEN_CAJA_LOCAL * espaciado_esperado * randomFloat(0.3f, 2.5f);
        float ry = MARGEN_CAJA_LOCAL * espaciado_esperado * randomFloat(0.3f, 2.5f);
        float rz = MARGEN_CAJA_LOCAL * espaciado_esperado * randomFloat(0.3f, 2.5f);

        lb[i * 3 + 0] = centro.x - rx; ub[i * 3 + 0] = centro.x + rx;
        lb[i * 3 + 1] = centro.y - ry; ub[i * 3 + 1] = centro.y + ry;
        lb[i * 3 + 2] = centro.z - rz; ub[i * 3 + 2] = centro.z + rz;
    }
}

void ejecutar_ga(const String& tipo) {
    Serial.print("Iniciando GA - Trayectoria: "); Serial.println(tipo);
    Serial.printf("Config: %d pts | Pop: %d | Gens: %d\n", N_PUNTOS, POP_SIZE, GENERACIONES);
    unsigned long t0 = millis();

    std::vector<Punto3D> trayectoria_ref = generar_trayectoria(tipo, N_PUNTOS_REF);
    std::vector<float> lb, ub;
    float long_total = 0.0f;
    obtener_bounds(trayectoria_ref, N_PUNTOS, lb, ub, long_total);

    float long_ideal = long_total / (N_PUNTOS + 1);
    float umbral_sep = long_ideal * FACTOR_UMBRAL_SEPARACION;
    int dim = N_PUNTOS * 3;

    std::vector<Individuo> poblacion(POP_SIZE);
    for (int i = 0; i < POP_SIZE; ++i) {
        poblacion[i].genes.resize(dim);
        for (int j = 0; j < dim; ++j) {
            poblacion[i].genes[j] = randomFloat(lb[j], ub[j]);
        }
    }

    Individuo mejor_global;
    mejor_global.fitness = FLT_MAX;

    for (int gen = 0; gen < GENERACIONES; ++gen) {
        for (int i = 0; i < POP_SIZE; ++i) {
            calcular_fitness(poblacion[i], trayectoria_ref, umbral_sep, long_ideal);
            if (poblacion[i].fitness < mejor_global.fitness) {
                mejor_global = poblacion[i];
            }
        }

        // Ordenar por Fitness
        std::sort(poblacion.begin(), poblacion.end(), [](const Individuo& a, const Individuo& b) {
            return a.fitness < b.fitness;
        });

        // Elitismo: Conservar top 20%
        std::vector<Individuo> nueva_poblacion;
        nueva_poblacion.reserve(POP_SIZE);
        int top_elite = static_cast<int>(POP_SIZE * 0.2);
        for (int i = 0; i < top_elite; ++i) {
            nueva_poblacion.push_back(poblacion[i]);
        }

        // Crossover y Mutación para completar el 80% restante
        int limite_seleccion = static_cast<int>(POP_SIZE * 0.4);
        while (nueva_poblacion.size() < POP_SIZE) {
            const Individuo& p1 = poblacion[rand() % limite_seleccion];
            const Individuo& p2 = poblacion[rand() % limite_seleccion];

            Individuo hijo;
            hijo.genes.resize(dim);
            for (int j = 0; j < dim; ++j) {
                hijo.genes[j] = (p1.genes[j] + p2.genes[j]) / 2.0f;

                // Mutación con probabilidad de 20%
                if (randomFloat(0.0f, 1.0f) < 0.2f) {
                    float delta = randomGauss(0.0f, (ub[j] - lb[j]) * 0.05f);
                    hijo.genes[j] = std::max(lb[j], std::min(ub[j], hijo.genes[j] + delta));
                }
            }
            nueva_poblacion.push_back(hijo);
        }

        poblacion = nueva_poblacion;
        Serial.print(".");
        // Mostrar progreso en monitor serie cada 100 generaciones
        if ((gen + 1) % 100 == 0 || gen == 0) {
            Serial.println("");
            Serial.printf("Gen %d/%d | Mejor Fitness: %.3f | RMSE: %.3f\n", gen + 1, GENERACIONES, mejor_global.fitness, mejor_global.rmse);
        }
    }

    float t_ejecucion = (millis() - t0) / 1000.0f;
    Serial.println("\n>> Resultados Finales [" + tipo + "]:");
    Serial.printf("   Tiempo total: %.2f s\n", t_ejecucion);
    Serial.printf("   RMSE final: %.3f\n", mejor_global.rmse);
    Serial.printf("   Fitness final: %.3f\n", mejor_global.fitness);

    // EXTRACCIÓN Y MUESTREO DE PUNTOS PARA GRAFICAR 
    std::vector<Punto3D> pts_mejor(N_PUNTOS);
    for (int i = 0; i < N_PUNTOS; ++i) {
        pts_mejor[i] = {mejor_global.genes[i * 3], mejor_global.genes[i * 3 + 1], mejor_global.genes[i * 3 + 2]};
    }
    // Ordenar los puntos según la trayectoria de referencia para que el gráfico no se cruce
    std::vector<Punto3D> pts_ordenados = ordenamiento_dinamico(pts_mejor, trayectoria_ref);

    Serial.println("---BEGIN_POINTS:" + tipo + "---");
    Serial.println("x,y,z");
    for (const auto& p : pts_ordenados) {
        Serial.printf("%.2f,%.2f,%.2f\n", p.x , p.y , p.z );
    }
    Serial.println("---END_POINTS---");
}


// SETUP Y LOOP DE ARDUINO IDE

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } 

    Serial.println("Conectado!");

    // Inicializar Semilla Aleatoria
    srand(micros());
    Serial.println("   ALGORITMO GENÉTICO EN ESP32-S3   ");

    for (const auto& tray : TRAYECTORIAS) {
        ejecutar_ga(tray);
    }
}

void loop() {
    delay(10000);
}