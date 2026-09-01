module ga_engine_top #(
    parameter GENES_PER_IND = 30,  // 10 puntos 3D (10 x 3 = 30 genes)
    parameter POP_SIZE      = 32,  // Tamaño de población
    parameter MAX_GENS      = 1000  // Número de generaciones
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    // Salidas de Control y Estado
    output reg         done,
    output reg  [15:0] current_gen,
    output reg  [31:0] best_fitness
);

    localparam STATE_IDLE            = 4'd0;
    localparam STATE_INIT_POP        = 4'd1;
    localparam STATE_EVAL_INIT       = 4'd2;
    localparam STATE_EVAL_LOOP       = 4'd3;
    localparam STATE_EVAL_STORE      = 4'd4;
    localparam STATE_CROSS_INIT      = 4'd5;
    localparam STATE_CROSS_LOOP      = 4'd6;
    localparam STATE_MUTATION        = 4'd7;
    localparam STATE_NEXT_GEN        = 4'd8;
    localparam STATE_FINISHED        = 4'd9;

    // 'state' no es un puerto, por lo que podemos inicializarlo de manera segura
    reg [3:0] state = STATE_IDLE;

    // Registros de la Población y Fitness
    // Memoria flattened para simular arreglos 2D
    reg signed [15:0] population [0:POP_SIZE*GENES_PER_IND - 1];
    reg        [31:0] fitness    [0:POP_SIZE - 1];

    // Contadores e Índices (ahora todos registrados, no "integer"
    // de bucles combinacionales)
    reg [15:0] gen_count = 16'd0;
    reg [7:0]  ind_idx;
    reg [7:0]  gene_idx;
    reg [31:0] min_fitness;

    // Generador Pseudo-Aleatorio (LFSR de 32 bits)
    reg [31:0] lfsr;

    // Variables temporales para Cálculos Aritméticos
    reg signed [15:0] diff;
    reg        [31:0] diff_sq;
    reg        [31:0] acc_sum;
    reg [7:0]  p1_idx, p2_idx;

    // Direcciones planas precalculadas (un solo acceso por ciclo)
    wire [15:0] addr_self   = ind_idx * GENES_PER_IND + gene_idx;
    wire [15:0] addr_parent1 = p1_idx  * GENES_PER_IND + gene_idx;
    wire [15:0] addr_parent2 = p2_idx  * GENES_PER_IND + gene_idx;

    // MÁQUINA DE ESTADOS PRINCIPAL
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= STATE_IDLE;
            gen_count    <= 16'd0;
            done         <= 1'b0;
            min_fitness  <= 32'hFFFFFFFF;
            ind_idx      <= 8'd0;
            gene_idx     <= 8'd0;
            lfsr         <= 32'hACE12678;
            current_gen  <= 16'd0;
            best_fitness <= 32'hFFFFFFFF;
            acc_sum      <= 32'd0;
        end else begin

            // Actualización constante del LFSR (Taps: 32, 22, 2, 1)
            lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};

            case (state)

                
                // ESTADO 0: Espera de Inicio
                
                STATE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        gen_count   <= 16'd0;
                        min_fitness <= 32'hFFFFFFFF;
                        state       <= STATE_INIT_POP;
                        ind_idx     <= 8'd0;
                        gene_idx    <= 8'd0;
                    end
                end

                
                // ESTADO 1: Inicializar Población Aleatoria
                // (ya accedía a un solo elemento por ciclo, se
                //  mantiene igual)
                
                STATE_INIT_POP: begin
                    population[addr_self] <= lfsr[15:0];

                    if (gene_idx == GENES_PER_IND - 1) begin
                        gene_idx <= 8'd0;
                        if (ind_idx == POP_SIZE - 1) begin
                            ind_idx <= 8'd0;
                            state   <= STATE_EVAL_INIT;
                        end else begin
                            ind_idx <= ind_idx + 8'd1;
                        end
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                
                // ESTADO 2: Preparar evaluación de un individuo
                
                STATE_EVAL_INIT: begin
                    acc_sum  <= 32'd0;
                    gene_idx <= 8'd0;
                    state    <= STATE_EVAL_LOOP;
                end

                
                // ESTADO 3: Evaluación de Fitness (1 gen/ciclo)
                
                STATE_EVAL_LOOP: begin
                    diff    = population[addr_self];   // 1 sola lectura dinámica activa
                    diff_sq = diff * diff;              // Mapeado a bloque DSP
                    acc_sum <= acc_sum + diff_sq;

                    if (gene_idx == GENES_PER_IND - 1) begin
                        state <= STATE_EVAL_STORE;
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                
					 // ESTADO 4: Guardar fitness del individuo actual
                STATE_EVAL_STORE: begin
                    // Dividimos la suma total entre 10 puntos (MSE) directamente
                    fitness[ind_idx] <= (acc_sum / 32'd10);

                    if ((acc_sum / 32'd10) < min_fitness) begin
                        min_fitness <= (acc_sum / 32'd10);
                    end

                    if (ind_idx == POP_SIZE - 1) begin
                        ind_idx <= 8'd0;
                        state   <= STATE_CROSS_INIT;
                    end else begin
                        ind_idx <= ind_idx + 8'd1;
                        state   <= STATE_EVAL_INIT;
                    end
                end

                
                // ESTADO 5: Preparar Selección y Crossover
                
                STATE_CROSS_INIT: begin
                    p1_idx   <= lfsr[3:0] % POP_SIZE;
                    p2_idx   <= lfsr[7:4] % POP_SIZE;
                    gene_idx <= 8'd0;
                    state    <= STATE_CROSS_LOOP;
                end

                
                // ESTADO 6: Crossover (1 gen/ciclo)
                // Hijo = (Padre1 + Padre2) / 2
                // Solo 2 lecturas + 1 escritura dinámicas por
                // ciclo (en vez de 60+30 simultáneas)
                
                STATE_CROSS_LOOP: begin
                    population[addr_self] <=
                        (population[addr_parent1] + population[addr_parent2]) >>> 1;

                    if (gene_idx == GENES_PER_IND - 1) begin
                        state <= STATE_MUTATION;
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                
                // ESTADO 7: Mutación (ya era 1 acceso por ciclo)
                
                STATE_MUTATION: begin
                    if (lfsr[1:0] == 2'b11) begin
                        // Se aplica perturbación con signo al primer gen del individuo
                        population[ind_idx * GENES_PER_IND] <=
                            population[ind_idx * GENES_PER_IND] + $signed(lfsr[7:0]);
                    end

                    if (ind_idx == POP_SIZE - 1) begin
                        state <= STATE_NEXT_GEN;
                    end else begin
                        ind_idx <= ind_idx + 8'd1;
                        state   <= STATE_CROSS_INIT;
                    end
                end

                
                // ESTADO 8: Bucle Evolutivo
                
                STATE_NEXT_GEN: begin
                    if (gen_count == MAX_GENS - 1) begin
                        state <= STATE_FINISHED;
                    end else begin
                        gen_count <= gen_count + 1'b1;
                        ind_idx   <= 8'd0;
                        state     <= STATE_EVAL_INIT;
                    end
                end

                
                // ESTADO 9: Fin
                
                STATE_FINISHED: begin
                    done         <= 1'b1;
                    current_gen  <= gen_count;
                    best_fitness <= min_fitness;
                    if (!start) begin
                        state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;

            endcase
        end
    end

endmodule