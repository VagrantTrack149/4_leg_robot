//   Ordenamiento dinamico: cada punto del individuo se asigna al
//      indice mas cercano de la trayectoria de referencia.
//   RMSE verdadero: para cada uno de los N_REF puntos de referencia,
//      se busca la distancia minima a los 11 segmentos formados por
//      [ancla_inicio, 10 puntos ordenados, ancla_fin], con proyeccion
//      y clamp (point_seg_dist.v). fitness = sqrt(sum_d2_min / N_REF).

module ga_engine_top #(
    parameter GENES_PER_IND = 30,   // 10 puntos 3D (10 x 3 = 30 genes)
    parameter POP_SIZE      = 32,   // Tam. poblacion (debe ser potencia de 2)
    parameter MAX_GENS      = 200,  // Numero de generaciones
    parameter N_REF         = 300   // Puntos en la trayectoria de referencia
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [1:0]  traj_sel,    // 00=circulo 01=helice 10=lissajous 11=espiral

    output reg         done,
    output reg  [15:0] current_gen,
    output reg  [31:0] best_fitness
);

    localparam N_POINTS = GENES_PER_IND / 3;      // 10
    localparam N_VERT   = N_POINTS + 2;            // 12 (con anclas)
    localparam N_SEG    = N_VERT - 1;               // 11

    localparam signed [15:0] GENE_MIN = -16'sd1500;
    localparam signed [15:0] GENE_MAX =  16'sd1500;

    
    // Estados
    localparam
        S_IDLE            = 6'd0,
        S_INIT_POP        = 6'd1,
        S_GEN_START        = 6'd2,
        S_EVAL_ANCHOR0_REQ = 6'd3,
        S_EVAL_ANCHOR0_WT  = 6'd4,
        S_EVAL_ANCHOR1_REQ = 6'd5,
        S_EVAL_ANCHOR1_WT  = 6'd6,
        S_ORDER_PT_RX      = 6'd7,
        S_ORDER_PT_RY      = 6'd8,
        S_ORDER_PT_RZ      = 6'd9,
        S_ORDER_INIT_SCAN  = 6'd10,
        S_ORDER_SCAN_REQ   = 6'd11,
        S_ORDER_SCAN_WT    = 6'd12,
        S_ORDER_SCAN_CMP   = 6'd13,
        S_ORDER_PT_DONE    = 6'd14,
        S_SORT_INIT        = 6'd15,
        S_SORT_CMP         = 6'd16,
        S_SORT_STEP        = 6'd17,
        S_BUILD_VERT       = 6'd18,
        S_RMSE_REF_REQ     = 6'd19,
        S_RMSE_REF_WT      = 6'd20,
        S_RMSE_SEG_START   = 6'd21,
        S_RMSE_SEG_WAIT    = 6'd22,
        S_RMSE_SEG_NEXT    = 6'd23,
        S_RMSE_ACCUM       = 6'd24,
        S_RMSE_SQRT_START  = 6'd25,
        S_RMSE_SQRT_WAIT   = 6'd26,
        S_EVAL_STORE       = 6'd27,
        S_ELITE_COPY       = 6'd28,
        S_ELITE_NEXT_IND   = 6'd29,
        S_CROSS_TOURN_A    = 6'd30,
        S_CROSS_TOURN_B    = 6'd31,
        S_CROSS_TOURN_C    = 6'd36,
        S_CROSS_LOOP       = 6'd32,
        S_ELITE_WB         = 6'd33,
        S_NEXT_GEN         = 6'd34,
        S_FINISHED         = 6'd35;

    reg [5:0] state;

    
    // Memorias
    reg signed [15:0] population [0:POP_SIZE*GENES_PER_IND-1];
    reg        [31:0] fitness    [0:POP_SIZE-1];
    reg signed [15:0] elite_genes[0:GENES_PER_IND-1];

    // Buffers de ordenamiento (10 puntos del individuo actual)
    reg signed [15:0] ord_x [0:N_POINTS-1];
    reg signed [15:0] ord_y [0:N_POINTS-1];
    reg signed [15:0] ord_z [0:N_POINTS-1];
    reg        [8:0]  ord_idx[0:N_POINTS-1];   // indice de ref. mas cercano

    // Vertices finales (ancla_ini + 10 ordenados + ancla_fin)
    reg signed [15:0] vert_x [0:N_VERT-1];
    reg signed [15:0] vert_y [0:N_VERT-1];
    reg signed [15:0] vert_z [0:N_VERT-1];

    
    // Contadores / indices
    
    reg [15:0] gen_count;
    reg [7:0]  ind_idx;
    reg [7:0]  gene_idx;
    reg [31:0] min_fitness;          // mejor de todas las generaciones
    reg [31:0] min_fitness_gen;      // mejor de la generacion actual
    reg        elite_valid_gen;      // ya se copio algun elite esta gen

    reg [31:0] lfsr;

    
    // LFSR -> valor de gen aleatorio acotado (para init y mutacion)
    
    function signed [15:0] clamp_gene;
        input signed [15:0] v;
        begin
            if (v > GENE_MAX)      clamp_gene = GENE_MAX;
            else if (v < GENE_MIN) clamp_gene = GENE_MIN;
            else                   clamp_gene = v;
        end
    endfunction

    wire [15:0] addr_self = ind_idx * GENES_PER_IND + gene_idx;

    
    // Interfaz al banco de trayectorias
    
    reg  [8:0] traj_addr;
    wire signed [15:0] traj_x, traj_y, traj_z;

    trajectory_bank #(.N_REF(N_REF)) u_traj (
        .clk(clk), .traj_sel(traj_sel), .addr(traj_addr),
        .ref_x(traj_x), .ref_y(traj_y), .ref_z(traj_z)
    );
    // Latencia medida: 2 ciclos de clk desde que cambia traj_addr
    // hasta que traj_x/y/z son validos (1 ciclo ROM + 1 ciclo mux).
    reg [1:0] traj_wait_cnt;

    // Ancla inicial / final de la trayectoria (para el individuo actual)
    reg signed [15:0] anchor0_x, anchor0_y, anchor0_z;
    reg signed [15:0] anchor1_x, anchor1_y, anchor1_z;

    
    // Punto actual siendo procesado en el ordenamiento dinamico
    
    reg [3:0]  pt_k;              // 0..9
    reg signed [15:0] cur_px, cur_py, cur_pz;
    reg [8:0]  ref_scan_idx;
    reg [31:0] scan_min_d2;
    reg [8:0]  scan_min_idx;
    reg signed [16:0] sdx, sdy, sdz;
    reg        [31:0] scan_sum_d2;   // suma de cuadrados con ancho explicito (evita overflow de 17 bits)

    
    // RMSE: acumuladores
    
    reg [8:0]  rmse_ref_idx;
    reg [3:0]  seg_idx;           // 0..N_SEG-1
    reg [31:0] seg_min_d2;
    reg [63:0] sum_d2_min;        // suficiente margen (300 * ~1e8 cabe en 64 bits)
    reg [63:0] mse_temp;

    // point_seg_dist: instancia unica, reutilizada secuencialmente
    reg         psd_start;
    wire [31:0] psd_dist_sq;
    wire        psd_busy, psd_done;

    point_seg_dist u_psd (
        .clk(clk), .rst(rst), .start(psd_start),
        .ax(vert_x[seg_idx]), .ay(vert_y[seg_idx]), .az(vert_z[seg_idx]),
        .bx(vert_x[seg_idx+1]), .by(vert_y[seg_idx+1]), .bz(vert_z[seg_idx+1]),
        .px(traj_x), .py(traj_y), .pz(traj_z),
        .dist_sq(psd_dist_sq), .busy(psd_busy), .done(psd_done)
    );

    // isqrt32: instancia unica
    reg         sqrt_start;
    reg  [31:0] sqrt_rad;
    wire [15:0] sqrt_root;
    wire        sqrt_done;

    isqrt32 u_sqrt (
        .clk(clk), .rst(rst), .start(sqrt_start), .rad(sqrt_rad),
        .root(sqrt_root), .done(sqrt_done)
    );

    
    // Torneo / cruce / mutacion
    
    reg [7:0] p1_idx, p2_idx;
    reg [7:0] cand_a, cand_b;

    wire [15:0] addr_parent1 = p1_idx * GENES_PER_IND + gene_idx;
    wire [15:0] addr_parent2 = p2_idx * GENES_PER_IND + gene_idx;

    reg signed [15:0] hijo_val;
    reg        [7:0]  mut_rand;

    // Ordenamiento por insercion (10 elementos): indices i (elemento
    // actual a insertar) y j (posicion de comparacion hacia atras)
    reg [3:0] sort_i, sort_j;
    reg signed [15:0] tmp_x, tmp_y, tmp_z;
    reg        [8:0]  tmp_idx;

    
    integer g; // usado solo en bloques 'initial' de simulacion, no sintetiza init masivo

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= S_IDLE;
            gen_count       <= 16'd0;
            done            <= 1'b0;
            min_fitness     <= 32'hFFFFFFFF;
            min_fitness_gen <= 32'hFFFFFFFF;
            ind_idx         <= 8'd0;
            gene_idx        <= 8'd0;
            lfsr            <= 32'hACE12678;
            current_gen     <= 16'd0;
            best_fitness    <= 32'hFFFFFFFF;
            elite_valid_gen <= 1'b0;
            traj_wait_cnt   <= 2'd0;
            psd_start       <= 1'b0;
            sqrt_start      <= 1'b0;
        end else begin

            lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
            psd_start  <= 1'b0;
            sqrt_start <= 1'b0;

            case (state)

                
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        gen_count       <= 16'd0;
                        min_fitness     <= 32'hFFFFFFFF;
                        ind_idx         <= 8'd0;
                        gene_idx        <= 8'd0;
                        state           <= S_INIT_POP;
                    end
                end

                
                S_INIT_POP: begin
                    population[addr_self] <= clamp_gene($signed(lfsr[15:0]));

                    if (gene_idx == GENES_PER_IND - 1) begin
                        gene_idx <= 8'd0;
                        if (ind_idx == POP_SIZE - 1) begin
                            ind_idx <= 8'd0;
                            state   <= S_GEN_START;
                        end else begin
                            ind_idx <= ind_idx + 8'd1;
                        end
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                
                // Inicio de una generacion: reset de min_fitness_gen
                
                S_GEN_START: begin
                    min_fitness_gen <= 32'hFFFFFFFF;
                    elite_valid_gen <= 1'b0;
                    ind_idx         <= 8'd0;
                    state           <= S_EVAL_ANCHOR0_REQ;
                end

                
                // Anclas de la trayectoria (extremos), una vez por individuo
                
                S_EVAL_ANCHOR0_REQ: begin
                    traj_addr     <= 9'd0;
                    traj_wait_cnt <= 2'd2;
                    state         <= S_EVAL_ANCHOR0_WT;
                end
                S_EVAL_ANCHOR0_WT: begin
                    if (traj_wait_cnt == 2'd0) begin
                        anchor0_x <= traj_x; anchor0_y <= traj_y; anchor0_z <= traj_z;
                        state <= S_EVAL_ANCHOR1_REQ;
                    end else begin
                        traj_wait_cnt <= traj_wait_cnt - 2'd1;
                    end
                end
                S_EVAL_ANCHOR1_REQ: begin
                    traj_addr     <= N_REF[8:0] - 9'd1;
                    traj_wait_cnt <= 2'd2;
                    state         <= S_EVAL_ANCHOR1_WT;
                end
                S_EVAL_ANCHOR1_WT: begin
                    if (traj_wait_cnt == 2'd0) begin
                        anchor1_x <= traj_x; anchor1_y <= traj_y; anchor1_z <= traj_z;
                        pt_k  <= 4'd0;
                        state <= S_ORDER_PT_RX;
                    end else begin
                        traj_wait_cnt <= traj_wait_cnt - 2'd1;
                    end
                end

                
                // Leer el punto k del individuo (3 ciclos: x,y,z)
                
                S_ORDER_PT_RX: begin
                    gene_idx <= pt_k * 3;
                    state    <= S_ORDER_PT_RY;
                end
                S_ORDER_PT_RY: begin
                    cur_px   <= population[ind_idx * GENES_PER_IND + pt_k*3 + 0];
                    gene_idx <= pt_k * 3 + 1;
                    state    <= S_ORDER_PT_RZ;
                end
                S_ORDER_PT_RZ: begin
                    cur_py <= population[ind_idx * GENES_PER_IND + pt_k*3 + 1];
                    state  <= S_ORDER_INIT_SCAN;
                end
                S_ORDER_INIT_SCAN: begin
                    cur_pz        <= population[ind_idx * GENES_PER_IND + pt_k*3 + 2];
                    scan_min_d2   <= 32'hFFFFFFFF;
                    scan_min_idx  <= 9'd0;
                    ref_scan_idx  <= 9'd0;
                    traj_addr     <= 9'd0;
                    traj_wait_cnt <= 2'd2;
                    state         <= S_ORDER_SCAN_WT;
                end

                
                // Escaneo de los N_REF puntos de referencia buscando
                // el mas cercano a (cur_px,cur_py,cur_pz)
                
                S_ORDER_SCAN_REQ: begin
                    traj_addr     <= ref_scan_idx;
                    traj_wait_cnt <= 2'd2;
                    state         <= S_ORDER_SCAN_WT;
                end
                S_ORDER_SCAN_WT: begin
                    if (traj_wait_cnt == 2'd0) begin
                        sdx <= cur_px - traj_x;
                        sdy <= cur_py - traj_y;
                        sdz <= cur_pz - traj_z;
                        state <= S_ORDER_SCAN_CMP;
                    end else begin
                        traj_wait_cnt <= traj_wait_cnt - 2'd1;
                    end
                end
                S_ORDER_SCAN_CMP: begin
                    scan_sum_d2 = sdx*sdx + sdy*sdy + sdz*sdz;

                    if (scan_sum_d2 < scan_min_d2) begin
                        scan_min_d2  <= scan_sum_d2;
                        scan_min_idx <= ref_scan_idx;
                    end

                    if (ref_scan_idx == N_REF[8:0] - 9'd1) begin
                        state <= S_ORDER_PT_DONE;
                    end else begin
                        ref_scan_idx <= ref_scan_idx + 9'd1;
                        state        <= S_ORDER_SCAN_REQ;
                    end
                end

                S_ORDER_PT_DONE: begin
                    ord_x[pt_k]   <= cur_px;
                    ord_y[pt_k]   <= cur_py;
                    ord_z[pt_k]   <= cur_pz;
                    ord_idx[pt_k] <= scan_min_idx;

                    if (pt_k == N_POINTS - 1) begin
                        state <= S_SORT_INIT;
                    end else begin
                        pt_k  <= pt_k + 4'd1;
                        state <= S_ORDER_PT_RX;
                    end
                end

                
                // Insertion sort de los 10 puntos por ord_idx ascendente
                
                S_SORT_INIT: begin
                    sort_i <= 4'd1;
                    state  <= S_SORT_CMP;
                end
                S_SORT_CMP: begin
                    if (sort_i >= N_POINTS[3:0]) begin
                        state <= S_BUILD_VERT;
                    end else begin
                        sort_j <= sort_i;
                        state  <= S_SORT_STEP;
                    end
                end
                S_SORT_STEP: begin
                    if (sort_j > 4'd0 && ord_idx[sort_j-1] > ord_idx[sort_j]) begin
                        tmp_x = ord_x[sort_j-1];   tmp_y = ord_y[sort_j-1];
                        tmp_z = ord_z[sort_j-1];   tmp_idx = ord_idx[sort_j-1];

                        ord_x[sort_j-1]   <= ord_x[sort_j];
                        ord_y[sort_j-1]   <= ord_y[sort_j];
                        ord_z[sort_j-1]   <= ord_z[sort_j];
                        ord_idx[sort_j-1] <= ord_idx[sort_j];

                        ord_x[sort_j]   <= tmp_x;
                        ord_y[sort_j]   <= tmp_y;
                        ord_z[sort_j]   <= tmp_z;
                        ord_idx[sort_j] <= tmp_idx;

                        sort_j <= sort_j - 4'd1;
                        // se repite S_SORT_STEP con el nuevo sort_j
                    end else begin
                        sort_i <= sort_i + 4'd1;
                        state  <= S_SORT_CMP;
                    end
                end

                
                S_BUILD_VERT: begin
                    vert_x[0] <= anchor0_x; vert_y[0] <= anchor0_y; vert_z[0] <= anchor0_z;
                    vert_x[N_VERT-1] <= anchor1_x; vert_y[N_VERT-1] <= anchor1_y; vert_z[N_VERT-1] <= anchor1_z;
                    for (g = 0; g < N_POINTS; g = g + 1) begin
                        vert_x[g+1] <= ord_x[g];
                        vert_y[g+1] <= ord_y[g];
                        vert_z[g+1] <= ord_z[g];
                    end
                    sum_d2_min   <= 64'd0;
                    rmse_ref_idx <= 9'd0;
                    state        <= S_RMSE_REF_REQ;
                end

                
                // RMSE verdadero: para cada punto de referencia, min
                // distancia a los N_SEG segmentos
                
                S_RMSE_REF_REQ: begin
                    traj_addr     <= rmse_ref_idx;
                    traj_wait_cnt <= 2'd2;
                    state         <= S_RMSE_REF_WT;
                end
                S_RMSE_REF_WT: begin
                    if (traj_wait_cnt == 2'd0) begin
                        seg_min_d2 <= 32'hFFFFFFFF;
                        seg_idx    <= 4'd0;
                        state      <= S_RMSE_SEG_START;
                    end else begin
                        traj_wait_cnt <= traj_wait_cnt - 2'd1;
                    end
                end
                S_RMSE_SEG_START: begin
                    psd_start <= 1'b1;
                    state     <= S_RMSE_SEG_WAIT;
                end
                S_RMSE_SEG_WAIT: begin
                    if (psd_done) begin
                        if (psd_dist_sq < seg_min_d2) begin
                            seg_min_d2 <= psd_dist_sq;
                        end
                        state <= S_RMSE_SEG_NEXT;
                    end
                end
                S_RMSE_SEG_NEXT: begin
                    if (seg_idx == N_SEG[3:0] - 4'd1) begin
                        state <= S_RMSE_ACCUM;
                    end else begin
                        seg_idx <= seg_idx + 4'd1;
                        state   <= S_RMSE_SEG_START;
                    end
                end
                S_RMSE_ACCUM: begin
                    sum_d2_min <= sum_d2_min + {32'd0, seg_min_d2};
                    if (rmse_ref_idx == N_REF[8:0] - 9'd1) begin
                        state <= S_RMSE_SQRT_START;
                    end else begin
                        rmse_ref_idx <= rmse_ref_idx + 9'd1;
                        state        <= S_RMSE_REF_REQ;
                    end
                end

                
                S_RMSE_SQRT_START: begin
                    mse_temp   = sum_d2_min / N_REF;
                    sqrt_rad   <= mse_temp[31:0];
                    sqrt_start <= 1'b1;
                    state      <= S_RMSE_SQRT_WAIT;
                end
                S_RMSE_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        fitness[ind_idx] <= {16'd0, sqrt_root};
                        if ({16'd0, sqrt_root} < min_fitness_gen) begin
                            min_fitness_gen <= {16'd0, sqrt_root};
                        end
                        if ({16'd0, sqrt_root} < min_fitness) begin
                            min_fitness <= {16'd0, sqrt_root};
                        end
                        state <= S_EVAL_STORE;
                    end
                end

                
                S_EVAL_STORE: begin
                    // ¿Es el mejor de esta generacion hasta ahora? -> copiar a elite_genes
                    if (fitness[ind_idx] <= min_fitness_gen) begin
                        gene_idx <= 8'd0;
                        state    <= S_ELITE_COPY;
                    end else begin
                        state <= S_ELITE_NEXT_IND;
                    end
                end

                S_ELITE_COPY: begin
                    elite_genes[gene_idx] <= population[ind_idx * GENES_PER_IND + gene_idx];
                    if (gene_idx == GENES_PER_IND - 1) begin
                        elite_valid_gen <= 1'b1;
                        state <= S_ELITE_NEXT_IND;
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                S_ELITE_NEXT_IND: begin
                    if (ind_idx == POP_SIZE - 1) begin
                        ind_idx  <= 8'd0;
                        gene_idx <= 8'd0;
                        state    <= S_CROSS_TOURN_A;
                    end else begin
                        ind_idx <= ind_idx + 8'd1;
                        state   <= S_EVAL_ANCHOR0_REQ;
                    end
                end

                
                // Seleccion por torneo (tam=2) para cada padre
                
                S_CROSS_TOURN_A: begin
                    cand_a <= {3'd0, lfsr[4:0]};
                    cand_b <= {3'd0, lfsr[9:5]};
                    state  <= S_CROSS_TOURN_B;
                end
                S_CROSS_TOURN_B: begin
                    p1_idx <= (fitness[cand_a] <= fitness[cand_b]) ? cand_a : cand_b;
                    cand_a <= {3'd0, lfsr[14:10]};
                    cand_b <= {3'd0, lfsr[19:15]};
                    state  <= S_CROSS_TOURN_C;
                end

                S_CROSS_TOURN_C: begin
                    p2_idx   <= (fitness[cand_a] <= fitness[cand_b]) ? cand_a : cand_b;
                    gene_idx <= 8'd0;
                    state    <= S_CROSS_LOOP;
                end

                
                // Cruce (promedio) + mutacion, gen por gen
                
                S_CROSS_LOOP: begin
                    hijo_val = (population[p1_idx*GENES_PER_IND + gene_idx] +
                                population[p2_idx*GENES_PER_IND + gene_idx]) >>> 1;

                    mut_rand = lfsr[7:0];
                    if (mut_rand < 8'd51) begin // ~20% de probabilidad
                        hijo_val = clamp_gene(hijo_val + $signed(lfsr[23:16]));
                    end

                    population[addr_self] <= hijo_val;

                    if (gene_idx == GENES_PER_IND - 1) begin
                        if (ind_idx == POP_SIZE - 1) begin
                            state <= S_ELITE_WB;
                        end else begin
                            ind_idx  <= ind_idx + 8'd1;
                            gene_idx <= 8'd0;
                            state    <= S_CROSS_TOURN_A;
                        end
                    end else begin
                        gene_idx <= gene_idx + 8'd1;
                    end
                end

                
                // Elitismo: el mejor individuo de la generacion pasa
                // intacto a la posicion 0 de la nueva poblacion
                
                S_ELITE_WB: begin
                    if (elite_valid_gen) begin
                        population[gene_idx] <= elite_genes[gene_idx]; // ind_idx=0 => addr=gene_idx
                        if (gene_idx == GENES_PER_IND - 1) begin
                            state <= S_NEXT_GEN;
                        end else begin
                            gene_idx <= gene_idx + 8'd1;
                        end
                    end else begin
                        state <= S_NEXT_GEN;
                    end
                end

                
                S_NEXT_GEN: begin
                    if (gen_count == MAX_GENS - 1) begin
                        state <= S_FINISHED;
                    end else begin
                        gen_count <= gen_count + 1'b1;
                        ind_idx   <= 8'd0;
                        gene_idx  <= 8'd0;
                        state     <= S_GEN_START;
                    end
                end

                
                S_FINISHED: begin
                    done         <= 1'b1;
                    current_gen  <= gen_count;
                    best_fitness <= min_fitness;
                    if (!start) begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
