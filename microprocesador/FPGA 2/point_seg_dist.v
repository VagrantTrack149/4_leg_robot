// =====================================================================
// point_seg_dist.v
// Distancia^2 de un punto P a un segmento A-B, con proyeccion y clamp
// en [0,1], igual que calcular_rmse_verdadero() en Python:
//
//   AB = B - A
//   den = |AB|^2
//   if den == 0: Q = A
//   else:
//       u = clamp(dot(P-A, AB) / den, 0, 1)
//       Q = A + u * AB
//   dist_sq = |P - Q|^2
//
// SCALE = 2^16 (fixed point Q0.16 para 'u').
// Usa div_seq solo cuando 0 < u < 1 (caso general); los casos u=0 y u=1
// se resuelven sin division.
// =====================================================================
module point_seg_dist (
    input  wire               clk,
    input  wire               rst,
    input  wire                start,

    input  wire signed [15:0] ax, ay, az,   // Punto A del segmento
    input  wire signed [15:0] bx, by, bz,   // Punto B del segmento
    input  wire signed [15:0] px, py, pz,   // Punto de referencia

    output reg  [31:0]        dist_sq,
    output reg                busy,
    output reg                done
);

    localparam SCALE_BITS = 16;
    localparam signed [31:0] SCALE = 32'sd65536;

    localparam S_IDLE      = 4'd0,
               S_DIFF      = 4'd1,
               S_DOTDEN    = 4'd2,
               S_DECIDE    = 4'd3,
               S_DIV_START = 4'd4,
               S_DIV_WAIT  = 4'd5,
               S_PROJ      = 4'd6,
               S_QDIFF     = 4'd7,
               S_QSQ       = 4'd8,
               S_DONE      = 4'd9;

    reg [3:0] state;

    reg signed [16:0] abx, aby, abz;   // B - A
    reg signed [16:0] apx, apy, apz;   // P - A

    reg [31:0] den;                    // |AB|^2  (siempre >=0)
    reg signed [31:0] dot;             // dot(AP,AB)  (puede ser negativo)

    reg [16:0] u_scaled;               // 0..SCALE, Q0.16

    reg signed [31:0] qx_rel, qy_rel, qz_rel; // u*AB antes de escalar (con redondeo)
    reg signed [16:0] qx, qy, qz;             // Q - origen (relativo a A), ya escalado

    reg signed [16:0] dpx, dpy, dpz;   // P - Q

    // --- Divisor secuencial compartido (64/64 -> 64) ---
    reg         div_start;
    reg  [63:0] div_dividend_r;
    reg  [63:0] div_divisor_r;
    wire [63:0] div_quot;
    wire        div_busy, div_done;

    div_seq #(.WIDTH(64)) u_div (
        .clk(clk), .rst(rst), .start(div_start),
        .dividend(div_dividend_r),
        .divisor(div_divisor_r),
        .quotient(div_quot),
        .busy(div_busy),
        .done(div_done)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state    <= S_IDLE;
            busy     <= 1'b0;
            done     <= 1'b0;
            dist_sq  <= 32'd0;
            div_start<= 1'b0;
        end else begin
            done      <= 1'b0;
            div_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy  <= 1'b1;
                        state <= S_DIFF;
                    end
                end

                S_DIFF: begin
                    abx <= bx - ax;
                    aby <= by - ay;
                    abz <= bz - az;
                    apx <= px - ax;
                    apy <= py - ay;
                    apz <= pz - az;
                    state <= S_DOTDEN;
                end

                S_DOTDEN: begin
                    den <= (abx*abx) + (aby*aby) + (abz*abz);
                    dot <= (apx*abx) + (apy*aby) + (apz*abz);
                    state <= S_DECIDE;
                end

                S_DECIDE: begin
                    if (den == 32'd0) begin
                        // A == B: el segmento colapsa a un punto
                        u_scaled <= 17'd0;
                        state    <= S_PROJ;
                    end else if (dot <= 32'sd0) begin
                        u_scaled <= 17'd0;
                        state    <= S_PROJ;
                    end else if (dot >= $signed(den)) begin
                        u_scaled <= SCALE[16:0];
                        state    <= S_PROJ;
                    end else begin
                        // 0 < u < 1: division real
                        div_dividend_r <= {32'd0, dot[31:0]} * 64'd65536;
                        div_divisor_r  <= {32'd0, den};
                        state          <= S_DIV_START;
                    end
                end

                S_DIV_START: begin
                    div_start <= 1'b1;   // pulso de 1 ciclo
                    state     <= S_DIV_WAIT;
                end

                S_DIV_WAIT: begin
                    if (div_done) begin
                        u_scaled <= div_quot[16:0];
                        state    <= S_PROJ;
                    end
                end

                S_PROJ: begin
                    // qx_rel = u_scaled * abx  (con redondeo al escalar)
                    qx_rel <= $signed({1'b0, u_scaled}) * abx;
                    qy_rel <= $signed({1'b0, u_scaled}) * aby;
                    qz_rel <= $signed({1'b0, u_scaled}) * abz;
                    state  <= S_QDIFF;
                end

                S_QDIFF: begin
                    // Q = A + (qx_rel + media_SCALE) >>> 16   (redondeo)
                    // NOTA: no usar ax[15:0]/ay[15:0]/az[15:0] aqui: un
                    // part-select de una señal signed se vuelve unsigned
                    // en Verilog y corrompe la suma cuando el termino
                    // desplazado es negativo.
                    qx <= ax + ((qx_rel + 32'sd32768) >>> 16);
                    qy <= ay + ((qy_rel + 32'sd32768) >>> 16);
                    qz <= az + ((qz_rel + 32'sd32768) >>> 16);
                    state <= S_QSQ;
                end

                S_QSQ: begin
                    dpx <= px - qx;
                    dpy <= py - qy;
                    dpz <= pz - qz;
                    state <= S_DONE;
                end

                S_DONE: begin
                    dist_sq <= (dpx*dpx) + (dpy*dpy) + (dpz*dpz);
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
