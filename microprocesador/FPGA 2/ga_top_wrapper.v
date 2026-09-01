//       00 = circulo, 01 = helice, 10 = lissajous, 11 = espiral

module ga_top_wrapper #(
    parameter GENES_PER_IND = 30,   // 10 puntos 3D (10 x 3 = 30 genes)
    parameter POP_SIZE      = 32,   // Tamaño de poblacion
    parameter MAX_GENS      = 200,  // Numero de generaciones
    parameter N_REF         = 300   // Puntos en la trayectoria de referencia
)(
    input  wire        CLOCK_50,    // Reloj 50 MHz DE1-SoC
    input  wire [0:0]  KEY,         // KEY[0] Reset
    input  wire [3:0]  SW,          // SW[0]=Start SW[1]=(sin uso) SW[3:2]=traj_sel
    output wire [9:0]  LEDR,        // Monitoreo
    output wire [31:0] best_fit_out, // RMSE final (unidades enteras)
    // Displays de 7 segmentos: muestran best_fit_out[23:0] en hex
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5
);

    wire rst      = ~KEY[0];
    wire start    = SW[0];
    wire [1:0] traj_sel = SW[3:2];

    wire        done_sig;
    wire [15:0] current_gen_sig;
    wire [31:0] best_rmse_sig;

    ga_engine_top #(
        .GENES_PER_IND(GENES_PER_IND),
        .POP_SIZE     (POP_SIZE),
        .MAX_GENS     (MAX_GENS),
        .N_REF        (N_REF)
    ) u_ga_core (
        .clk          (CLOCK_50),
        .rst          (rst),
        .start        (start),
        .traj_sel     (traj_sel),
        .done         (done_sig),
        .current_gen  (current_gen_sig),
        .best_fitness (best_rmse_sig)
    );

    assign LEDR[0]   = done_sig;
    assign LEDR[9:1] = current_gen_sig[8:0];
    assign best_fit_out = best_rmse_sig;
    wire blank_hex = ~done_sig;

    char_7seg u_hex0 (.value(best_fit_out[3:0]),   .blank(blank_hex), .seg(HEX0));
    char_7seg u_hex1 (.value(best_fit_out[7:4]),   .blank(blank_hex), .seg(HEX1));
    char_7seg u_hex2 (.value(best_fit_out[11:8]),  .blank(blank_hex), .seg(HEX2));
    char_7seg u_hex3 (.value(best_fit_out[15:12]), .blank(blank_hex), .seg(HEX3));
    char_7seg u_hex4 (.value(best_fit_out[19:16]), .blank(blank_hex), .seg(HEX4));
    char_7seg u_hex5 (.value(best_fit_out[23:20]), .blank(blank_hex), .seg(HEX5));

endmodule
