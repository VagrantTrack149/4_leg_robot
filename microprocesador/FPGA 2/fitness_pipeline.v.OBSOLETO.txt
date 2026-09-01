module fitness_pipeline #(
    parameter N_POINTS = 10
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        valid_in,
    input  wire [299:0] genes_in,
    output reg         valid_out,
    output wire [31:0] rmse_scaled_out
);

    wire        dist_valid;
    wire [31:0] dist_sq_out;

    // 1. Suma de distancias 3D al cuadrado
    fitness_dist_sq_3d #(
        .N_POINTS(N_POINTS)
    ) u_dist_sq (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_in),
        .genes_in(genes_in),
        .valid_out(dist_valid),
        .sum_sq_out(dist_sq_out)
    );

    // 2. Cálculo del MSE (Mean Squared Error)
    wire [31:0] mse = dist_sq_out / N_POINTS;

    // 3. Raíz Cuadrada Secuencial
    wire [15:0] rmse_raw;
    wire        sqrt_done;

    isqrt32 u_sqrt (
        .clk(clk),
        .rst(rst),
        .start(dist_valid),
        .rad(mse),
        .root(rmse_raw),
        .done(sqrt_done)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            valid_out <= 1'b0;
        end else begin
            valid_out <= sqrt_done;
        end
    end

    assign rmse_scaled_out = {16'd0, rmse_raw};

endmodule