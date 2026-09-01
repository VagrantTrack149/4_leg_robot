module fitness_dist_sq_3d (
    input  wire        clk,
    input  wire        rst,
    // Coordenadas Punto 1 (Gene/Individuo)
    input  wire signed [15:0] p1_x,
    input  wire signed [15:0] p1_y,
    input  wire signed [15:0] p1_z,
    // Coordenadas Punto 2 (Referencia)
    input  wire signed [15:0] p2_x,
    input  wire signed [15:0] p2_y,
    input  wire signed [15:0] p2_z,
    // Resultado Distancia Cuadrada
    output reg  [31:0] dist_sq
);

    // Registros intermedios para la resta (Diferencia)
    reg signed [16:0] diff_x, diff_y, diff_z;
    
    // Registros para los productos (Cuadrados)
    reg [31:0] sq_x, sq_y, sq_z;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            diff_x  <= 17'd0;
            diff_y  <= 17'd0;
            diff_z  <= 17'd0;
            sq_x    <= 32'd0;
            sq_y    <= 32'd0;
            sq_z    <= 32'd0;
            dist_sq <= 32'd0;
        end else begin
            // Etapa 1: Diferencia
            diff_x <= p1_x - p2_x;
            diff_y <= p1_y - p2_y;
            diff_z <= p1_z - p2_z;

            // Etapa 2: Elevación al cuadrado
            sq_x <= diff_x * diff_x;
            sq_y <= diff_y * diff_y;
            sq_z <= diff_z * diff_z;

            // Etapa 3: Acumulación de distancia
            dist_sq <= sq_x + sq_y + sq_z;
        end
    end

endmodule