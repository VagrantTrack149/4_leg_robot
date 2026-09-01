module lfsr_32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    output reg  [31:0] rnd
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rnd <= 32'hACE12678; // Semilla inicial
        end else if (enable) begin
            // Polinomio x^32 + x^22 + x^2 + x + 1
            rnd <= {rnd[30:0], rnd[31] ^ rnd[21] ^ rnd[1] ^ rnd[0]};
        end
    end
endmodule