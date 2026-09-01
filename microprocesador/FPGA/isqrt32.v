// Algoritmo de raíz cuadrada entera secuencial en 16 ciclos (Ultra-ligero en área)
module isqrt32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [31:0] rad,
    output reg  [15:0] root,
    output reg         done
);

    reg [31:0] rem;
    reg [31:0] root_temp;
    reg [31:0] q;
    reg [4:0]  count;
    reg        busy;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rem       <= 32'd0;
            root_temp <= 32'd0;
            q         <= 32'd0;
            count     <= 5'd0;
            busy      <= 1'b0;
            done      <= 1'b0;
            root      <= 16'd0;
        end else begin
            if (start && !busy) begin
                busy      <= 1'b1;
                done      <= 1'b0;
                rem       <= 32'd0;
                root_temp <= 32'd0;
                q         <= rad;
                count     <= 5'd0;
            end else if (busy) begin
                if (count == 5'd16) begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    root  <= root_temp[15:0];
                end else begin
                    // Paso del algoritmo no restaurativo
                    if (((rem << 2) | ((q >> 30) & 2'b11)) >= ((root_temp << 1) + 1)) begin
                        rem       <= ((rem << 2) | ((q >> 30) & 2'b11)) - ((root_temp << 1) + 1);
                        root_temp <= (root_temp << 1) + 1;
                    end else begin
                        rem       <= (rem << 2) | ((q >> 30) & 2'b11);
                        root_temp <= (root_temp << 1);
                    end
                    q     <= q << 2;
                    count <= count + 1'b1;
                end
            end else begin
                done <= 1'b0;
            end
        end
    end
endmodule