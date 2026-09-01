module div_seq #(
    parameter WIDTH = 64
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             start,      // pulso de 1 ciclo para iniciar
    input  wire [WIDTH-1:0] dividend,
    input  wire [WIDTH-1:0] divisor,    // se asume != 0 (verificar antes de start)
    output reg  [WIDTH-1:0] quotient,
    output reg               busy,
    output reg               done       // pulso de 1 ciclo cuando termina
);

    localparam CNT_W = $clog2(WIDTH+1);

    reg [WIDTH-1:0]   rem;
    reg [WIDTH-1:0]   div_shift;   // "mitad baja" del registro combinado
    reg [CNT_W-1:0]   count;
    reg [WIDTH-1:0]   divisor_r;

    reg [WIDTH-1:0]   rem_next;
    reg [WIDTH-1:0]   div_shift_next;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            rem       <= {WIDTH{1'b0}};
            div_shift <= {WIDTH{1'b0}};
            quotient  <= {WIDTH{1'b0}};
            count     <= {CNT_W{1'b0}};
            divisor_r <= {WIDTH{1'b0}};
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                busy      <= 1'b1;
                rem       <= {WIDTH{1'b0}};
                div_shift <= dividend;
                divisor_r <= divisor;
                count     <= WIDTH[CNT_W-1:0];
            end else if (busy) begin
                // Desplazar {rem, div_shift} un bit a la izquierda (registro combinado 2*WIDTH)
                rem_next       = {rem[WIDTH-2:0], div_shift[WIDTH-1]};
                div_shift_next = {div_shift[WIDTH-2:0], 1'b0};

                if (rem_next >= divisor_r) begin
                    rem_next       = rem_next - divisor_r;
                    div_shift_next = div_shift_next | {{(WIDTH-1){1'b0}}, 1'b1};
                end

                rem       <= rem_next;
                div_shift <= div_shift_next;

                if (count == {{(CNT_W-1){1'b0}}, 1'b1}) begin
                    busy     <= 1'b0;
                    done     <= 1'b1;
                    quotient <= div_shift_next;
                end else begin
                    count <= count - 1'b1;
                end
            end
        end
    end

endmodule
