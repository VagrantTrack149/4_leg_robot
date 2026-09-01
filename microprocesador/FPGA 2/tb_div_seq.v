`timescale 1ns/1ps
module tb_div_seq;
    localparam WIDTH = 64;
    reg clk = 0, rst = 1, start = 0;
    reg [WIDTH-1:0] dividend, divisor;
    wire [WIDTH-1:0] quotient;
    wire busy, done;

    integer errors = 0;
    integer i;

    div_seq #(.WIDTH(WIDTH)) dut (
        .clk(clk), .rst(rst), .start(start),
        .dividend(dividend), .divisor(divisor),
        .quotient(quotient), .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    task run_div(input [63:0] a, input [63:0] b);
        reg [63:0] expected;
        begin
            expected = a / b;
            @(negedge clk);
            dividend = a;
            divisor  = b;
            start = 1;
            @(negedge clk);
            start = 0;
            wait(done == 1);
            @(negedge clk);
            if (quotient !== expected) begin
                $display("FALLO: %0d / %0d = %0d (esperado %0d)", a, b, quotient, expected);
                errors = errors + 1;
            end else begin
                $display("OK:    %0d / %0d = %0d", a, b, quotient);
            end
        end
    endtask

    initial begin
        rst = 1;
        start = 0;
        dividend = 0;
        divisor = 0;
        #12 rst = 0;

        run_div(64'd0, 64'd7);
        run_div(64'd7, 64'd7);
        run_div(64'd100, 64'd3);
        run_div(64'd1966080000000, 64'd300000000); // caso tipico: dot*SCALE / den
        run_div(64'd65536, 64'd1);
        run_div(64'hFFFFFFFF, 64'd1);
        run_div(64'd123456789, 64'd987);
        run_div(64'd1, 64'd65536);

        if (errors == 0)
            $display(">>> TODOS LOS TESTS DE div_seq PASARON");
        else
            $display(">>> %0d TESTS FALLARON", errors);

        $finish;
    end
endmodule
