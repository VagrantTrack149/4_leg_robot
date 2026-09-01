`timescale 1ns/1ps
module tb_ga_engine_small;
    reg clk = 0, rst = 1, start = 0;
    reg [1:0] traj_sel = 2'b00;
    wire done;
    wire [15:0] current_gen;
    wire [31:0] best_fitness;

    ga_engine_top #(
        .GENES_PER_IND(6),   // 2 puntos por individuo
        .POP_SIZE(4),
        .MAX_GENS(2),
        .N_REF(5)
    ) dut (
        .clk(clk), .rst(rst), .start(start), .traj_sel(traj_sel),
        .done(done), .current_gen(current_gen), .best_fitness(best_fitness)
    );

    always #5 clk = ~clk;

    integer cycles;

    initial begin
        rst = 1; start = 0;
        #20 rst = 0;
        #10 start = 1;

        cycles = 0;
        while (!done && cycles < 2_000_000) begin
            @(posedge clk);
            cycles = cycles + 1;
        end

        if (done) begin
            $display(">>> DONE en %0d ciclos. gen=%0d best_fitness=%0d", cycles, current_gen, best_fitness);
        end else begin
            $display(">>> TIMEOUT: la FSM no termino en %0d ciclos (posible cuelgue)", cycles);
        end
        $finish;
    end

    // Watchdog de estado para depuracion (imprime cambios de estado)
    reg [5:0] prev_state;
    always @(posedge clk) begin
        if (dut.state !== prev_state) begin
            prev_state <= dut.state;
        end
    end
endmodule
