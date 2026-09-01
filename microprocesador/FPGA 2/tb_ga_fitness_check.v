`timescale 1ns/1ps
module tb_ga_fitness_check;
    reg clk = 0, rst = 1, start = 0;
    reg [1:0] traj_sel = 2'b00;
    wire done;
    wire [15:0] current_gen;
    wire [31:0] best_fitness;

    ga_engine_top #(
        .GENES_PER_IND(6),   // 2 puntos por individuo
        .POP_SIZE(2),
        .MAX_GENS(1),
        .N_REF(5)
    ) dut (
        .clk(clk), .rst(rst), .start(start), .traj_sel(traj_sel),
        .done(done), .current_gen(current_gen), .best_fitness(best_fitness)
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (dut.pt_k == 4'd1 && dut.ind_idx==8'd0 && dut.state==6'd13) begin
            $display("t=%0t CMP ref_scan_idx=%0d sdx=%0d sdy=%0d sdz=%0d sum=%0d min_d2=%0d min_idx=%0d",
                $time, dut.ref_scan_idx, dut.sdx, dut.sdy, dut.sdz,
                (dut.sdx*dut.sdx + dut.sdy*dut.sdy + dut.sdz*dut.sdz),
                dut.scan_min_d2, dut.scan_min_idx);
        end
        if (dut.ind_idx==8'd0 && dut.state==6'd22 && dut.psd_done) begin
            $display("t=%0t RMSE ref_idx=%0d seg_idx=%0d psd_dist_sq=%0d seg_min_d2(prev)=%0d",
                $time, dut.rmse_ref_idx, dut.seg_idx, dut.psd_dist_sq, dut.seg_min_d2);
        end
        if (dut.ind_idx==8'd0 && dut.state==6'd24) begin
            $display("t=%0t ACCUM ref_idx=%0d seg_min_d2=%0d sum_d2_min(prev)=%0d",
                $time, dut.rmse_ref_idx, dut.seg_min_d2, dut.sum_d2_min);
        end
    end

    initial begin
        rst = 1; start = 0;
        #20 rst = 0;

        // Forzar los genes del individuo 0 ANTES de start, para que
        // STATE_INIT_POP los sobreescriba... en vez de eso, dejamos que
        // corra normalmente y forzamos DESPUES de INIT_POP, justo antes
        // de que arranque la evaluacion.
        #10 start = 1;

        // Esperar a que termine la inicializacion aleatoria (estado GEN_START)
        wait (dut.state == 6'd2); // S_GEN_START

        // Depositar individuo 0 = punto(0)=(500,0,0) punto(1)=(0,500,0)
        @(negedge clk);
        dut.population[0] = 16'sd500; // x0
        dut.population[1] = 16'sd0;   // y0
        dut.population[2] = 16'sd0;   // z0
        dut.population[3] = 16'sd0;   // x1
        dut.population[4] = 16'sd500; // y1
        dut.population[5] = 16'sd0;   // z1

        // Esperar a que se calcule fitness[0]
        wait (dut.ind_idx == 8'd0 && dut.state == 6'd29); // S_ELITE_NEXT_IND tras individuo 0
        $display("fitness[0] (HW) = %0d", dut.fitness[0]);
        $display("ord_idx[0]=%0d ord_idx[1]=%0d", dut.ord_idx[0], dut.ord_idx[1]);
        $display("vert0=(%0d,%0d,%0d)", dut.vert_x[0], dut.vert_y[0], dut.vert_z[0]);
        $display("vert1=(%0d,%0d,%0d)", dut.vert_x[1], dut.vert_y[1], dut.vert_z[1]);
        $display("vert2=(%0d,%0d,%0d)", dut.vert_x[2], dut.vert_y[2], dut.vert_z[2]);
        $display("vert3=(%0d,%0d,%0d)", dut.vert_x[3], dut.vert_y[3], dut.vert_z[3]);
        $display("sum_d2_min=%0d", dut.sum_d2_min);

        wait(done);
        $display("done. best_fitness=%0d", best_fitness);
        $finish;
    end
endmodule
