`timescale 1ns/1ps
module tb_trajectory_bank;
    reg clk = 0;
    reg [1:0] traj_sel;
    reg [8:0] addr;
    wire signed [15:0] ref_x, ref_y, ref_z;

    trajectory_bank #(.N_REF(300)) dut (
        .clk(clk), .traj_sel(traj_sel), .addr(addr),
        .ref_x(ref_x), .ref_y(ref_y), .ref_z(ref_z)
    );

    always #5 clk = ~clk;

    initial begin
        traj_sel = 2'b00; // circulo
        addr = 9'd0;
        @(negedge clk);
        @(negedge clk); // 2 ciclos de latencia (rom + mux registrado)
        $display("circulo[0]   x=%0d y=%0d z=%0d (esperado x=1000 y=0 z=0)", ref_x, ref_y, ref_z);

        traj_sel = 2'b01; // helice
        addr = 9'd0;
        @(negedge clk);
        @(negedge clk);
        $display("helice[0]    x=%0d y=%0d z=%0d (esperado x=1000 y=0 z=0)", ref_x, ref_y, ref_z);

        traj_sel = 2'b11; // espiral
        addr = 9'd0;
        @(negedge clk);
        @(negedge clk);
        $display("espiral[0]   x=%0d y=%0d z=%0d (esperado x=100 y=0 z=0)", ref_x, ref_y, ref_z);

        traj_sel = 2'b10; // lissajous
        addr = 9'd299;
        @(negedge clk);
        @(negedge clk);
        $display("lissajous[299] x=%0d y=%0d z=%0d", ref_x, ref_y, ref_z);

        $finish;
    end
endmodule
