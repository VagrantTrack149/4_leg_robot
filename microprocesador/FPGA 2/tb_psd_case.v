`timescale 1ns/1ps
module tb_psd_case;
    reg clk=0, rst=1, start=0;
    reg signed [15:0] ax,ay,az,bx,by,bz,px,py,pz;
    wire [31:0] dist_sq;
    wire busy,done;

    point_seg_dist dut(.clk(clk),.rst(rst),.start(start),
        .ax(ax),.ay(ay),.az(az),.bx(bx),.by(by),.bz(bz),.px(px),.py(py),.pz(pz),
        .dist_sq(dist_sq),.busy(busy),.done(done));

    always #5 clk=~clk;

    initial begin
        rst=1; start=0; #12 rst=0;
        @(negedge clk);
        ax=0; ay=500; az=0;
        bx=996; by=84; bz=0;
        px=1000; py=0; pz=0;
        start=1;
        @(negedge clk); start=0;
        wait(done==1);
        @(negedge clk);
        $display("dist_sq=%0d (esperado 7072)", dist_sq);
        $display("internos: den=%0d dot=%0d u_scaled=%0d qy_rel=%0d qx=%0d qy=%0d qz=%0d",
            dut.den, dut.dot, dut.u_scaled, dut.qy_rel, dut.qx, dut.qy, dut.qz);
        $finish;
    end
endmodule
