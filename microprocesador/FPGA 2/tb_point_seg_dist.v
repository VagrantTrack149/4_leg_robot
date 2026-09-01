`timescale 1ns/1ps
module tb_point_seg_dist;
    reg clk = 0, rst = 1, start = 0;
    reg signed [15:0] ax, ay, az, bx, by, bz, px, py, pz;
    wire [31:0] dist_sq;
    wire busy, done;

    integer errors = 0;

    point_seg_dist dut (
        .clk(clk), .rst(rst), .start(start),
        .ax(ax), .ay(ay), .az(az),
        .bx(bx), .by(by), .bz(bz),
        .px(px), .py(py), .pz(pz),
        .dist_sq(dist_sq), .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    task run_case(
        input signed [15:0] tax, tay, taz,
        input signed [15:0] tbx, tby, tbz,
        input signed [15:0] tpx, tpy, tpz,
        input [31:0] expected,
        input [200*8-1:0] label
    );
        begin
            @(negedge clk);
            ax=tax; ay=tay; az=taz;
            bx=tbx; by=tby; bz=tbz;
            px=tpx; py=tpy; pz=tpz;
            start = 1;
            @(negedge clk);
            start = 0;
            wait(done==1);
            @(negedge clk);
            if (dist_sq !== expected) begin
                $display("FALLO [%0s]: dist_sq=%0d esperado=%0d", label, dist_sq, expected);
                errors = errors + 1;
            end else begin
                $display("OK    [%0s]: dist_sq=%0d", label, dist_sq);
            end
        end
    endtask

    initial begin
        rst=1; start=0;
        #12 rst=0;

        // Segmento (0,0,0)-(10,0,0), proyeccion dentro del rango
        run_case(0,0,0, 10,0,0, 5,5,0, 25, "proyeccion_interior");

        // Punto antes de A -> clamp a A
        run_case(0,0,0, 10,0,0, -5,0,0, 25, "clamp_a_A");

        // Punto despues de B -> clamp a B
        run_case(0,0,0, 10,0,0, 15,0,0, 25, "clamp_a_B");

        // Segmento degenerado A==B
        run_case(3,3,3, 3,3,3, 6,3,3, 9, "segmento_degenerado");

        // Punto exactamente sobre el segmento
        run_case(0,0,0, 10,0,0, 5,0,0, 0, "sobre_el_segmento");

        // Segmento en diagonal 3D
        run_case(0,0,0, 3,4,0, 0,0,0, 0, "diagonal_en_A");

        if (errors==0)
            $display(">>> TODOS LOS TESTS DE point_seg_dist PASARON");
        else
            $display(">>> %0d TESTS FALLARON", errors);
        $finish;
    end
endmodule
