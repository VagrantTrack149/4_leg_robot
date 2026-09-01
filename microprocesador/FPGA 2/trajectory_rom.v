// =====================================================================
// trajectory_rom.v
// ROM de solo lectura para UN eje (x, y o z) de una trayectoria de
// referencia de N_REF puntos, 16 bits con signo cada uno.
// Contenido inicializado desde un archivo .mem via $readmemh
// (generado por scripts/gen_trajectories.py).
// =====================================================================
module trajectory_rom #(
    parameter N_REF   = 300,
    parameter INIT_FILE = "traj_circulo_x.mem"
)(
    input  wire        clk,
    input  wire [8:0]  addr,     // 0 .. N_REF-1  (9 bits cubren hasta 511)
    output reg  signed [15:0] data_out
);

    reg signed [15:0] mem [0:N_REF-1];

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        data_out <= mem[addr];
    end

endmodule
