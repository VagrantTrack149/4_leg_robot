// =====================================================================
// trajectory_bank.v
// Contiene las 4 trayectorias de referencia (circulo, helice, lissajous,
// espiral), cada una con 3 ROMs (x,y,z), seleccionables por 'traj_sel':
//   00 = circulo, 01 = helice, 10 = lissajous, 11 = espiral
//
// Lectura sincrona de 1 ciclo de latencia: dado 'addr' (indice de punto
// de referencia, 0..N_REF-1) entrega ref_x/ref_y/ref_z del punto
// seleccionado, un ciclo despues.
// =====================================================================
module trajectory_bank #(
    parameter N_REF = 300
)(
    input  wire        clk,
    input  wire [1:0]  traj_sel,
    input  wire [8:0]  addr,
    output reg  signed [15:0] ref_x,
    output reg  signed [15:0] ref_y,
    output reg  signed [15:0] ref_z
);

    wire signed [15:0] x0, y0, z0; // circulo
    wire signed [15:0] x1, y1, z1; // helice
    wire signed [15:0] x2, y2, z2; // lissajous
    wire signed [15:0] x3, y3, z3; // espiral

    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_circulo_x.mem"))   rom_x0 (.clk(clk), .addr(addr), .data_out(x0));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_circulo_y.mem"))   rom_y0 (.clk(clk), .addr(addr), .data_out(y0));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_circulo_z.mem"))   rom_z0 (.clk(clk), .addr(addr), .data_out(z0));

    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_helice_x.mem"))    rom_x1 (.clk(clk), .addr(addr), .data_out(x1));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_helice_y.mem"))    rom_y1 (.clk(clk), .addr(addr), .data_out(y1));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_helice_z.mem"))    rom_z1 (.clk(clk), .addr(addr), .data_out(z1));

    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_lissajous_x.mem")) rom_x2 (.clk(clk), .addr(addr), .data_out(x2));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_lissajous_y.mem")) rom_y2 (.clk(clk), .addr(addr), .data_out(y2));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_lissajous_z.mem")) rom_z2 (.clk(clk), .addr(addr), .data_out(z2));

    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_espiral_x.mem"))   rom_x3 (.clk(clk), .addr(addr), .data_out(x3));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_espiral_y.mem"))   rom_y3 (.clk(clk), .addr(addr), .data_out(y3));
    trajectory_rom #(.N_REF(N_REF), .INIT_FILE("traj_espiral_z.mem"))   rom_z3 (.clk(clk), .addr(addr), .data_out(z3));

    // El mux se aplica sobre 'traj_sel' registrado un ciclo (misma
    // latencia que la salida de las ROMs) para que el select y el dato
    // queden alineados en el tiempo.
    reg [1:0] traj_sel_r;
    always @(posedge clk) traj_sel_r <= traj_sel;

    always @(*) begin
        case (traj_sel_r)
            2'b00: begin ref_x = x0; ref_y = y0; ref_z = z0; end
            2'b01: begin ref_x = x1; ref_y = y1; ref_z = z1; end
            2'b10: begin ref_x = x2; ref_y = y2; ref_z = z2; end
            default: begin ref_x = x3; ref_y = y3; ref_z = z3; end
        endcase
    end

endmodule
