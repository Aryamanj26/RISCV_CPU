`include "PipeTypes.svh"

module id_stage (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         stall_id,
    input  logic         flush_id,
    input  IF_ID_t       if_id_i,
    
    // Register file read ports (from top level)
    input  logic [31:0]  rf_rs1_data_i,
    input  logic [31:0]  rf_rs2_data_i,
    
    output ID_EX_t       id_ex_o
);

    ID_EX_t id_ex_l;

    // Decoder outputs
    logic [4:0]  rs1, rs2, rd;
    logic [31:0] imm;
    ctrl_t       ctrl;
    logic        illegal;

    decoder u_decoder (
        .instr_i  (if_id_i.instr),
        .rs1_o    (rs1),
        .rs2_o    (rs2),
        .rd_o     (rd),
        .imm_o    (imm),
        .ctrl_o   (ctrl),
        .illegal_o(illegal)
    );

    always_comb begin
        id_ex_l = '0;

        id_ex_l.valid      = if_id_i.valid & ~illegal;
        id_ex_l.pc         = if_id_i.pc;
        id_ex_l.rs1        = rs1;
        id_ex_l.rs2        = rs2;
        id_ex_l.rd         = rd;
        id_ex_l.imm        = imm;
        id_ex_l.ctrl       = ctrl;
        id_ex_l.rs1_data   = rf_rs1_data_i;
        id_ex_l.rs2_data   = rf_rs2_data_i;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_o <= '0;
        end else if (flush_id) begin
        id_ex_o.valid <= 1'b0;
        end else if (stall_id) begin
            id_ex_o <= id_ex_o;
        end else begin
            id_ex_o <= id_ex_l;
        end
    end

endmodule
