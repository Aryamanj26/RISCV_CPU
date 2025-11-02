`include "PipeTypes.svh"

module wb_stage (
    input  logic         clk,
    input  logic         rst_n,
    input  MEM_WB_t      mem_wb_i,
    
    // Writeback to register file
    output logic         reg_write_o,
    output logic [4:0]   rd_addr_o,
    output logic [31:0]  rd_data_o
);

    logic [31:0] wb_data;

    // Writeback data mux
    always_comb begin
        case (mem_wb_i.ctrl.wb_sel)
            WB_ALU: wb_data = mem_wb_i.alu_res;
            WB_MEM: wb_data = mem_wb_i.mem_data;
            WB_PC:  wb_data = mem_wb_i.pc + 32'd4;
            default: wb_data = 32'h0;
        endcase
    end

    // Writeback outputs
    assign reg_write_o = mem_wb_i.ctrl.reg_write & mem_wb_i.valid;
    assign rd_addr_o   = mem_wb_i.rd;
    assign rd_data_o   = wb_data;

endmodule
