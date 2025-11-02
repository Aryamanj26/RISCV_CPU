module IFStage(
    input logic clk, rst_n,
    input logic branch_taken,
    input logic [31:0] branch_target,
    input logic flush_if,
    input logic stall_if,
    
    // Instruction memory interface
    output logic [31:0] imem_addr_o,
    input  logic [31:0] imem_data_i,
    
    output IF_ID_t if_id_o
);
    // Separate PC register (not part of pipeline register)
    logic [31:0] pc_reg;
    logic [31:0] pc_next;
    
    // Local IF/ID struct for combinational logic
    IF_ID_t if_id_local;
    
    //===========================================================================
    // PC Register (updates every cycle unless stalled)
    //===========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pc_reg <= 32'h00000000;
        end else if (!stall_if) begin
            pc_reg <= pc_next;
        end
        // When stalled, PC holds its value
    end
    
    //===========================================================================
    // PC Next Logic (Combinational)
    //===========================================================================
    always_comb begin
        if (branch_taken) begin
            pc_next = branch_target;
        end else begin
            pc_next = pc_reg + 4;
        end
    end
    
    //===========================================================================
    // Instruction Memory Interface
    //===========================================================================
    assign imem_addr_o = pc_reg;
    
    //===========================================================================
    // Combinational preparation of IF/ID data
    //===========================================================================
    always_comb begin
        if_id_local.pc = pc_reg;
        if_id_local.instr = imem_data_i;
        if_id_local.valid = 1'b1;  // Will be overridden by flush/reset in FF
    end
    
    //===========================================================================
    // IF/ID Pipeline Register (just latches the local struct)
    //===========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            if_id_o.pc <= 32'h00000000;
            if_id_o.instr <= 32'h00000013; // NOP
            if_id_o.valid <= 1'b0;
        end else if (flush_if) begin
            if_id_o.instr <= 32'h00000013; // NOP
            if_id_o.valid <= 1'b0;
            if_id_o.pc <= if_id_local.pc;  // Keep PC for debugging
        end else if (stall_if) begin
            if_id_o <= if_id_o;  // Hold current value
        end else begin
            if_id_o <= if_id_local;  // Latch the combinational struct
        end
    end

endmodule