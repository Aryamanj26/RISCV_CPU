module IFStage(
    input logic clk, rst_n,
    input logic branch_taken,
    input logic [31:0] branch_target,
    input logic flush_if,
    input logic stall_if,
    
    // Instruction memory interface
    output logic [31:0] imem_addr_o,
    input  logic [31:0] imem_data_i,
    
    output IF_ID_t if_if_o
);
    IF_ID_t if_id_l;
    logic [31:0] pc_next;
    
    // PC next logic
    always_comb begin
        if (branch_taken) begin
            pc_next = branch_target;
        end else begin
            pc_next = if_id_l.pc + 4;
        end
    end
    
    // Request instruction from external memory
    assign imem_addr_o = if_id_l.pc;

    // Pipeline register
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            if_id_l.pc <= 32'h00000000;
            if_id_l.instr <= 32'h00000013; // NOP
            if_id_l.valid <= 1'b0;
        end else if (flush_if) begin
            if_id_l.valid <= 1'b0;
        end else if (stall_if) begin
            if_id_l <= if_id_l;
        end else begin
            if_id_l.pc <= pc_next;
            if_id_l.instr <= imem_data_i;
            if_id_l.valid <= 1'b1;
        end
    end

    assign if_if_o = if_id_l;

endmodule