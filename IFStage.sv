module IFStage(
    input logic clk, rst_n,
    input logic branch_taken, 
    input logic flush_if,
    input logic stall_if,
    output IF_ID_t if_if_o,
);
    IF_ID_t if_id_l;

    always @(posedge clk) begin
        if (!rst_n) begin
            if_id_l.pc <= 32'h00000000;
            if_id_l.instr <= 32'h00000000;
            if_id_l.valid <= 1'b0;
        end else if (flush_if) begin
            if_id_l.valid <= 1'b0;
        end else if(stall_if) begin
            // Hold the current state
            if_id_l <= if_id_l;
        end else if (!stall_if) begin
            if_id_l.pc <= if_id_l.pc + 4;
            if_id_l.instr <= instr;
            if_id_l.valid <= 1'b1;
        end
    end

    assign if_if_o = if_id_l;

endmodule