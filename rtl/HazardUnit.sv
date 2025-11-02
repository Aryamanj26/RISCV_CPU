`include "PipeTypes.svh"

// Hazard Detection and Forwarding Unit
module hazard_unit (
    // Pipeline stage inputs
    input  IF_ID_t   if_id_i,
    input  ID_EX_t   id_ex_i,
    input  EX_MEM_t  ex_mem_i,
    input  MEM_WB_t  mem_wb_i,
    
    // Branch/Jump control
    input  logic     branch_taken_i,
    
    // Hazard control outputs
    output logic     stall_if_o,
    output logic     stall_id_o,
    output logic     stall_ex_o,
    output logic     flush_if_o,
    output logic     flush_id_o,
    output logic     flush_ex_o,
    
    // Forwarding control outputs
    output logic [1:0] forward_a_o,  // 00: no forward, 01: from EX/MEM, 10: from MEM/WB
    output logic [1:0] forward_b_o
);

    logic load_use_hazard;
    logic [4:0] rs1_id, rs2_id;
    
    assign rs1_id = if_id_i.instr[19:15];
    assign rs2_id = if_id_i.instr[24:20];

    //===========================================================================
    // Load-Use Hazard Detection
    //===========================================================================
    always_comb begin
        load_use_hazard = 1'b0;
        
        if (id_ex_i.ctrl.mem_read && id_ex_i.valid) begin
            if ((id_ex_i.rd == rs1_id && rs1_id != 5'b0) ||
                (id_ex_i.rd == rs2_id && rs2_id != 5'b0)) begin
                load_use_hazard = 1'b1;
            end
        end
    end

    //===========================================================================
    // Stall Control
    //===========================================================================
    always_comb begin
        stall_if_o = 1'b0;
        stall_id_o = 1'b0;
        stall_ex_o = 1'b0;
        
        if (load_use_hazard) begin
            stall_if_o = 1'b1;  // Stall IF stage
            stall_id_o = 1'b1;  // Stall ID stage
        end
    end

    //===========================================================================
    // Flush Control (for control hazards)
    //===========================================================================
    always_comb begin
        flush_if_o = 1'b0;
        flush_id_o = 1'b0;
        flush_ex_o = 1'b0;
        
        if (branch_taken_i) begin
            flush_if_o = 1'b1;  // Flush IF stage
            flush_id_o = 1'b1;  // Flush ID stage
            flush_ex_o = 1'b1;  // Flush EX stage
        end
    end

    //===========================================================================
    // Forwarding Logic (EX stage operand forwarding)
    //===========================================================================
    
    // Forward A (rs1)
    always_comb begin
        forward_a_o = 2'b00;  // No forwarding by default
        
        // Forward from MEM/WB stage
        if (mem_wb_i.ctrl.reg_write && mem_wb_i.valid &&
            (mem_wb_i.rd != 5'b0) && (mem_wb_i.rd == id_ex_i.rs1)) begin
            forward_a_o = 2'b10;
        end
        
        // Forward from EX/MEM stage (higher priority)
        if (ex_mem_i.ctrl.reg_write && ex_mem_i.valid &&
            (ex_mem_i.rd != 5'b0) && (ex_mem_i.rd == id_ex_i.rs1)) begin
            forward_a_o = 2'b01;
        end
    end

    // Forward B (rs2)
    always_comb begin
        forward_b_o = 2'b00;  // No forwarding by default
        
        // Forward from MEM/WB stage
        if (mem_wb_i.ctrl.reg_write && mem_wb_i.valid &&
            (mem_wb_i.rd != 5'b0) && (mem_wb_i.rd == id_ex_i.rs2)) begin
            forward_b_o = 2'b10;
        end
        
        // Forward from EX/MEM stage (higher priority)
        if (ex_mem_i.ctrl.reg_write && ex_mem_i.valid &&
            (ex_mem_i.rd != 5'b0) && (ex_mem_i.rd == id_ex_i.rs2)) begin
            forward_b_o = 2'b01;
        end
    end

endmodule
