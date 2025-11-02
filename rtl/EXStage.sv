`include "PipeTypes.svh"

module ex_stage (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         stall_ex,
    input  logic         flush_ex,
    input  ID_EX_t       id_ex_i,
    output EX_MEM_t      ex_mem_o,
    output logic         branch_taken_o,
    output logic [31:0]  branch_target_o
);

    EX_MEM_t ex_mem_l;
    
    // ALU signals
    logic [31:0] alu_a, alu_b;
    logic [31:0] alu_result;
    
    // Branch logic
    logic branch_taken;
    logic [31:0] branch_target;

    // ALU instantiation
    alu u_alu (
        .operand_a_i (alu_a),
        .operand_b_i (alu_b),
        .alu_op_i    (id_ex_i.ctrl.alu_op),
        .result_o    (alu_result)
    );

    // Operand A mux
    always_comb begin
        case (id_ex_i.ctrl.op_a_sel)
            OP_A_RS1:  alu_a = id_ex_i.rs1_data;
            OP_A_PC:   alu_a = id_ex_i.pc;
            OP_A_ZERO: alu_a = 32'h0;
            default:   alu_a = 32'h0;
        endcase
    end

    // Operand B mux
    always_comb begin
        case (id_ex_i.ctrl.op_b_sel)
            OP_B_RS2: alu_b = id_ex_i.rs2_data;
            OP_B_IMM: alu_b = id_ex_i.imm;
            default:  alu_b = 32'h0;
        endcase
    end

    // Branch logic
    always_comb begin
        branch_taken = 1'b0;
        case (id_ex_i.ctrl.branch_kind)
            BR_BEQ:  branch_taken = (id_ex_i.rs1_data == id_ex_i.rs2_data);
            BR_BNE:  branch_taken = (id_ex_i.rs1_data != id_ex_i.rs2_data);
            BR_BLT:  branch_taken = ($signed(id_ex_i.rs1_data) < $signed(id_ex_i.rs2_data));
            BR_BGE:  branch_taken = ($signed(id_ex_i.rs1_data) >= $signed(id_ex_i.rs2_data));
            BR_BLTU: branch_taken = (id_ex_i.rs1_data < id_ex_i.rs2_data);
            BR_BGEU: branch_taken = (id_ex_i.rs1_data >= id_ex_i.rs2_data);
            default: branch_taken = 1'b0;
        endcase
    end

    // Branch/Jump target calculation
    always_comb begin
        if (id_ex_i.ctrl.jump_kind == JMP_JALR) begin
            branch_target = (id_ex_i.rs1_data + id_ex_i.imm) & ~32'h1; // JALR: (rs1 + imm) & ~1
        end else begin
            branch_target = id_ex_i.pc + id_ex_i.imm; // Branch or JAL: PC + imm
        end
    end

    // Byte enable generation (wstrb) for stores
    logic [3:0] wstrb;
    always_comb begin
        wstrb = 4'b1111; // Default: word access
        // Can be extended based on funct3 for byte/halfword stores
    end

    // Output to branch predictor/IF stage
    assign branch_taken_o = (branch_taken | (id_ex_i.ctrl.jump_kind != JMP_NONE)) & id_ex_i.valid;
    assign branch_target_o = branch_target;

    // Combinational stage output
    always_comb begin
        ex_mem_l = '0;
        ex_mem_l.valid         = id_ex_i.valid;
        ex_mem_l.pc            = id_ex_i.pc;
        ex_mem_l.alu_res       = alu_result;
        ex_mem_l.rs2_data      = id_ex_i.rs2_data;
        ex_mem_l.rd            = id_ex_i.rd;
        ex_mem_l.ctrl          = id_ex_i.ctrl;
        ex_mem_l.branch_taken  = branch_taken;
        ex_mem_l.branch_target = branch_target;
        ex_mem_l.wstrb         = wstrb;
    end

    // Sequential stage register
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ex_mem_o <= '0;
        end else if (flush_ex) begin
            ex_mem_o.valid <= 1'b0;
        end else if (stall_ex) begin
            ex_mem_o <= ex_mem_o;
        end else begin
            ex_mem_o <= ex_mem_l;
        end
    end

endmodule
