`include "PipeTypes.svh"

// ALU Module - Arithmetic Logic Unit
// Modular design for easy extension of operations
module alu (
    input  logic [31:0]  operand_a_i,
    input  logic [31:0]  operand_b_i,
    input  alu_op_t      alu_op_i,
    output logic [31:0]  result_o
);

    // Intermediate results for different operation categories
    logic [31:0] arith_result;
    logic [31:0] logic_result;
    logic [31:0] shift_result;
    logic [31:0] compare_result;

    // Arithmetic operations
    always_comb begin
        case (alu_op_i)
            ALU_ADD: arith_result = operand_a_i + operand_b_i;
            ALU_SUB: arith_result = operand_a_i - operand_b_i;
            default: arith_result = 32'h0;
        endcase
    end

    // Logical operations
    always_comb begin
        case (alu_op_i)
            ALU_AND: logic_result = operand_a_i & operand_b_i;
            ALU_OR:  logic_result = operand_a_i | operand_b_i;
            ALU_XOR: logic_result = operand_a_i ^ operand_b_i;
            default: logic_result = 32'h0;
        endcase
    end

    // Shift operations
    always_comb begin
        case (alu_op_i)
            ALU_SLL: shift_result = operand_a_i << operand_b_i[4:0];
            ALU_SRL: shift_result = operand_a_i >> operand_b_i[4:0];
            ALU_SRA: shift_result = $signed(operand_a_i) >>> operand_b_i[4:0];
            default: shift_result = 32'h0;
        endcase
    end

    // Comparison operations
    always_comb begin
        case (alu_op_i)
            ALU_SLT:  compare_result = ($signed(operand_a_i) < $signed(operand_b_i)) ? 32'd1 : 32'd0;
            ALU_SLTU: compare_result = (operand_a_i < operand_b_i) ? 32'd1 : 32'd0;
            default:  compare_result = 32'h0;
        endcase
    end

    // Final result mux - select based on operation category
    always_comb begin
        case (alu_op_i)
            ALU_ADD, ALU_SUB:           result_o = arith_result;
            ALU_AND, ALU_OR, ALU_XOR:   result_o = logic_result;
            ALU_SLL, ALU_SRL, ALU_SRA:  result_o = shift_result;
            ALU_SLT, ALU_SLTU:          result_o = compare_result;
            default:                    result_o = 32'h0;
        endcase
    end

endmodule
