package pipe_pkg;

  // Writeback select
  typedef enum logic [1:0] { WB_ALU = 2'b00, WB_MEM = 2'b01, WB_PC = 2'b10 } wb_sel_t;

  // Operand selectors
  typedef enum logic [1:0] { OP_A_RS1 = 2'b00, OP_A_PC = 2'b01, OP_A_ZERO = 2'b10 } op_a_sel_t;
  typedef enum logic [1:0] { OP_B_RS2 = 2'b00, OP_B_IMM = 2'b01 } op_b_sel_t;

  // ALU operations
  typedef enum logic [3:0] {
    ALU_ADD = 4'd0,
    ALU_SUB = 4'd1,
    ALU_AND = 4'd2,
    ALU_OR  = 4'd3,
    ALU_XOR = 4'd4,
    ALU_SLT = 4'd5,
    ALU_SLTU= 4'd6,
    ALU_SLL = 4'd7,
    ALU_SRL = 4'd8,
    ALU_SRA = 4'd9
  } alu_op_t;

  // Branch kinds
  typedef enum logic [2:0] {
    BR_NONE = 3'd0,
    BR_BEQ  = 3'd1,
    BR_BNE  = 3'd2,
    BR_BLT  = 3'd3,
    BR_BGE  = 3'd4,
    BR_BLTU = 3'd5,
    BR_BGEU = 3'd6
  } branch_t;

  // Jump kinds
  typedef enum logic [1:0] { JMP_NONE = 2'd0, JMP_JAL = 2'd1, JMP_JALR = 2'd2 } jump_t;

  // Control struct
  typedef struct packed {
    wb_sel_t    wb_sel;
    op_a_sel_t  op_a_sel;
    op_b_sel_t  op_b_sel;
    alu_op_t    alu_op;
    branch_t    branch_kind;
    jump_t      jump_kind;
    logic       reg_write;
    logic       mem_read;
    logic       mem_write;
  } ctrl_t;

endpackage : pipe_pkg
