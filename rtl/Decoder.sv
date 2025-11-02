`include "PipeTypes.svh"

module decoder
(
  input  logic [31:0] instr_i,
  output logic [4:0]  rs1_o,
  output logic [4:0]  rs2_o,
  output logic [4:0]  rd_o,
  output logic [31:0] imm_o,
  output ctrl_t       ctrl_o,
  output logic        illegal_o
);

    // Opcode definitions
  localparam logic [6:0]
    OP_LUI    = 7'b0110111,
    OP_AUIPC  = 7'b0010111,
    OP_JAL    = 7'b1101111,
    OP_JALR   = 7'b1100111,
    OP_BRANCH = 7'b1100011,
    OP_LOAD   = 7'b0000011,
    OP_STORE  = 7'b0100011,
    OP_OPIMM  = 7'b0010011,
    OP_OP     = 7'b0110011,
    OP_SYSTEM = 7'b1110011;

  logic [6:0] opcode = instr_i[6:0]; //always bits [6:0]
  logic [2:0] funct3 = instr_i[14:12];
  logic [6:0] funct7 = instr_i[31:25];

  // Instruction bitfield layout (RV32):

  //  - opcode  = instr[6:0]
  //  - rd      = instr[11:7] or imm (for S-type)
  //  - funct3  = instr[14:12]
  //  - rs1     = instr[19:15]
  //  - rs2     = instr[24:20] or imm (for I-type)
  //  - funct7  = instr[31:25] (full 7-bit field decoded for future expansion)
  //
  // Immediate encodings summary (where bits come from in the 32-bit word):
  //  - I-type: imm[11:0] = instr[31:20]
  //  - S-type: imm[11:5] = instr[31:25]; imm[4:0] = instr[11:7]
  //  - B-type: imm[12] = instr[31]; imm[11] = instr[7]; imm[10:5] = instr[30:25]; imm[4:1]=instr[11:8]; imm[0]=0
  //  - U-type: imm[31:12] = instr[31:12]; imm[11:0]=0
  //  - J-type: imm[20]=instr[31]; imm[19:12]=instr[19:12]; imm[11]=instr[20]; imm[10:1]=instr[30:21]; imm[0]=0

  assign rs1_o = instr_i[19:15];
  assign rs2_o = instr_i[24:20];
  assign rd_o  = instr_i[11:7];

  ctrl_t c;
  logic [31:0] imm;
  // immediate extraction functions using masks and shifts
  // I-type: imm[11:0] = instr[31:20]
  function automatic logic [31:0] imm_i(input logic [31:0] ins);
    logic [11:0] imm12;
    imm12 = (ins >> 20) & 12'hFFF; // mask low 12 bits after shift
    imm_i = {{20{imm12[11]}}, imm12}; //sign-extend
  endfunction

  // S-type: imm[11:5]=instr[31:25], imm[4:0]=instr[11:7]
  function automatic logic [31:0] imm_s(input logic [31:0] ins);
    logic [11:0] imm12;
    imm12 = (((ins >> 25) & 7'h7F) << 5) | ((ins >> 7) & 5'h1F);
    imm_s = {{20{imm12[11]}}, imm12}; //sign-extend
  endfunction

  // B-type: imm[12]=instr[31], imm[11]=instr[7], imm[10:5]=instr[30:25], imm[4:1]=instr[11:8], imm[0]=0
  function automatic logic [31:0] imm_b(input logic [31:0] ins);
    logic [12:0] imm13;
    imm13 = { (ins >> 31) & 1'b1,
              (ins >> 7)  & 1'b1,
              (ins >> 25) & 6'h3F,
              (ins >> 8)  & 4'hF,
              1'b0 };
    imm_b = {{19{imm13[12]}}, imm13}; // sign-extend 13-bit B-immediate
  endfunction

  // U-type: imm[31:12] = instr[31:12], low 12 bits zero
  function automatic logic [31:0] imm_u(input logic [31:0] ins);
    imm_u = ins & 32'hFFFFF000; // mask lower 12 bits to zero
  endfunction

  // J-type: imm[20]=instr[31], imm[19:12]=instr[19:12], imm[11]=instr[20], imm[10:1]=instr[30:21], imm[0]=0
  function automatic logic [31:0] imm_j(input logic [31:0] ins);
    logic [20:0] imm21;
    imm21 = { (ins >> 31) & 1'b1,
              (ins >> 12) & 8'hFF,
              (ins >> 20) & 1'b1,
              (ins >> 21) & 10'h3FF,
              1'b0 };
    imm_j = {{11{imm21[20]}}, imm21}; // sign-extend 21-bit J-immediate
  endfunction

  always_comb begin
    // default values: clear control signals first
    c.wb_sel      = WB_ALU;
    c.op_a_sel    = OP_A_RS1;
    c.op_b_sel    = OP_B_RS2;
    c.alu_op      = ALU_ADD;
    c.branch_kind = BR_NONE;
    c.jump_kind   = JMP_NONE;
    c.reg_write   = 1'b0;
    c.mem_read    = 1'b0;
    c.mem_write   = 1'b0;
    imm = 32'h0;
    illegal_o = 1'b0; // assume legal until proven otherwise

    case (opcode)
      OP_OP: begin
        // R-type arithmetic. We only support add/sub for now.
        // Full funct7+funct3 decoded to allow easy expansion later
        c.reg_write = 1'b1; // writes back to rd
        case ({funct7, funct3})
          10'b0000000_000: begin
            c.alu_op = ALU_ADD; // add
          end
          10'b0100000_000: begin
            c.alu_op = ALU_SUB; // sub
          end
          default: begin
            // unsupported R-type (e.g., AND, OR, XOR, shifts, MUL/DIV not implemented)
            illegal_o = 1'b1;
            c.reg_write = 1'b0;
          end
        endcase
      end

      OP_OPIMM: begin
        // I-type immediate ALU. We only accept ADDI here.
        c.op_b_sel  = OP_B_IMM;
        imm = imm_i(instr_i);
        if (funct3 == 3'b000) begin
          c.reg_write = 1'b1;
          c.alu_op = ALU_ADD; // addi
        end else begin
          illegal_o = 1'b1;
        end
      end

      OP_LOAD: begin
        // LW only
        if (funct3 == 3'b010) begin
          c.reg_write = 1'b1;
          c.mem_read  = 1'b1;
          c.wb_sel    = WB_MEM;
          c.op_b_sel  = OP_B_IMM;
          imm         = imm_i(instr_i);
          c.alu_op    = ALU_ADD; // address calc
        end else begin
          illegal_o = 1'b1;
        end
      end

      OP_STORE: begin
        // SW only
        if (funct3 == 3'b010) begin
          c.mem_write = 1'b1;
          c.op_b_sel  = OP_B_IMM;
          imm         = imm_s(instr_i);
          c.alu_op    = ALU_ADD; // address calc
        end else begin
          illegal_o = 1'b1;
        end
      end

      OP_BRANCH: begin
        // only BEQ and BNE supported in reduced set
        case (funct3)
          3'b000: begin c.branch_kind = BR_BEQ; end
          3'b001: begin c.branch_kind = BR_BNE; end
          default: begin illegal_o = 1'b1; end
        endcase
        imm = imm_b(instr_i);
      end

      OP_JAL: begin
        // JAL: write link and jump
        c.reg_write  = 1'b1;
        c.jump_kind  = JMP_JAL;
        c.wb_sel     = WB_PC;    // write PC+4 to rd
        c.op_a_sel   = OP_A_PC;
        c.op_b_sel   = OP_B_IMM;
        c.alu_op     = ALU_ADD;
        imm          = imm_j(instr_i);
      end

      // JALR is not supported here
      OP_JALR: begin
        illegal_o = 1'b1;
      end

      OP_LUI: begin
        c.reg_write = 1'b1;
        c.op_a_sel  = OP_A_ZERO;
        c.op_b_sel  = OP_B_IMM;
        c.alu_op    = ALU_ADD;
        imm         = imm_u(instr_i);
      end

      // AUIPC not supported
      OP_AUIPC: begin
        illegal_o = 1'b1;
      end

      // SYSTEM not supported
      OP_SYSTEM: begin
        illegal_o = 1'b1;
      end
      default: begin
        // unknown opcode
        illegal_o = 1'b1;
      end
    endcase
  end

  assign ctrl_o = c;
  assign imm_o  = imm;

endmodule
