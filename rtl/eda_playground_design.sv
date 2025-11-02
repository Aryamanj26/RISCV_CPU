// Combined design file for EDA Playground
// Copy all content to design.sv

`ifndef PIPETYPES_SVH
`define PIPETYPES_SVH
typedef enum logic [1:0] { WB_ALU = 2'b00, WB_MEM = 2'b01, WB_PC = 2'b10 } wb_sel_t;
typedef enum logic [1:0] { OP_A_RS1 = 2'b00, OP_A_PC = 2'b01, OP_A_ZERO = 2'b10 } op_a_sel_t;
typedef enum logic [1:0] { OP_B_RS2 = 2'b00, OP_B_IMM = 2'b01 } op_b_sel_t;
typedef enum logic [3:0] {
  ALU_ADD = 4'd0, ALU_SUB = 4'd1, ALU_AND = 4'd2, ALU_OR = 4'd3,
  ALU_XOR = 4'd4, ALU_SLT = 4'd5, ALU_SLTU= 4'd6, ALU_SLL = 4'd7,
  ALU_SRL = 4'd8, ALU_SRA = 4'd9
} alu_op_t;
typedef enum logic [2:0] { BR_NONE = 3'd0, BR_BEQ = 3'd1, BR_BNE = 3'd2, BR_BLT = 3'd3, BR_BGE = 3'd4, BR_BLTU = 3'd5, BR_BGEU = 3'd6 } branch_t;
typedef enum logic [1:0] { JMP_NONE = 2'd0, JMP_JAL = 2'd1, JMP_JALR = 2'd2 } jump_t;

typedef struct packed {
  wb_sel_t wb_sel; op_a_sel_t op_a_sel; op_b_sel_t op_b_sel; alu_op_t alu_op;
  branch_t branch_kind; jump_t jump_kind;
  logic reg_write; logic mem_read; logic mem_write;
} ctrl_t;

typedef struct packed {
  logic [31:0] pc; logic [31:0] instr; bit valid; bit flush;
} IF_ID_t;

typedef struct packed {
  logic [31:0] pc; logic [31:0] rs1_data, rs2_data, imm;
  logic [4:0] rs1, rs2, rd; ctrl_t ctrl; bit valid; bit flush; bit illegal;
} ID_EX_t;

typedef struct packed {
  logic [31:0] alu_res, rs2_data; logic [4:0] rd; ctrl_t ctrl;
  bit branch_taken; logic [31:0] branch_target; bit valid; bit flush;
  logic [3:0] wstrb; logic [31:0] pc;
} EX_MEM_t;

typedef struct packed {
  logic [31:0] alu_res, mem_data; logic [4:0] rd; ctrl_t ctrl;
  bit valid; bit flush; logic [31:0] pc;
} MEM_WB_t;
`endif

//===========================================================================
// ALU Module
//===========================================================================
module alu (
    input  logic [31:0] operand_a_i, operand_b_i,
    input  alu_op_t alu_op_i,
    output logic [31:0] result_o
);
    logic [31:0] arith_result, logic_result, shift_result, compare_result;
    
    always_comb begin
        case (alu_op_i)
            ALU_ADD: arith_result = operand_a_i + operand_b_i;
            ALU_SUB: arith_result = operand_a_i - operand_b_i;
            default: arith_result = 32'h0;
        endcase
    end
    
    always_comb begin
        case (alu_op_i)
            ALU_AND: logic_result = operand_a_i & operand_b_i;
            ALU_OR:  logic_result = operand_a_i | operand_b_i;
            ALU_XOR: logic_result = operand_a_i ^ operand_b_i;
            default: logic_result = 32'h0;
        endcase
    end
    
    always_comb begin
        case (alu_op_i)
            ALU_SLL: shift_result = operand_a_i << operand_b_i[4:0];
            ALU_SRL: shift_result = operand_a_i >> operand_b_i[4:0];
            ALU_SRA: shift_result = $signed(operand_a_i) >>> operand_b_i[4:0];
            default: shift_result = 32'h0;
        endcase
    end
    
    always_comb begin
        case (alu_op_i)
            ALU_SLT:  compare_result = ($signed(operand_a_i) < $signed(operand_b_i)) ? 32'd1 : 32'd0;
            ALU_SLTU: compare_result = (operand_a_i < operand_b_i) ? 32'd1 : 32'd0;
            default:  compare_result = 32'h0;
        endcase
    end
    
    always_comb begin
        case (alu_op_i)
            ALU_ADD, ALU_SUB: result_o = arith_result;
            ALU_AND, ALU_OR, ALU_XOR: result_o = logic_result;
            ALU_SLL, ALU_SRL, ALU_SRA: result_o = shift_result;
            ALU_SLT, ALU_SLTU: result_o = compare_result;
            default: result_o = 32'h0;
        endcase
    end
endmodule

//===========================================================================
// Register File
//===========================================================================
module reg_file #(parameter int XLEN = 32, parameter int NUM_REGS = 32) (
    input  logic clk, rst_n,
    input  logic [4:0] rs1_addr_i, rs2_addr_i, rd_addr_i,
    output logic [XLEN-1:0] rs1_data_o, rs2_data_o,
    input  logic wr_en_i,
    input  logic [XLEN-1:0] rd_data_i
);
    logic [XLEN-1:0] regs [NUM_REGS];
    
    assign rs1_data_o = (rs1_addr_i == 5'b0) ? '0 : regs[rs1_addr_i];
    assign rs2_data_o = (rs2_addr_i == 5'b0) ? '0 : regs[rs2_addr_i];
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_REGS; i++) regs[i] <= '0;
        end else if (wr_en_i && (rd_addr_i != 5'b0)) begin
            regs[rd_addr_i] <= rd_data_i;
        end
    end
endmodule

//===========================================================================
// Memory
//===========================================================================
module memory #(parameter int ADDR_WIDTH = 32, parameter int DATA_WIDTH = 32, parameter int MEM_SIZE_BYTES = 4096) (
    input  logic clk, rst_n,
    input  logic mem_read_i, mem_write_i,
    input  logic [ADDR_WIDTH-1:0] mem_addr_i,
    input  logic [DATA_WIDTH-1:0] mem_wdata_i,
    input  logic [3:0] mem_byte_en_i,
    output logic [DATA_WIDTH-1:0] mem_rdata_o,
    output logic mem_ready_o
);
    localparam int NUM_WORDS = MEM_SIZE_BYTES / (DATA_WIDTH / 8);
    localparam int WORD_ADDR_WIDTH = $clog2(NUM_WORDS);
    
    logic [DATA_WIDTH-1:0] mem_array [NUM_WORDS];
    logic [WORD_ADDR_WIDTH-1:0] word_addr;
    logic [DATA_WIDTH-1:0] rdata_reg;
    
    assign word_addr = mem_addr_i[WORD_ADDR_WIDTH+1:2];
    assign mem_ready_o = 1'b1;
    
    always_ff @(posedge clk) begin
        if (!rst_n) rdata_reg <= '0;
        else if (mem_read_i && word_addr < NUM_WORDS) rdata_reg <= mem_array[word_addr];
        else if (mem_read_i) rdata_reg <= '0;
    end
    
    assign mem_rdata_o = rdata_reg;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_WORDS; i++) mem_array[i] <= '0;
        end else if (mem_write_i && word_addr < NUM_WORDS) begin
            if (mem_byte_en_i[0]) mem_array[word_addr][7:0] <= mem_wdata_i[7:0];
            if (mem_byte_en_i[1]) mem_array[word_addr][15:8] <= mem_wdata_i[15:8];
            if (mem_byte_en_i[2]) mem_array[word_addr][23:16] <= mem_wdata_i[23:16];
            if (mem_byte_en_i[3]) mem_array[word_addr][31:24] <= mem_wdata_i[31:24];
        end
    end
endmodule

//===========================================================================
// Decoder (Stub - needs full implementation)
//===========================================================================
module decoder (
    input  logic [31:0] instr_i,
    output logic [4:0] rs1_o, rs2_o, rd_o,
    output logic [31:0] imm_o,
    output ctrl_t ctrl_o,
    output logic illegal_o
);
    assign rs1_o = instr_i[19:15];
    assign rs2_o = instr_i[24:20];
    assign rd_o = instr_i[11:7];
    assign imm_o = 32'h4;
    assign ctrl_o = '{default: 0};
    assign illegal_o = 1'b0;
endmodule

//===========================================================================
// IF Stage
//===========================================================================
module IFStage(
    input logic clk, rst_n, branch_taken, flush_if, stall_if,
    input logic [31:0] branch_target,
    output IF_ID_t if_id_o
);
    IF_ID_t if_id_l;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            if_id_l.pc <= 32'h0;
            if_id_l.instr <= 32'h13; // NOP
            if_id_l.valid <= 1'b0;
        end else if (flush_if) begin
            if_id_l.valid <= 1'b0;
        end else if (!stall_if) begin
            if_id_l.pc <= branch_taken ? branch_target : if_id_l.pc + 4;
            if_id_l.instr <= 32'h13; // NOP
            if_id_l.valid <= 1'b1;
        end
    end
    
    assign if_id_o = if_id_l;
endmodule

//===========================================================================
// ID Stage
//===========================================================================
module id_stage (
    input  logic clk, rst_n, stall_id, flush_id,
    input  IF_ID_t if_id_i,
    output ID_EX_t id_ex_o
);
    ID_EX_t id_ex_l;
    logic [4:0] rs1, rs2, rd;
    logic [31:0] imm;
    ctrl_t ctrl;
    logic illegal;
    
    decoder u_decoder (
        .instr_i(if_id_i.instr), .rs1_o(rs1), .rs2_o(rs2), .rd_o(rd),
        .imm_o(imm), .ctrl_o(ctrl), .illegal_o(illegal)
    );
    
    always_comb begin
        id_ex_l = '0;
        id_ex_l.valid = if_id_i.valid & ~illegal;
        id_ex_l.pc = if_id_i.pc;
        id_ex_l.rs1 = rs1; id_ex_l.rs2 = rs2; id_ex_l.rd = rd;
        id_ex_l.imm = imm; id_ex_l.ctrl = ctrl;
    end
    
    always_ff @(posedge clk) begin
        if (!rst_n) id_ex_o <= '0;
        else if (stall_id) id_ex_o <= id_ex_o;
        else if (flush_id) id_ex_o.valid <= 1'b0;
        else id_ex_o <= id_ex_l;
    end
endmodule

//===========================================================================
// EX Stage
//===========================================================================
module ex_stage (
    input  logic clk, rst_n, stall_ex, flush_ex,
    input  ID_EX_t id_ex_i,
    output EX_MEM_t ex_mem_o,
    output logic branch_taken_o,
    output logic [31:0] branch_target_o
);
    EX_MEM_t ex_mem_l;
    logic [31:0] alu_a, alu_b, alu_result;
    logic branch_taken;
    logic [31:0] branch_target;
    
    alu u_alu (.operand_a_i(alu_a), .operand_b_i(alu_b), .alu_op_i(id_ex_i.ctrl.alu_op), .result_o(alu_result));
    
    always_comb begin
        case (id_ex_i.ctrl.op_a_sel)
            OP_A_RS1: alu_a = id_ex_i.rs1_data;
            OP_A_PC: alu_a = id_ex_i.pc;
            default: alu_a = 32'h0;
        endcase
    end
    
    always_comb begin
        case (id_ex_i.ctrl.op_b_sel)
            OP_B_RS2: alu_b = id_ex_i.rs2_data;
            OP_B_IMM: alu_b = id_ex_i.imm;
            default: alu_b = 32'h0;
        endcase
    end
    
    always_comb begin
        branch_taken = 1'b0;
        case (id_ex_i.ctrl.branch_kind)
            BR_BEQ: branch_taken = (id_ex_i.rs1_data == id_ex_i.rs2_data);
            BR_BNE: branch_taken = (id_ex_i.rs1_data != id_ex_i.rs2_data);
            BR_BLT: branch_taken = ($signed(id_ex_i.rs1_data) < $signed(id_ex_i.rs2_data));
            BR_BGE: branch_taken = ($signed(id_ex_i.rs1_data) >= $signed(id_ex_i.rs2_data));
            BR_BLTU: branch_taken = (id_ex_i.rs1_data < id_ex_i.rs2_data);
            BR_BGEU: branch_taken = (id_ex_i.rs1_data >= id_ex_i.rs2_data);
        endcase
    end
    
    always_comb begin
        if (id_ex_i.ctrl.jump_kind == JMP_JALR) branch_target = (id_ex_i.rs1_data + id_ex_i.imm) & ~32'h1;
        else branch_target = id_ex_i.pc + id_ex_i.imm;
    end
    
    assign branch_taken_o = (branch_taken | (id_ex_i.ctrl.jump_kind != JMP_NONE)) & id_ex_i.valid;
    assign branch_target_o = branch_target;
    
    always_comb begin
        ex_mem_l = '0;
        ex_mem_l.valid = id_ex_i.valid;
        ex_mem_l.pc = id_ex_i.pc;
        ex_mem_l.alu_res = alu_result;
        ex_mem_l.rs2_data = id_ex_i.rs2_data;
        ex_mem_l.rd = id_ex_i.rd;
        ex_mem_l.ctrl = id_ex_i.ctrl;
        ex_mem_l.branch_taken = branch_taken;
        ex_mem_l.branch_target = branch_target;
        ex_mem_l.wstrb = 4'b1111;
    end
    
    always_ff @(posedge clk) begin
        if (!rst_n) ex_mem_o <= '0;
        else if (stall_ex) ex_mem_o <= ex_mem_o;
        else if (flush_ex) ex_mem_o.valid <= 1'b0;
        else ex_mem_o <= ex_mem_l;
    end
endmodule

//===========================================================================
// WB Stage
//===========================================================================
module wb_stage (
    input  logic clk, rst_n,
    input  MEM_WB_t mem_wb_i,
    output logic reg_write_o,
    output logic [4:0] rd_addr_o,
    output logic [31:0] rd_data_o
);
    logic [31:0] wb_data;
    
    always_comb begin
        case (mem_wb_i.ctrl.wb_sel)
            WB_ALU: wb_data = mem_wb_i.alu_res;
            WB_MEM: wb_data = mem_wb_i.mem_data;
            WB_PC: wb_data = mem_wb_i.pc + 32'd4;
            default: wb_data = 32'h0;
        endcase
    end
    
    assign reg_write_o = mem_wb_i.ctrl.reg_write & mem_wb_i.valid;
    assign rd_addr_o = mem_wb_i.rd;
    assign rd_data_o = wb_data;
endmodule

//===========================================================================
// Hazard Unit
//===========================================================================
module hazard_unit (
    input IF_ID_t if_id_i,
    input ID_EX_t id_ex_i,
    input EX_MEM_t ex_mem_i,
    input MEM_WB_t mem_wb_i,
    input logic branch_taken_i,
    output logic stall_if_o, stall_id_o, stall_ex_o,
    output logic flush_if_o, flush_id_o, flush_ex_o,
    output logic [1:0] forward_a_o, forward_b_o
);
    logic load_use_hazard;
    logic [4:0] rs1_id, rs2_id;
    
    assign rs1_id = if_id_i.instr[19:15];
    assign rs2_id = if_id_i.instr[24:20];
    
    always_comb begin
        load_use_hazard = 1'b0;
        if (id_ex_i.ctrl.mem_read && id_ex_i.valid) begin
            if ((id_ex_i.rd == rs1_id && rs1_id != 5'b0) || (id_ex_i.rd == rs2_id && rs2_id != 5'b0)) begin
                load_use_hazard = 1'b1;
            end
        end
    end
    
    always_comb begin
        stall_if_o = load_use_hazard;
        stall_id_o = load_use_hazard;
        stall_ex_o = 1'b0;
    end
    
    always_comb begin
        flush_if_o = branch_taken_i;
        flush_id_o = branch_taken_i;
        flush_ex_o = branch_taken_i;
    end
    
    always_comb begin
        forward_a_o = 2'b00;
        if (mem_wb_i.ctrl.reg_write && mem_wb_i.valid && (mem_wb_i.rd != 5'b0) && (mem_wb_i.rd == id_ex_i.rs1))
            forward_a_o = 2'b10;
        if (ex_mem_i.ctrl.reg_write && ex_mem_i.valid && (ex_mem_i.rd != 5'b0) && (ex_mem_i.rd == id_ex_i.rs1))
            forward_a_o = 2'b01;
    end
    
    always_comb begin
        forward_b_o = 2'b00;
        if (mem_wb_i.ctrl.reg_write && mem_wb_i.valid && (mem_wb_i.rd != 5'b0) && (mem_wb_i.rd == id_ex_i.rs2))
            forward_b_o = 2'b10;
        if (ex_mem_i.ctrl.reg_write && ex_mem_i.valid && (ex_mem_i.rd != 5'b0) && (ex_mem_i.rd == id_ex_i.rs2))
            forward_b_o = 2'b01;
    end
endmodule

//===========================================================================
// Top Level CPU
//===========================================================================
module riscv_cpu #(parameter int XLEN = 32, parameter int MEM_SIZE_BYTES = 4096) (
    input logic clk, rst_n
);
    IF_ID_t if_id;
    ID_EX_t id_ex;
    EX_MEM_t ex_mem;
    MEM_WB_t mem_wb;
    
    logic stall_if, stall_id, stall_ex;
    logic flush_if, flush_id, flush_ex;
    logic [1:0] forward_a, forward_b;
    logic branch_taken;
    logic [31:0] branch_target;
    logic rf_wr_en;
    logic [4:0] rf_rd_addr;
    logic [31:0] rf_rd_data, rf_rs1_data, rf_rs2_data;
    logic [31:0] mem_rdata;
    logic mem_ready;
    
    IFStage u_if_stage (.clk(clk), .rst_n(rst_n), .branch_taken(branch_taken), .branch_target(branch_target),
                        .flush_if(flush_if), .stall_if(stall_if), .if_id_o(if_id));
    
    id_stage u_id_stage (.clk(clk), .rst_n(rst_n), .stall_id(stall_id), .flush_id(flush_id),
                         .if_id_i(if_id), .id_ex_o(id_ex));
    
    reg_file #(.XLEN(XLEN), .NUM_REGS(32)) u_reg_file (
        .clk(clk), .rst_n(rst_n), .rs1_addr_i(if_id.instr[19:15]), .rs1_data_o(rf_rs1_data),
        .rs2_addr_i(if_id.instr[24:20]), .rs2_data_o(rf_rs2_data),
        .wr_en_i(rf_wr_en), .rd_addr_i(rf_rd_addr), .rd_data_i(rf_rd_data));
    
    ex_stage u_ex_stage (.clk(clk), .rst_n(rst_n), .stall_ex(stall_ex), .flush_ex(flush_ex),
                         .id_ex_i(id_ex), .ex_mem_o(ex_mem),
                         .branch_taken_o(branch_taken), .branch_target_o(branch_target));
    
    memory #(.ADDR_WIDTH(32), .DATA_WIDTH(32), .MEM_SIZE_BYTES(MEM_SIZE_BYTES)) u_data_memory (
        .clk(clk), .rst_n(rst_n), .mem_read_i(ex_mem.ctrl.mem_read), .mem_write_i(ex_mem.ctrl.mem_write),
        .mem_addr_i(ex_mem.alu_res), .mem_wdata_i(ex_mem.rs2_data), .mem_byte_en_i(ex_mem.wstrb),
        .mem_rdata_o(mem_rdata), .mem_ready_o(mem_ready));
    
    always_ff @(posedge clk) begin
        if (!rst_n) mem_wb <= '0;
        else begin
            mem_wb.valid <= ex_mem.valid;
            mem_wb.pc <= ex_mem.pc;
            mem_wb.alu_res <= ex_mem.alu_res;
            mem_wb.mem_data <= mem_rdata;
            mem_wb.rd <= ex_mem.rd;
            mem_wb.ctrl <= ex_mem.ctrl;
        end
    end
    
    wb_stage u_wb_stage (.clk(clk), .rst_n(rst_n), .mem_wb_i(mem_wb),
                         .reg_write_o(rf_wr_en), .rd_addr_o(rf_rd_addr), .rd_data_o(rf_rd_data));
    
    hazard_unit u_hazard_unit (
        .if_id_i(if_id), .id_ex_i(id_ex), .ex_mem_i(ex_mem), .mem_wb_i(mem_wb),
        .branch_taken_i(branch_taken), .stall_if_o(stall_if), .stall_id_o(stall_id), .stall_ex_o(stall_ex),
        .flush_if_o(flush_if), .flush_id_o(flush_id), .flush_ex_o(flush_ex),
        .forward_a_o(forward_a), .forward_b_o(forward_b));
endmodule
