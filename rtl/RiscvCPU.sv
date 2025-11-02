`include "PipeTypes.svh"

// Top-level RISC-V CPU module
module riscv_cpu #(
    parameter int XLEN = 32,
    parameter int MEM_SIZE_BYTES = 4096
) (
    input  logic clk,
    input  logic rst_n
);

    //===========================================================================
    // Pipeline Stage Registers
    //===========================================================================
    IF_ID_t  if_id;
    ID_EX_t  id_ex;
    EX_MEM_t ex_mem;
    MEM_WB_t mem_wb;

    //===========================================================================
    // Hazard Control Signals
    //===========================================================================
    logic stall_if, stall_id, stall_ex;
    logic flush_if, flush_id, flush_ex;
    logic [1:0] forward_a, forward_b;
    
    //===========================================================================
    // Branch/Jump Signals
    //===========================================================================
    logic branch_taken;
    logic [31:0] branch_target;

    //===========================================================================
    // Register File Signals
    //===========================================================================
    logic        rf_wr_en;
    logic [4:0]  rf_rd_addr;
    logic [31:0] rf_rd_data;
    logic [31:0] rf_rs1_data, rf_rs2_data;

    //===========================================================================
    // Memory Signals
    //===========================================================================
    logic [31:0] mem_rdata;
    logic        mem_ready;

    //===========================================================================
    // Instruction Memory Signals
    //===========================================================================
    logic [31:0] imem_addr;
    logic [31:0] imem_data;

    //===========================================================================
    // Instruction Memory
    //===========================================================================
    instr_mem #(
        .ADDR_WIDTH     (32),
        .DATA_WIDTH     (32),
        .MEM_SIZE_BYTES (MEM_SIZE_BYTES)
    ) u_instr_mem (
        .clk      (clk),
        .addr_i   (imem_addr),
        .data_o   (imem_data)
    );

    //===========================================================================
    // Instruction Fetch Stage
    //===========================================================================
    IFStage u_if_stage (
        .clk           (clk),
        .rst_n         (rst_n),
        .branch_taken  (branch_taken),
        .branch_target (branch_target),
        .flush_if      (flush_if),
        .stall_if      (stall_if),
        .imem_addr_o   (imem_addr),
        .imem_data_i   (imem_data),
        .if_id_o       (if_id)
    );

    //===========================================================================
    // Instruction Decode Stage
    //===========================================================================
    id_stage u_id_stage (
        .clk            (clk),
        .rst_n          (rst_n),
        .stall_id       (stall_id),
        .flush_id       (flush_id),
        .if_id_i        (if_id),
        .rf_rs1_data_i  (rf_rs1_data),
        .rf_rs2_data_i  (rf_rs2_data),
        .id_ex_o        (id_ex)
    );

    //===========================================================================
    // Register File
    //===========================================================================
    reg_file #(
        .XLEN      (XLEN),
        .NUM_REGS  (32)
    ) u_reg_file (
        .clk         (clk),
        .rst_n       (rst_n),
        .rs1_addr_i  (if_id.instr[19:15]),
        .rs1_data_o  (rf_rs1_data),
        .rs2_addr_i  (if_id.instr[24:20]),
        .rs2_data_o  (rf_rs2_data),
        .wr_en_i     (rf_wr_en),
        .rd_addr_i   (rf_rd_addr),
        .rd_data_i   (rf_rd_data)
    );

    //===========================================================================
    // Execute Stage
    //===========================================================================
    ex_stage u_ex_stage (
        .clk             (clk),
        .rst_n           (rst_n),
        .stall_ex        (stall_ex),
        .flush_ex        (flush_ex),
        .id_ex_i         (id_ex),
        .ex_mem_o        (ex_mem),
        .branch_taken_o  (branch_taken),
        .branch_target_o (branch_target)
    );

    //===========================================================================
    // Memory Stage (Data Memory)
    //===========================================================================
    memory #(
        .ADDR_WIDTH     (32),
        .DATA_WIDTH     (32),
        .MEM_SIZE_BYTES (MEM_SIZE_BYTES)
    ) u_data_memory (
        .clk           (clk),
        .rst_n         (rst_n),
        .mem_read_i    (ex_mem.ctrl.mem_read),
        .mem_write_i   (ex_mem.ctrl.mem_write),
        .mem_addr_i    (ex_mem.alu_res),
        .mem_wdata_i   (ex_mem.rs2_data),
        .mem_byte_en_i (ex_mem.wstrb),
        .mem_rdata_o   (mem_rdata),
        .mem_ready_o   (mem_ready)
    );

    // MEM stage register
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mem_wb <= '0;
        end else begin
            mem_wb.valid    <= ex_mem.valid;
            mem_wb.pc       <= ex_mem.pc;
            mem_wb.alu_res  <= ex_mem.alu_res;
            mem_wb.mem_data <= mem_rdata;
            mem_wb.rd       <= ex_mem.rd;
            mem_wb.ctrl     <= ex_mem.ctrl;
        end
    end

    //===========================================================================
    // Writeback Stage
    //===========================================================================
    wb_stage u_wb_stage (
        .clk         (clk),
        .rst_n       (rst_n),
        .mem_wb_i    (mem_wb),
        .reg_write_o (rf_wr_en),
        .rd_addr_o   (rf_rd_addr),
        .rd_data_o   (rf_rd_data)
    );

    //===========================================================================
    // Hazard Detection and Forwarding Unit
    //===========================================================================
    hazard_unit u_hazard_unit (
        .if_id_i        (if_id),
        .id_ex_i        (id_ex),
        .ex_mem_i       (ex_mem),
        .mem_wb_i       (mem_wb),
        .branch_taken_i (branch_taken),
        .stall_if_o     (stall_if),
        .stall_id_o     (stall_id),
        .stall_ex_o     (stall_ex),
        .flush_if_o     (flush_if),
        .flush_id_o     (flush_id),
        .flush_ex_o     (flush_ex),
        .forward_a_o    (forward_a),
        .forward_b_o    (forward_b)
    );

endmodule
