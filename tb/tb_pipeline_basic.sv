`timescale 1ns/1ps
`include "PipeTypes.svh"

module tb_pipeline_basic;

    logic clk;
    logic rst_n;

    // Instantiate CPU
    riscv_cpu #(
        .XLEN(32),
        .MEM_SIZE_BYTES(4096)
    ) dut (
        .clk(clk),
        .rst_n(rst_n)
    );

    // Clock generation (10ns period = 100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Load test program into instruction memory
    initial begin
        $display("=== Loading Test Program into Instruction Memory ===");
        
        // Simple test program:
        // Address 0x00: ADDI x1, x0, 5      (x1 = 5)
        dut.u_instr_mem.mem[0] = 32'h00500093;
        
        // Address 0x04: ADDI x2, x0, 10     (x2 = 10)
        dut.u_instr_mem.mem[1] = 32'h00A00113;
        
        // Address 0x08: ADD x3, x1, x2      (x3 = x1 + x2 = 15)
        dut.u_instr_mem.mem[2] = 32'h002081B3;
        
        // Address 0x0C: SUB x4, x2, x1      (x4 = x2 - x1 = 5)
        dut.u_instr_mem.mem[3] = 32'h40110233;
        
        // Address 0x10: AND x5, x3, x2      (x5 = x3 & x2 = 10)
        dut.u_instr_mem.mem[4] = 32'h0021F2B3;
        
        // Address 0x14: OR x6, x1, x2       (x6 = x1 | x2 = 15)
        dut.u_instr_mem.mem[5] = 32'h0020E333;
        
        // Address 0x18: XOR x7, x3, x4      (x7 = x3 ^ x4 = 10)
        dut.u_instr_mem.mem[6] = 32'h004183B3;
        
        // Address 0x1C: SLL x8, x1, x1      (x8 = x1 << x1 = 5 << 5 = 160)
        dut.u_instr_mem.mem[7] = 32'h00109433;
        
        // Address 0x20: NOP (ADDI x0, x0, 0)
        dut.u_instr_mem.mem[8] = 32'h00000013;
        
        // Address 0x24: NOP
        dut.u_instr_mem.mem[9] = 32'h00000013;
        
        $display("Test program loaded:");
        $display("  [0] ADDI x1, x0, 5      -> x1 = 5");
        $display("  [1] ADDI x2, x0, 10     -> x2 = 10");
        $display("  [2] ADD  x3, x1, x2     -> x3 = 15");
        $display("  [3] SUB  x4, x2, x1     -> x4 = 5");
        $display("  [4] AND  x5, x3, x2     -> x5 = 10");
        $display("  [5] OR   x6, x1, x2     -> x6 = 15");
        $display("  [6] XOR  x7, x3, x4     -> x7 = 10");
        $display("  [7] SLL  x8, x1, x1     -> x8 = 160");
        $display("  [8-9] NOPs");
    end

    // Test sequence
    initial begin
        $display("\n=== RISC-V CPU Pipeline Testbench Start ===");
        $display("Time: %0t", $time);
        
        // Reset
        rst_n = 0;
        #25;  // Hold reset for 2.5 cycles
        rst_n = 1;
        $display("\nTime %0t: Reset released", $time);
        
        // Run for enough cycles to see instructions flow through pipeline
        // Need at least 12-15 cycles to see all instructions complete
        repeat(30) @(posedge clk);
        
        // Display final register file state
        $display("\n=== Final Register File State ===");
        $display("Time %0t:", $time);
        $display("  x1 = %0d (expected: 5)", dut.u_reg_file.regs[1]);
        $display("  x2 = %0d (expected: 10)", dut.u_reg_file.regs[2]);
        $display("  x3 = %0d (expected: 15)", dut.u_reg_file.regs[3]);
        $display("  x4 = %0d (expected: 5)", dut.u_reg_file.regs[4]);
        $display("  x5 = %0d (expected: 10)", dut.u_reg_file.regs[5]);
        $display("  x6 = %0d (expected: 15)", dut.u_reg_file.regs[6]);
        $display("  x7 = %0d (expected: 10)", dut.u_reg_file.regs[7]);
        $display("  x8 = %0d (expected: 160)", dut.u_reg_file.regs[8]);
        
        $display("\n=== Testbench Complete ===");
        $finish;
    end

    // Monitor pipeline progression
    always @(posedge clk) begin
        if (rst_n && dut.if_id.valid) begin
            $display("Time %0t: IF - PC=0x%h, Instr=0x%h", 
                     $time, dut.if_id.pc, dut.if_id.instr);
        end
        
        if (rst_n && dut.id_ex.valid) begin
            $display("Time %0t: ID - PC=0x%h, rd=x%0d, rs1=x%0d (val=%0d), rs2=x%0d (val=%0d)", 
                     $time, dut.id_ex.pc, dut.id_ex.rd, 
                     dut.id_ex.rs1, dut.id_ex.rs1_data,
                     dut.id_ex.rs2, dut.id_ex.rs2_data);
        end
        
        if (rst_n && dut.ex_mem.valid) begin
            $display("Time %0t: EX - PC=0x%h, rd=x%0d, ALU Result=%0d", 
                     $time, dut.ex_mem.pc, dut.ex_mem.rd, dut.ex_mem.alu_res);
        end
        
        if (rst_n && dut.mem_wb.valid && dut.mem_wb.ctrl.reg_write) begin
            $display("Time %0t: WB - Writing x%0d = %0d", 
                     $time, dut.mem_wb.rd, dut.rf_rd_data);
        end
    end

    // Monitor hazards and stalls
    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.stall_if || dut.stall_id || dut.stall_ex)
                $display("Time %0t: STALL - IF=%b, ID=%b, EX=%b", 
                         $time, dut.stall_if, dut.stall_id, dut.stall_ex);
            
            if (dut.flush_if || dut.flush_id || dut.flush_ex)
                $display("Time %0t: FLUSH - IF=%b, ID=%b, EX=%b", 
                         $time, dut.flush_if, dut.flush_id, dut.flush_ex);
            
            if (dut.forward_a != 2'b00 || dut.forward_b != 2'b00)
                $display("Time %0t: FORWARD - A=%b, B=%b", 
                         $time, dut.forward_a, dut.forward_b);
        end
    end

    // Waveform dump for viewing
    initial begin
        $dumpfile("pipeline_basic.vcd");
        $dumpvars(0, tb_pipeline_basic);
        // Dump register file contents
        $dumpvars(0, dut.u_reg_file.regs[1]);
        $dumpvars(0, dut.u_reg_file.regs[2]);
        $dumpvars(0, dut.u_reg_file.regs[3]);
        $dumpvars(0, dut.u_reg_file.regs[4]);
        $dumpvars(0, dut.u_reg_file.regs[5]);
        $dumpvars(0, dut.u_reg_file.regs[6]);
        $dumpvars(0, dut.u_reg_file.regs[7]);
        $dumpvars(0, dut.u_reg_file.regs[8]);
    end

endmodule
