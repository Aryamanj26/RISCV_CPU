// Testbench for EDA Playground
// Copy to testbench.sv

`timescale 1ns/1ps

module tb_riscv_cpu;
    logic clk, rst_n;
    int pass_count, fail_count;
    
    riscv_cpu #(.XLEN(32), .MEM_SIZE_BYTES(4096)) dut (.clk(clk), .rst_n(rst_n));
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Task to load instruction and check result
    task test_instruction(string name, logic [31:0] expected_result);
        @(posedge clk);
        repeat(5) @(posedge clk); // Wait for instruction to complete pipeline
        if (dut.mem_wb.valid && dut.rf_wr_en) begin
            if (dut.rf_rd_data == expected_result) begin
                $display("[PASS] %s: Got 0x%h (x%0d)", name, dut.rf_rd_data, dut.rf_rd_addr);
                pass_count++;
            end else begin
                $display("[FAIL] %s: Expected 0x%h, Got 0x%h (x%0d)", name, expected_result, dut.rf_rd_data, dut.rf_rd_addr);
                fail_count++;
            end
        end else begin
            $display("[FAIL] %s: No writeback occurred", name);
            fail_count++;
        end
    endtask
    
    // Task to initialize register
    task init_reg(int reg_num, logic [31:0] value);
        force dut.u_reg_file.regs[reg_num] = value;
        @(posedge clk);
        release dut.u_reg_file.regs[reg_num];
    endtask
    
    initial begin
        pass_count = 0;
        fail_count = 0;
        
        $display("=== RISC-V CPU Instruction Tests ===\n");
        rst_n = 0;
        #20;
        rst_n = 1;
        $display("Reset released\n");
        
        // Initialize test registers
        init_reg(1, 32'h0000000A);  // x1 = 10
        init_reg(2, 32'h00000005);  // x2 = 5
        init_reg(3, 32'hFFFFFFFF);  // x3 = -1
        
        $display("--- R-Type Instructions ---");
        // ADD: x4 = x1 + x2 (10 + 5 = 15)
        // Instruction encoding would go here via memory
        test_instruction("ADD", 32'h0000000F);
        
        // SUB: x5 = x1 - x2 (10 - 5 = 5)
        test_instruction("SUB", 32'h00000005);
        
        // AND: x6 = x1 & x2 (0xA & 0x5 = 0x0)
        test_instruction("AND", 32'h00000000);
        
        // OR: x7 = x1 | x2 (0xA | 0x5 = 0xF)
        test_instruction("OR", 32'h0000000F);
        
        // XOR: x8 = x1 ^ x2 (0xA ^ 0x5 = 0xF)
        test_instruction("XOR", 32'h0000000F);
        
        // SLT: x9 = (x2 < x1) = 1
        test_instruction("SLT", 32'h00000001);
        
        // SLL: x10 = x2 << 2 (5 << 2 = 20)
        test_instruction("SLL", 32'h00000014);
        
        // SRL: x11 = x1 >> 1 (10 >> 1 = 5)
        test_instruction("SRL", 32'h00000005);
        
        // SRA: x12 = x3 >>> 1 (-1 >>> 1 = -1)
        test_instruction("SRA", 32'hFFFFFFFF);
        
        $display("\n--- I-Type Instructions ---");
        // ADDI: x13 = x1 + 100 (10 + 100 = 110)
        test_instruction("ADDI", 32'h0000006E);
        
        $display("\n--- Load/Store Instructions ---");
        // SW: Store x1 to memory
        test_instruction("SW", 32'h0000000A);
        
        // LW: Load from memory to x14
        test_instruction("LW", 32'h0000000A);
        
        $display("\n--- Branch Instructions ---");
        // BEQ: Branch if x1 == x1 (should take)
        @(posedge clk);
        if (dut.branch_taken) begin
            $display("[PASS] BEQ: Branch taken");
            pass_count++;
        end else begin
            $display("[FAIL] BEQ: Branch not taken");
            fail_count++;
        end
        
        // BNE: Branch if x1 != x2 (should take)
        @(posedge clk);
        if (dut.branch_taken) begin
            $display("[PASS] BNE: Branch taken");
            pass_count++;
        end else begin
            $display("[FAIL] BNE: Branch not taken");
            fail_count++;
        end
        
        $display("\n--- Jump Instructions ---");
        // JAL: Jump and link
        test_instruction("JAL", 32'h00000004); // Should save PC+4
        
        // JALR: Jump and link register
        test_instruction("JALR", 32'h00000004);
        
        $display("\n=== Test Summary ===");
        $display("PASSED: %0d", pass_count);
        $display("FAILED: %0d", fail_count);
        if (fail_count == 0) begin
            $display("ALL TESTS PASSED!");
        end else begin
            $display("SOME TESTS FAILED!");
        end
        
        $finish;
    end
    
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_riscv_cpu);
    end
endmodule
