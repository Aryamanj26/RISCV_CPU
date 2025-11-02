`timescale 1ns/1ps

module tb_reg_file;
    logic clk, rst_n;
    logic [4:0] rs1_addr, rs2_addr, rd_addr;
    logic [31:0] rs1_data, rs2_data, rd_data;
    logic wr_en;
    
    int pass_count, fail_count;
    
    // Instantiate regist er file
    reg_file #(.XLEN(32), .NUM_REGS(32)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr_i(rs1_addr),
        .rs1_data_o(rs1_data),
        .rs2_addr_i(rs2_addr),
        .rs2_data_o(rs2_data),
        .wr_en_i(wr_en),
        .rd_addr_i(rd_addr),
        .rd_data_i(rd_data)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Check task
    task check(string name, logic [31:0] expected, logic [31:0] actual);
        if (expected == actual) begin
            $display("[PASS] %s: Expected=0x%h, Got=0x%h", name, expected, actual);
            pass_count++;
        end else begin
            $display("[FAIL] %s: Expected=0x%h, Got=0x%h", name, expected, actual);
            fail_count++;
        end
    endtask
    
    // Write to register task
    task write_reg(int reg_num, logic [31:0] value);
        @(posedge clk);
        wr_en = 1'b1;
        rd_addr = reg_num;
        rd_data = value;
        @(posedge clk);
        wr_en = 1'b0;
    endtask
    
    // Read register task
    task read_reg(int reg_num, output logic [31:0] value);
        rs1_addr = reg_num;
        @(posedge clk);
        #1; // Small delay for combinational read
        value = rs1_data;
    endtask
    
    // Test sequence
    initial begin
        pass_count = 0;
        fail_count = 0;
        
        $display("=== Register File Test Start ===\n");
        
        // Initialize
        rst_n = 0;
        wr_en = 0;
        rs1_addr = 0;
        rs2_addr = 0;
        rd_addr = 0;
        rd_data = 0;
        
        #20;
        rst_n = 1;
        $display("Reset released\n");
        
        // Test 1: x0 is always zero
        $display("--- Test 1: x0 hardwired to zero ---");
        write_reg(0, 32'hDEADBEEF);
        read_reg(0, rs1_data);
        check("x0 should be 0 after write", 32'h0, rs1_data);
        
        // Test 2: Write and read single register
        $display("\n--- Test 2: Single register write/read ---");
        write_reg(1, 32'h12345678);
        read_reg(1, rs1_data);
        check("x1 write/read", 32'h12345678, rs1_data);
        
        // Test 3: Multiple register writes
        $display("\n--- Test 3: Multiple register writes ---");
        write_reg(2, 32'hAAAAAAAA);
        write_reg(3, 32'h55555555);
        write_reg(4, 32'hFFFFFFFF);
        
        read_reg(2, rs1_data);
        check("x2 write/read", 32'hAAAAAAAA, rs1_data);
        
        read_reg(3, rs1_data);
        check("x3 write/read", 32'h55555555, rs1_data);
        
        read_reg(4, rs1_data);
        check("x4 write/read", 32'hFFFFFFFF, rs1_data);
        
        // Test 4: Simultaneous dual read
        $display("\n--- Test 4: Dual port read ---");
        rs1_addr = 2;
        rs2_addr = 3;
        @(posedge clk);
        #1;
        check("Dual read rs1 (x2)", 32'hAAAAAAAA, rs1_data);
        check("Dual read rs2 (x3)", 32'h55555555, rs2_data);
        
        // Test 5: Overwrite register
        $display("\n--- Test 5: Register overwrite ---");
        write_reg(1, 32'h11111111);
        read_reg(1, rs1_data);
        check("x1 overwrite", 32'h11111111, rs1_data);
        
        // Test 6: All registers (except x0)
        $display("\n--- Test 6: Write/Read all registers ---");
        for (int i = 1; i < 32; i++) begin
            write_reg(i, 32'h100 + i);
        end
        
        for (int i = 1; i < 32; i++) begin
            read_reg(i, rs1_data);
            check($sformatf("x%0d", i), 32'h100 + i, rs1_data);
        end
        
        // Test 7: No write when wr_en = 0
        $display("\n--- Test 7: Write enable = 0 ---");
        read_reg(5, rs1_data);
        logic [31:0] original = rs1_data;
        @(posedge clk);
        wr_en = 1'b0;
        rd_addr = 5;
        rd_data = 32'hBADBAD00;
        @(posedge clk);
        read_reg(5, rs1_data);
        check("x5 unchanged when wr_en=0", original, rs1_data);
        
        // Summary
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
    
    // Waveform dump
    initial begin
        $dumpfile("reg_file.vcd");
        $dumpvars(0, tb_reg_file);
    end
    
endmodule
