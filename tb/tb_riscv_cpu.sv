`timescale 1ns/1ps
`include "PipeTypes.svh"

module tb_riscv_cpu;

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

    // Test sequence
    initial begin
        $display("=== RISC-V CPU Testbench Start ===");
        
        // Reset
        rst_n = 0;
        #20;
        rst_n = 1;
        $display("Time %0t: Reset released", $time);
        
        // Run for some cycles
        repeat(50) @(posedge clk);
        
        $display("=== Testbench Complete ===");
        $finish;
    end

    // Monitor key signals
    initial begin
        $monitor("Time %0t: PC=%h, IF_ID.valid=%b, ID_EX.valid=%b, EX_MEM.valid=%b, MEM_WB.valid=%b",
                 $time, dut.if_id.pc, dut.if_id.valid, dut.id_ex.valid, 
                 dut.ex_mem.valid, dut.mem_wb.valid);
    end

    // Waveform dump
    initial begin
        $dumpfile("riscv_cpu.vcd");
        $dumpvars(0, tb_riscv_cpu);
    end

endmodule
