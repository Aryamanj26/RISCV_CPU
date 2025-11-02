// Register File for RISC-V CPU
// 32 general-purpose registers (x0-x31)
// x0 is hardwired to zero
// 2 read ports, 1 write port

module reg_file #(
    parameter int XLEN = 32,
    parameter int NUM_REGS = 32
) (
    input  logic              clk,
    input  logic              rst_n,
    
    // Read port 1 (rs1)
    input  logic [4:0]        rs1_addr_i,
    output logic [XLEN-1:0]   rs1_data_o,
    
    // Read port 2 (rs2)
    input  logic [4:0]        rs2_addr_i,
    output logic [XLEN-1:0]   rs2_data_o,
    
    // Write port (rd)
    input  logic              wr_en_i,
    input  logic [4:0]        rd_addr_i,
    input  logic [XLEN-1:0]   rd_data_i
);

    logic [XLEN-1:0] regs [NUM_REGS];

    // Combinational read
    // x0 is always zero according to RISC-V spec
    assign rs1_data_o = (rs1_addr_i == 5'b0) ? '0 : regs[rs1_addr_i];
    assign rs2_data_o = (rs2_addr_i == 5'b0) ? '0 : regs[rs2_addr_i];

    // Sequential write
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_REGS; i++) begin
                regs[i] <= '0;
            end
        end else begin
            // Write to register if write enabled and not writing to x0
            if (wr_en_i && (rd_addr_i != 5'b0)) begin
                regs[rd_addr_i] <= rd_data_i;
            end
        end
    end

    `ifdef SIMULATION
    initial begin
        $display("Register File initialized with %0d registers of width %0d", NUM_REGS, XLEN);
    end
    `endif

endmodule
