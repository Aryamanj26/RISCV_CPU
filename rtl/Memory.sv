// Memory Module for RISC-V CPU
// Simple byte-addressable memory array
// Designed to be scalable into a cache/DRAM hierarchy
//
// Current implementation: Simple synchronous SRAM
// Future expansion paths:
//   - Add cache controller interface
//   - Separate into L1/L2 cache modules
//   - Add memory controller for DRAM interface
//   - Support for different memory regions (instruction/data)

module memory #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int MEM_SIZE_BYTES = 4096 
) (
    input  logic                    clk,
    input  logic                    rst_n,
    
    input  logic                    mem_read_i,    
    input  logic                    mem_write_i, 
    input  logic [ADDR_WIDTH-1:0]   mem_addr_i,
    input  logic [DATA_WIDTH-1:0]   mem_wdata_i,
    input  logic [3:0]              mem_byte_en_i,
    output logic [DATA_WIDTH-1:0]   mem_rdata_o,
    output logic                    mem_ready_o
);

    localparam int NUM_WORDS = MEM_SIZE_BYTES / (DATA_WIDTH / 8);
    localparam int WORD_ADDR_WIDTH = $clog2(NUM_WORDS);//10 bits for 4096 bytes

    // stored as words
    logic [DATA_WIDTH-1:0] mem_array [NUM_WORDS];

    // word aligned address
    logic [WORD_ADDR_WIDTH-1:0] word_addr;
    assign word_addr = mem_addr_i[WORD_ADDR_WIDTH+1:2]; // Divide by 4 for word address
    
    logic [DATA_WIDTH-1:0] rdata_reg;

    assign mem_ready_o = 1'b1;// Memory ready signal (always ready for simple array, can be modified for cache)
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rdata_reg <= '0;
        end else if (mem_read_i) begin
            // Check if address is in bounds
            if (word_addr < NUM_WORDS) begin
                rdata_reg <= mem_array[word_addr];
            end else begin
                rdata_reg <= '0; // Return zero for out-of-bounds reads
                `ifdef SIMULATION
                $display("Warning: Memory read out of bounds at address 0x%h", mem_addr_i);
                `endif
            end
        end
    end
    
    assign mem_rdata_o = rdata_reg;
    
    // Write operation with byte enable support
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Initialize memory to zero
            for (int i = 0; i < NUM_WORDS; i++) begin
                mem_array[i] <= '0;
            end
        end else if (mem_write_i) begin
            // Check if address is in bounds
            if (word_addr < NUM_WORDS) begin
                // Byte-enable write
                if (mem_byte_en_i[0]) mem_array[word_addr][7:0]   <= mem_wdata_i[7:0];
                if (mem_byte_en_i[1]) mem_array[word_addr][15:8]  <= mem_wdata_i[15:8];
                if (mem_byte_en_i[2]) mem_array[word_addr][23:16] <= mem_wdata_i[23:16];
                if (mem_byte_en_i[3]) mem_array[word_addr][31:24] <= mem_wdata_i[31:24];
            end else begin
                `ifdef SIMULATION
                $display("Warning: Memory write out of bounds at address 0x%h", mem_addr_i);
                `endif
            end
        end
    end
    
    // Optional: Memory initialization from file
    `ifdef SIMULATION
    initial begin
        $display("Memory initialized: Size=%0d bytes, %0d words", MEM_SIZE_BYTES, NUM_WORDS);
        // Uncomment to load memory from file:
        // $readmemh("memory_init.hex", mem_array);
    end
    `endif
    
    //===========================================================================
    // Future expansion hooks for cache/DRAM hierarchy:
    //===========================================================================
    // 1. Replace mem_array with cache controller instance
    // 2. Add cache miss/hit logic
    // 3. Connect mem_ready_o to cache miss handler
    // 4. Add burst transfer support for DRAM
    // 5. Add separate instruction and data memory ports
    // 6. Implement cache coherency protocol if multi-core
    
endmodule
