// Instruction Memory (ROM)
module instr_mem #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int MEM_SIZE_BYTES = 4096
) (
    input  logic                    clk,
    input  logic [ADDR_WIDTH-1:0]   addr_i,
    output logic [DATA_WIDTH-1:0]   data_o
);
    localparam int NUM_WORDS = MEM_SIZE_BYTES / (DATA_WIDTH / 8);
    localparam int WORD_ADDR_WIDTH = $clog2(NUM_WORDS);
    
    logic [DATA_WIDTH-1:0] mem [NUM_WORDS];
    logic [WORD_ADDR_WIDTH-1:0] word_addr;
    
    assign word_addr = addr_i[WORD_ADDR_WIDTH+1:2];
    
    always_comb begin
        if (word_addr < NUM_WORDS) begin
            data_o = mem[word_addr];
        end else begin
            data_o = 32'h00000013; // NOP for out of bounds
        end
    end
    
    // Initialize with NOPs or load from file
    initial begin
        for (int i = 0; i < NUM_WORDS; i++) begin
            mem[i] = 32'h00000013; 
        end
        
        // Uncomment to load from hex file:
        // $readmemh("program.hex", mem);
    end

endmodule
