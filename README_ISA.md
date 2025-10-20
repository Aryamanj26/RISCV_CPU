# RISCV_CPU — ISA reference

This document describes the reduced RV32I subset implemented by this project. It's written to be concise and easy to read so you can quickly see which instructions are supported and how immediates and control signals are produced.

Key points
- Reset: synchronous, active-low.
- Target ISA: a reduced RV32I subset (see supported instructions below).

Supported instruction categories
- R-type (register-register ALU): ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- I-type (register-immediate ALU): ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
- Loads (I-type): LW (32-bit aligned load)
- Stores (S-type): SW (32-bit aligned store)
- Branches (B-type): BEQ, BNE, BLT, BGE, BLTU, BGEU
- Jumps: JAL (J-type), JALR (I-type with rd and rs1)
- Upper immediate: LUI, AUIPC (U-type)
- SYSTEM: ECALL/EBREAK are recognized by the decoder (further handling is elsewhere)

Immediate formats (how the decoder builds the immediate field)
- I-type: sign-extend instr[31:20]
- S-type: sign-extend {instr[31:25], instr[11:7]}
- B-type: sign-extend {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}
- U-type: {instr[31:12], 12'b0}
- J-type: sign-extend {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}

Shifts and immediates
- Shift-immediates (SLLI, SRLI, SRAI) use the low 5 bits of instr[24:20] for RV32.

Control signals
- The decoder produces a `ctrl_t` structure (see `PipeTypes.svh`) containing fields such as:
  - `wb_sel` (WB_ALU, WB_MEM, WB_PC)
  - `op_a_sel`, `op_b_sel` (operand selectors)
  - `alu_op` (ALU operation codes: ALU_ADD, ALU_SUB, ...)
  - `branch_kind` and `jump_kind`
  - `reg_write`, `mem_read`, `mem_write`

JAL/JALR behavior
- JAL and JALR write the return address (PC+4) into `rd` and set the writeback source to `WB_PC`.

Memory alignment and write strobes
- LW/SW assume 32-bit aligned accesses. The EX/MEM pipeline structure includes a 4-bit write strobe (`wstrb`) for byte enables.

Illegal instructions
- The decoder sets an `illegal` flag for encodings it doesn't support. SYSTEM instructions are recognized at decode (ECALL/EBREAK patterns) but are handled later in the pipeline for exceptions/privilege.

Examples (assembly -> decoder behaviour)
- `add x5, x6, x7`   -> R-type: rd=x5, rs1=x6, rs2=x7, `alu_op`=ALU_ADD
- `addi x5, x6, -4`  -> I-type: imm=0xffff_fffc, `op_b_sel`=OP_B_IMM, `alu_op`=ALU_ADD
- `lw x5, 12(x6)`    -> I-type load: imm=12, `mem_read`=1, `wb_sel`=WB_MEM
- `sw x5, 8(x6)`     -> S-type store: imm=8, `mem_write`=1
- `beq x1, x2, 8`    -> B-type: `branch_kind`=BR_BEQ, branch offset = 8
- `jal x1, 256`      -> J-type: `jump_kind`=JMP_JAL, `wb_sel`=WB_PC, write PC+4 to x1
- `jalr x1, x2, 4`   -> I-type JALR: `jump_kind`=JMP_JALR, `wb_sel`=WB_PC, jump to x2+4

Where to look in the source
- `Decoder.sv` — decoding logic and immediate extractors
- `IDStage.sv`  — decoder is invoked here, and `ID_EX_t` is prepared
- `PipeTypes.svh` — pipeline structs and `ctrl_t` (control encodings)

Next steps I can take
- Add a SystemVerilog testbench that feeds example instructions into the decoder and prints/asserts the outputs.
- Replace `README.md` with this text (I can do this if you want me to overwrite the current README).

If you'd like the `README.md` itself replaced, say so and I'll overwrite it directly. Otherwise, `README_ISA.md` is now in the repo with the ISA docs.
