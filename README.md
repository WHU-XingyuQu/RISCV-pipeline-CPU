# RISC-V RV32I Five-Stage Pipelined CPU

## Overview

This is a complete implementation of a RISC-V RV32I five-stage pipelined CPU, supporting all 37 standard instructions with forwarding unit, hazard detection unit, and basic interrupt handling mechanisms.

## Features

### 1. Five-Stage Pipeline Architecture

$$
\text{IF} \rightarrow \text{ID} \rightarrow \text{EX} \rightarrow \text{MEM} \rightarrow \text{WB}
$$

- **IF (Instruction Fetch)**: Fetch instruction from memory
- **ID (Instruction Decode)**: Decode instruction and read registers
- **EX (Execute)**: Execute arithmetic/logic operations
- **MEM (Memory Access)**: Access data memory
- **WB (Write Back)**: Write result back to register file

### 2. Supported Instruction Set (RV32I - 37 Instructions)

#### Arithmetic Instructions
- `ADD`, `ADDI`, `SUB`
- `SLT`, `SLTI`, `SLTU`, `SLTIU`

#### Logical Instructions
- `AND`, `ANDI`, `OR`, `ORI`, `XOR`, `XORI`

#### Shift Instructions
- `SLL`, `SLLI`, `SRL`, `SRLI`, `SRA`, `SRAI`

#### Load/Store Instructions
- `LB`, `LH`, `LW`, `LBU`, `LHU`
- `SB`, `SH`, `SW`

#### Branch Instructions
- `BEQ`, `BNE`, `BLT`, `BGE`, `BLTU`, `BGEU`

#### Jump Instructions
- `JAL`, `JALR`

#### Upper Immediate Instructions
- `LUI`, `AUIPC`

#### System Instructions
- `FENCE` (treated as NOP)
- `ECALL`, `EBREAK` (basic support)

### 3. Hazard Handling

#### Data Hazards

**Forwarding Unit**
- Resolves EX-EX and MEM-EX data hazards
- Forwards data from EX/MEM pipeline register to EX stage
- Forwards data from MEM/WB pipeline register to EX stage

The forwarding logic can be expressed as:

$$
\text{operand} = \begin{cases}
\text{EX/MEM result} & \text{if EX hazard detected} \\
\text{MEM/WB result} & \text{if MEM hazard detected} \\
\text{ID/EX operand} & \text{otherwise}
\end{cases}
$$

**Hazard Detection Unit**
- Detects Load-Use hazards
- Inserts pipeline bubbles (stalls) when necessary
- Maintains PC and IF/ID registers unchanged during stall

Load-Use hazard detection condition:

$$
\text{Stall} = \text{ID/EX.MemRead} \land (\text{ID/EX.rd} \neq 0) \land 
$$
$$
((\text{ID/EX.rd} = \text{IF/ID.rs1}) \lor (\text{ID/EX.rd} = \text{IF/ID.rs2}))
$$

#### Control Hazards
- Simple "not-taken" branch prediction
- Branch condition calculated in EX stage
- Flushes IF and ID stages on misprediction

Branch penalty:

$$
\text{Penalty} = \begin{cases}
0 \text{ cycles} & \text{if prediction correct} \\
2 \text{ cycles} & \text{if prediction incorrect}
\end{cases}
$$

### 4. Interrupt Handling

#### Supported Interrupt Types
- External Interrupt
- Timer Interrupt
- Software Interrupt

#### CSR Registers
- `mstatus`: Machine status register
- `mie`: Machine interrupt enable register
- `mip`: Machine interrupt pending register
- `mtvec`: Machine trap-handler base address
- `mepc`: Machine exception program counter
- `mcause`: Machine trap cause
- `mscratch`: Scratch register
- `misa`: ISA information

#### Interrupt Priority

$$
\text{Priority: External} > \text{Software} > \text{Timer}
$$

## Module Structure

```
riscv_cpu.v                 # Top-level CPU module
├── control_unit.v          # Control unit
├── alu.v                   # Arithmetic Logic Unit
├── imm_gen.v               # Immediate generator
├── register_file.v         # Register file (32 registers)
├── forwarding_unit.v       # Forwarding unit
├── hazard_detection_unit.v # Hazard detection unit
├── branch_comp.v           # Branch comparator
├── mem_controller.v        # Memory write controller
├── mem_read_aligner.v      # Memory read aligner
└── csr_unit.v              # CSR and interrupt controller
```

## Interface Specification

### Top-Level Interface

```verilog
module riscv_cpu (
    input wire clk,                    // Clock
    input wire rst_n,                  // Active-low reset
    
    // Instruction memory interface
    output wire [31:0] imem_addr,      // Instruction address
    input wire [31:0] imem_rdata,      // Instruction data
    
    // Data memory interface
    output wire [31:0] dmem_addr,      // Data address
    output wire [31:0] dmem_wdata,     // Write data
    output wire [3:0] dmem_we,         // Byte write enable
    output wire dmem_re,               // Read enable
    input wire [31:0] dmem_rdata,      // Read data
    
    // Interrupt interface
    input wire external_interrupt,     // External interrupt
    input wire timer_interrupt,        // Timer interrupt
    input wire software_interrupt      // Software interrupt
);
```

### Memory Interface

- **Instruction Memory**: 32-bit address, 32-bit data, read-only
- **Data Memory**: 32-bit address, 32-bit data, supports byte/half-word/word access
- Byte enable signals support `SB` and `SH` instructions

## Usage

### Simulation and Testing

1. **Compile with iverilog**:
```bash
iverilog -o riscv_cpu_sim riscv_cpu.v control_unit.v alu.v imm_gen.v \
         register_file.v forwarding_unit.v hazard_detection_unit.v \
         branch_comp.v mem_controller.v mem_read_aligner.v csr_unit.v \
         riscv_cpu_tb.v
```

2. **Run simulation**:
```bash
vvp riscv_cpu_sim
```

3. **View waveform**:
```bash
gtkwave riscv_cpu.vcd
```

### Using Makefile

```bash
make compile    # Compile the design
make sim        # Run simulation
make wave       # View waveform
make clean      # Clean generated files
```

### Synthesis

Compatible with various FPGA toolchains:
- Xilinx Vivado
- Intel Quartus
- Open-source toolchain (yosys + nextpnr)

## Performance Metrics

### Theoretical CPI (Cycles Per Instruction)

$$
\text{CPI}_{\text{ideal}} = 1.0
$$

$$
\text{CPI}_{\text{actual}} = 1.0 + P_{\text{load-use}} \times 1 + P_{\text{branch}} \times 2 + P_{\text{interrupt}} \times N
$$

Where:
- $P_{\text{load-use}}$: Probability of Load-Use hazard
- $P_{\text{branch}}$: Probability of branch misprediction
- $P_{\text{interrupt}}$: Probability of interrupt occurrence
- $N$: Interrupt handling overhead

### Critical Paths

1. **ALU Path**: Register Read → Forward MUX → ALU → Result
2. **Branch Path**: Register Read → Forward MUX → Branch Comparator → PC Update
3. **Memory Access Path**: ALU → Address Alignment → Memory Access → Data Alignment

## Design Details

### Pipeline Stages

The pipeline implements the classic RISC design:

```
┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
│   IF    │ → │   ID    │ → │   EX    │ → │   MEM   │ → │   WB    │
└─────────┘   └─────────┘   └─────────┘   └─────────┘   └─────────┘
     │             │             │             │             │
     ▼             ▼             ▼             ▼             ▼
  IF/ID         ID/EX         EX/MEM        MEM/WB      Register
Pipeline Reg  Pipeline Reg  Pipeline Reg  Pipeline Reg    File
```

### Pipeline Flushing

Pipeline is flushed under the following conditions:
1. Branch taken (misprediction)
2. Interrupt response
3. `MRET` instruction

### Register x0

- Register `x0` is hardwired to zero
- Writes to `x0` are ignored

$$
\text{x0} \equiv 0, \quad \forall t
$$

### Memory Alignment

- Load/Store instructions automatically handle byte alignment
- Unaligned access implemented through shifting and masking

## Test Programs

The testbench includes tests for:
1. Basic arithmetic operations
2. Logical operations
3. Load/Store operations
4. Branch and jump instructions
5. Loop execution
6. Interrupt handling

Example test sequence:

```assembly
    addi x2, x0, 5          # x2 = 5
    addi x3, x0, 10         # x3 = 10
    add  x1, x2, x3         # x1 = 15
    sw   x1, 0(x2)          # mem[5] = 15
    lw   x4, 0(x2)          # x4 = mem[5]
    beq  x1, x4, target     # branch if equal
```

## Extension Possibilities

### Performance Optimizations

1. **Dynamic Branch Prediction**
   - 2-bit saturating counter
   - Branch Target Buffer (BTB)
   - Return Address Stack (RAS)

2. **Superscalar Execution**
   - Dual-issue or more
   - Out-of-order execution using Tomasulo's algorithm

3. **Advanced Forwarding**
   - Full forwarding network
   - Bypass paths for all pipeline stages

### Instruction Set Extensions

1. **M Extension**: Integer multiplication and division
2. **A Extension**: Atomic instructions
3. **F/D Extension**: Single/double-precision floating-point
4. **C Extension**: Compressed 16-bit instructions

### System Extensions

1. **Cache System**
   - L1 Instruction Cache
   - L1 Data Cache
   - Translation Lookaside Buffer (TLB)

2. **Privilege Levels**
   - User mode support
   - Supervisor mode support
   - Virtual memory with page tables

3. **Multi-core Support**
   - Symmetric Multi-Processing (SMP)
   - Cache coherence protocol

## Resource Estimation

### Register Resources

$$
\begin{align*}
\text{Pipeline Registers} &\approx 400 \text{ bits} \\
\text{PC Register} &= 32 \text{ bits} \\
\text{Register File} &= 32 \times 32 = 1024 \text{ bits} \\
\text{CSR Registers} &\approx 256 \text{ bits} \\
\hline
\text{Total} &\approx 1700 \text{ bits}
\end{align*}
$$

### Logic Resources (FPGA)

- ALU: ~500 LUTs
- Control Logic: ~200 LUTs
- Forwarding Unit: ~100 LUTs
- Hazard Detection: ~50 LUTs
- Other Logic: ~150 LUTs
- **Total**: ~1000 LUTs

## Timing Analysis

### Critical Path Timing

Assuming target frequency of 100 MHz (period = 10 ns):

$$
\begin{align*}
T_{\text{clk}} &= 10 \text{ ns} \\
T_{\text{setup}} &= 0.5 \text{ ns} \\
T_{\text{clk-to-q}} &= 0.8 \text{ ns} \\
T_{\text{logic}} &= T_{\text{clk}} - T_{\text{setup}} - T_{\text{clk-to-q}} \\
&= 10 - 0.5 - 0.8 = 8.7 \text{ ns}
\end{align*}
$$

Critical path must complete within 8.7 ns.

## Verification Strategy

### Unit Testing
- ALU: All arithmetic and logic operations
- Control Unit: Control signals for all instructions
- Forwarding Unit: Various hazard scenarios

### Integration Testing
- Basic instruction sequences
- Hazard scenarios
- Branch and jump instructions
- Load/Store operations
- Interrupt handling

### Performance Testing
- Dhrystone benchmark
- CoreMark benchmark
- Custom performance test programs

## Documentation

- **README.md**: Project overview and usage guide
- **INSTRUCTION_SET.md**: Detailed RV32I instruction set reference
- **DESIGN_DOCUMENT.md**: Comprehensive pipeline design documentation

## Notes

1. This implementation is for educational and research purposes
2. Interrupt handling is simplified and doesn't fully comply with RISC-V privileged specification
3. Not all CSR registers are implemented
4. `FENCE` instruction is treated as NOP

## References

1. Waterman, A., & Asanović, K. (Eds.). (2019). *The RISC-V Instruction Set Manual, Volume I: User-Level ISA*
2. Waterman, A., & Asanović, K. (Eds.). (2019). *The RISC-V Instruction Set Manual, Volume II: Privileged Architecture*
3. Patterson, D. A., & Hennessy, J. L. (2017). *Computer Organization and Design RISC-V Edition: The Hardware Software Interface*
4. Harris, S., & Harris, D. (2021). *Digital Design and Computer Architecture, RISC-V Edition*

## License

MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Author

Created by: Claude AI Assistant

## Version History

- **v1.0** (January 2026): Initial release with RV32I base instruction set support

## Contact

For questions, issues, or contributions, please open an issue on GitHub.

---

**Star ⭐ this repository if you find it helpful!**
