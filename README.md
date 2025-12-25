# RISC-V Pipelined Processor (RV32I + M)

## Overview

This project implements a **32-bit RISC-V pipelined processor** that transitions from a **single-cycle design** to a **multi-stage pipelined architecture** with a **unified single-port memory** for both instructions and data.

The processor supports **all RV32I instructions** in addition to **multiplication instructions (RV32M)**.  
The pipelined design improves performance through instruction overlap while preserving correctness using hazard detection, forwarding, and enhanced control logic.

---
## Key Features

- **5-stage pipelined architecture:** IF → ID → EX → MEM → WB  
- **Full RV32I instruction set support**  
- **Multiplication instructions (RV32M)** fully implemented  
- **Unified single-port instruction & data memory**  
- **Data hazard handling** using forwarding  
- **Load-use hazard detection** and pipeline stalling  
- **Branch control hazard handling** with pipeline flushing  
- **Seven-segment display (SSD) visualization**  
- **Automated test generation (bonus)**

---
## Block Diagram & Simulation for the First Program

<img src="https://github.com/user-attachments/assets/ce71df87-707c-48dc-aa85-21b19bdea88a" width="400"/>
<img src="https://github.com/user-attachments/assets/447fa511-d7e0-442a-8cd0-d524789a6a9f" width="500"/>


## Architecture Overview

- Pipeline registers between all stages to enable instruction overlap  
- Hazard Detection Unit to manage load-use and structural hazards  
- Forwarding Unit to reduce pipeline stalls  
- Branch resolution in the EX stage with pipeline flushing  
- Single unified memory accessed via clock-phase multiplexing

---

## Instruction Set Support

- **Base ISA:** RV32I (all instructions)  
- **Extension:** RV32M (multiplication instructions)

---

## Module Descriptions

### Core Modules

- **`32-bit_ALU.v`** – Implements arithmetic and logic operations, including RV32M multiplication instructions, in the EX stage with support for operand forwarding.  
- **`control_unit.v`** – Generates control signals, pipelined through all stages, with flush and bubble control for hazard management.  
- **`ALU_control_unit.v`** – Produces ALU-specific control signals, supporting pipelined decoding and RV32M operations.  
- **`Register_File.v`** – Dual read ports (ID stage) and single write port (WB stage) with hazard and forwarding support.  
- **`Uni_mem.v`** – Unified, single-port, byte-addressable memory for both instructions and data. Supports byte, half-word, and word accesses with clock-phase multiplexing.

### Hazard & Control Modules

- **`ForwardingUnit.v`** – Resolves data hazards by forwarding results from later stages to ALU inputs.  
- **`Hazard_Detection_Unit.v`** – Detects load-use and structural hazards and inserts stalls/bubbles when necessary.  
- **`branch.v`** – Executes branch logic in EX stage and generates the `PCSrc` signal.  
- **`Branch_decision.v`** – Determines branch outcomes using `funct3` and ALU flags (Zero, Sign, Carry, Overflow).

### Supporting Modules

- **`immediate_generator.v`** – Extracts immediate values in the ID stage and forwards them through pipeline registers.  
- **`Shifter.v` / `Shift_Left.v`** – Performs shift operations and address calculations in EX stage.  
- **`top_riscv_ssd.v`** – Integrates all pipeline stages, hazard detection, forwarding, and unified memory. Connects processor signals to SSD for visualization.  
- **`ssd_driver.v`** – Drives seven-segment display using pipeline stage outputs.

---

## Pipeline Registers

- IF / ID  
- ID / EX  
- EX / MEM  
- MEM / WB  

These registers store instructions, operands, immediates, and control signals to maintain correctness across stages.

---

## Branch Handling

- Branches are resolved in **EX stage**  
- If taken:
  - Pipeline is flushed  
  - PC is updated to branch target  
- Minimizes incorrect instruction execution due to control hazards

---

## Test Case Generator

A **C++ program** generates randomized RISC-V test cases:

- Generates instructions in:
  - Binary
  - Assembly
  - Hexadecimal
- Supports **all RV32M instructions**
- Enhances testing coverage and robustness

[RISC-V Test Generator Repository](https://github.com/Rawan10101/RISC-V-testcase-generator)

---

## Conclusion

This project demonstrates a complete transition from a **single-cycle RISC-V processor** to a fully functional **pipelined architecture** with unified memory and robust hazard management.  
Supports **RV32I + multiplication instructions**, making it suitable for **educational purposes and architectural exploration**.

---

