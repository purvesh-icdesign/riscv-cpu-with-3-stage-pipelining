<img width="1024" height="852" alt="image" src="https://github.com/user-attachments/assets/a2d6dd4c-4b4c-4038-8fd5-673723636478" />
<br><br>

# 🚀 RISC-V RV32I CPU Core with UART Interface

## 📌 Overview

This project implements a **32-bit RISC-V (RV32I) CPU core** using a **multi-cycle FSM-based architecture**, integrated with a **unified memory system** and a **memory-mapped UART interface**.

The design is **FPGA-friendly**, fully synthesizable, and supports:
- Instruction execution  
- Memory access  
- Serial communication via UART  

---

## 🧠 Architecture

- **ISA**: RV32I (Base Integer Instruction Set)  
- **Design Style**: Multi-cycle FSM  
  *(Fetch → Decode → Execute → Memory → Writeback)*  
- **Memory Model**: Unified instruction + data memory  
- **Peripherals**: Memory-mapped UART transmitter  
- **Register File**: 32 × 32-bit registers (x0–x31)  
  - `x0` is hardwired to zero  

---

## 📂 Project Structure
├── cpu.v # Core CPU implementation<br>
├── progmem.v # Unified instruction/data memory<br>
├── uart_gpio.v # Memory-mapped UART interface<br>
├── uart_tx.v # UART transmitter<br>
├── top.v # Top-level integration module<br>
├── firmware.hex # Program memory initialization<br>



---

## ⚙️ CPU Core Features

### ✅ Supported Instructions

#### 🔹 R-Type
- ADD, SUB, AND, OR, XOR  
- SLT, SLTU  
- SLL, SRL  

#### 🔹 I-Type
- ADDI, ANDI, ORI, XORI  
- SLTI, SLTIU  
- SLLI, SRLI  

#### 🔹 Load
- LW, LH, LB, LHU, LBU  

#### 🔹 Store
- SW, SH, SB  

#### 🔹 Branch
- BEQ, BNE, BLT, BGE, BLTU, BGEU  

#### 🔹 Jump
- JAL, JALR  

#### 🔹 Upper Immediate
- LUI, AUIPC  

---
