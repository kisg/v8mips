// Copyright 2006-2008 the V8 project authors. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef  V8_MIPS_CONSTANTS_H_
#define  V8_MIPS_CONSTANTS_H_

// UNIMPLEMENTED_ macro for MIPS.
#define UNIMPLEMENTED_()                         \
  printf("%s, \tline %d: \tfunction %s not implemented. \n", \
         __FILE__, __LINE__, __func__);

// Defines constants and accessor classes to assemble, disassemble and
// simulate MIPS32 instructions.
//
// See  MIPS32 Architecture For Programmers
//     Volume II: The MIPS32 Instruction Set
// ( cf www.mips.com )

namespace assembler {
namespace mips {

// -----------------------------------------------------------------------------
// Registers and CRegisters

// Number of general purpose registers
static const int kNumRegisters = 32;
static const int kInvalidRegister = -1;

// Number of registers with HI, LO, and pc.
static const int kNumSimuRegisters = 35;

// In the simulator, the PC register is simulated as the 34th register
static const int kPCRegister = 34;

// Number coprocessor registers
static const int kNumCRegisters = 32;
static const int kInvalidCRegister = -1;

// Helper functions for converting between register numbers and names.
class Registers {
 public:
  // Return the name of the register.
  static const char* Name(int reg);

  // Lookup the register number for the name provided.
  static int Number(const char* name);

  struct RegisterAlias {
    int reg;
    const char *name;
  };

  static int32_t max_val() { return max_val_; }
  static int32_t min_val() { return min_val_; }

 private:
  static const int32_t max_val_ = 0x7fffffff;
  static const int32_t min_val_ = 0x80000000;

  static const char* names_[kNumSimuRegisters];
  static const RegisterAlias aliases_[];
};

// Helper functions for converting between register numbers and names.
class CRegisters {
 public:
  // Return the name of the register.
  static const char* Name(int reg);

  // Lookup the register number for the name provided.
  static int Number(const char* name);

  struct RegisterAlias {
    int creg;
    const char *name;
  };

 private:

  static const char* names_[kNumCRegisters];
  static const RegisterAlias aliases_[];
};


// -----------------------------------------------------------------------------
// Instructions encoding constants.

// On MIPS all instructions are 32 bits.
typedef int32_t Instr;

typedef unsigned char byte;

// Special Software Interrupt codes when used in the presence of the ARM
// simulator.
enum SoftwareInterruptCodes {
  // transition to C code
  call_rt_redirected = 0xfffff
};

// ----- Fields offset (_o) and length (_l)
static const int opcode_o   = 26;
static const int opcode_l   = 6;
static const int rs_o       = 21;
static const int rs_l       = 5;
static const int rt_o       = 16;
static const int rt_l       = 5;
static const int rd_o       = 11;
static const int rd_l       = 5;
static const int sa_o       = 6;
static const int sa_l       = 5;
static const int function_o = 0;
static const int function_l = 6;

static const int imm16_o = 0;
static const int imm16_l = 16;
static const int imm26_o = 0;
static const int imm26_l = 26;

static const int fs_o       = 11;
static const int fs_l       = 5;
static const int ft_o       = 16;
static const int ft_l       = 5;

// ----- MIPS Opcodes and Function Fields
// We use this presentation to stay close to the table representation in
// MIPS32 Architecture For Programmers, Volume II: The MIPS32 Instruction Set
enum Opcode {
  SPECIAL   =   0 << opcode_o,
  REGIMM    =   1 << opcode_o,

  J         =   ((0 << 3) + 2) << opcode_o,
  JAL       =   ((0 << 3) + 3) << opcode_o,
  BEQ       =   ((0 << 3) + 4) << opcode_o,
  BNE       =   ((0 << 3) + 5) << opcode_o,
  BLEZ      =   ((0 << 3) + 6) << opcode_o,
  BGTZ      =   ((0 << 3) + 7) << opcode_o,

  ADDI      =   ((1 << 3) + 0) << opcode_o,
  ADDIU     =   ((1 << 3) + 1) << opcode_o,
  SLTI      =   ((1 << 3) + 2) << opcode_o,
  SLTIU     =   ((1 << 3) + 3) << opcode_o,
  ANDI      =   ((1 << 3) + 4) << opcode_o,
  ORI       =   ((1 << 3) + 5) << opcode_o,
  XORI      =   ((1 << 3) + 6) << opcode_o,
  LUI       =   ((1 << 3) + 7) << opcode_o,

  COP1      =   ((2 << 3) + 1) << opcode_o,  // Coprocessor 1 class
  BEQL      =   ((2 << 3) + 4) << opcode_o,
  BNEL      =   ((2 << 3) + 5) << opcode_o,
  BLEZL     =   ((2 << 3) + 6) << opcode_o,
  BGTZL     =   ((2 << 3) + 7) << opcode_o,

  SPECIAL2  =   ((3 << 3) + 4) << opcode_o,

  LB        =   ((4 << 3) + 0) << opcode_o,
  LW        =   ((4 << 3) + 3) << opcode_o,
  LBU       =   ((4 << 3) + 4) << opcode_o,
  SB        =   ((5 << 3) + 0) << opcode_o,
  SW        =   ((5 << 3) + 3) << opcode_o,

  LWC1      =   ((6 << 3) + 1) << opcode_o,
  LDC1      =   ((6 << 3) + 5) << opcode_o,

  SWC1      =   ((7 << 3) + 1) << opcode_o,
  SDC1      =   ((7 << 3) + 5) << opcode_o
};

enum SecondaryField {
  // SPECIAL Encoding of Function Field
  SLL       =   ((0 << 3) + 0),
  SRL       =   ((0 << 3) + 2),
  SRA       =   ((0 << 3) + 3),
  SLLV      =   ((0 << 3) + 4),
  SRLV      =   ((0 << 3) + 6),
  SRAV      =   ((0 << 3) + 7),

  JR        =   ((1 << 3) + 0),
  JALR      =   ((1 << 3) + 1),
  BREAK     =   ((1 << 3) + 5),

  MFHI      =   ((2 << 3) + 0),
  MFLO      =   ((2 << 3) + 2),

  MULT      =   ((3 << 3) + 0),
  MULTU     =   ((3 << 3) + 1),
  DIV       =   ((3 << 3) + 2),
  DIVU      =   ((3 << 3) + 3),

  ADD       =   ((4 << 3) + 0),
  ADDU      =   ((4 << 3) + 1),
  SUB       =   ((4 << 3) + 2),
  SUBU      =   ((4 << 3) + 3),
  AND       =   ((4 << 3) + 4),
  OR        =   ((4 << 3) + 5),
  XOR       =   ((4 << 3) + 6),
  NOR       =   ((4 << 3) + 7),

  SLT       =   ((5 << 3) + 2),
  SLTU      =   ((5 << 3) + 3),

  TGE       =   ((6 << 3) + 0),
  TGEU      =   ((6 << 3) + 1),
  TLT       =   ((6 << 3) + 2),
  TLTU      =   ((6 << 3) + 3),
  TEQ       =   ((6 << 3) + 4),
  TNE       =   ((6 << 3) + 6),

  // SPECIAL2 Encoding of Function Field
  MUL       =   ((0 << 3) + 2),

  // REGIMM  encoding of rt Field
  BLTZ      =   ((0 << 3) + 0) << 16,
  BGEZ      =   ((0 << 3) + 1) << 16,
  BLTZAL    =   ((2 << 3) + 0) << 16,
  BGEZAL    =   ((2 << 3) + 1) << 16,

  // COP1 Encoding of rs Field
  MFC1      =   ((0 << 3) + 0) << 21,
  MFHC1     =   ((0 << 3) + 3) << 21,
  MTC1      =   ((0 << 3) + 4) << 21,
  MTHC1     =   ((0 << 3) + 7) << 21,
  BC1       =   ((1 << 3) + 0) << 21,
  S         =   ((2 << 3) + 0) << 21,
  D         =   ((2 << 3) + 1) << 21,
  W         =   ((2 << 3) + 4) << 21,
  L         =   ((2 << 3) + 5) << 21,
  PS        =   ((2 << 3) + 6) << 21,
  // COP1 Encoding of Function Field When rs=S
  CVT_D_S   =   ((4 << 3) + 1),
  CVT_W_S   =   ((4 << 3) + 4),
  CVT_L_S   =   ((4 << 3) + 5),
  CVT_PS_S  =   ((4 << 3) + 6),
  // COP1 Encoding of Function Field When rs=D
  CVT_S_D   =   ((4 << 3) + 0),
  CVT_W_D   =   ((4 << 3) + 4),
  CVT_L_D   =   ((4 << 3) + 5),
  // COP1 Encoding of Function Field When rs=W or L
  CVT_S_W   =   ((4 << 3) + 0),
  CVT_D_W   =   ((4 << 3) + 1),
  CVT_S_L   =   ((4 << 3) + 0),
  CVT_D_L   =   ((4 << 3) + 1),
  // COP1 Encoding of Function Field When rs=PS

  NULLSF    =   0
};

// ----- Miscellianous useful masks.
enum {
  // Instruction bit masks
  OpcodeMask    =   0x3f << opcode_o,
  Imm16Mask     =   0xffff,
  Imm26Mask     =   0x3fffffff,
  rsFieldMask   =   0x1f << rs_o,
  rtFieldMask   =   0x1f << rt_o,
  rdFieldMask   =   0x1f << rd_o,
  saFieldMask   =   0x1f << sa_o,
  functionFieldMask   =   0x3f << function_o,
  // Misc masks
  HIMask        =   0xffff << 16,
  LOMask        =   0xffff,
  signMask      =   0x80000000
};


// ----- Emulated conditions
// On MIPS we use this enum to abstract from conditionnal branch instructions.
// the 'U' prefix is used to specify unsigned comparisons.
enum Condition {
  // any value < 0 is considered no_condition
  no_condition  = -1,

  overflow      =  0,
  no_overflow   =  1,
  Uless         =  2,
  Ugreater_equal=  3,
  equal         =  4,
  not_equal     =  5,
  Uless_equal   =  6,
  Ugreater      =  7,
  negative      =  8,
  positive      =  9,
  parity_even   = 10,
  parity_odd    = 11,
  less          = 12,
  greater_equal = 13,
  less_equal    = 14,
  greater       = 15,

  cc_always     = 16,

  // aliases
  carry         = Uless,
  not_carry     = Ugreater_equal,
  zero          = equal,
  eq            = equal,
  not_zero      = not_equal,
  ne            = not_equal,
  sign          = negative,
  not_sign      = positive,

  cc_default    = no_condition
};

// ----- Coprocessor conditions
enum C_Condition {
  F,    // False
  UN,   // Unordered
  EQ,   // Equal
  UEQ,  // Unordered or Equal
  OLT,  // Ordered or Less Than
  ULT,  // Unordered or Less Than
  OLE,  // Ordered or Less Than or Equal
  ULE   // Unordered or Less Than or Equal
};


// break 0xfffff, reserved for redirected real time call
const Instr rtCallRedirInstr = SPECIAL | BREAK | call_rt_redirected<<6;

// The class Instruction enables access to individual fields defined in the ARM
// architecture instruction set encoding as described in figure A3-1.
//
// Example: Test whether the instruction at ptr does set the condition code
// bits.
//
// bool InstructionSetsConditionCodes(byte* ptr) {
//   Instruction* instr = Instruction::At(ptr);
//   int type = instr->TypeField();
//   return ((type == 0) || (type == 1)) && instr->HasS();
// }
//
class Instruction {
 public:
  enum {
    kInstructionSize = 4,
    kInstructionSizeLog2 = 2,
    // On MIPS PC cannot actually be directly accessed. We behave as if PC was
    // always the value of the current instruction being exectued.
    kPCReadOffset = 0
  };

  // Get the raw instruction bits.
  inline Instr InstructionBits() const {
    return *reinterpret_cast<const Instr*>(this);
  }

  // Set the raw instruction bits to value.
  inline void SetInstructionBits(Instr value) {
    *reinterpret_cast<Instr*>(this) = value;
  }

  // Read one particular bit out of the instruction bits.
  inline int Bit(int nr) const {
    return (InstructionBits() >> nr) & 1;
  }

  // Read a bit field out of the instruction bits.
  inline int Bits(int hi, int lo) const {
    return (InstructionBits() >> lo) & ((2 << (hi - lo)) - 1);
  }


  // Accessors for the different named fields used in the MIPS encoding.
  // Generally applicable fields

  // Fields used in Data processing instructions
  inline Opcode OpcodeField() const {
    return static_cast<Opcode>(Bits(opcode_o + opcode_l - 1, opcode_o));
  }

  inline int rsField() const {
    return Bits(rs_o + rs_l - 1, rs_o);
  }

  inline int rtField() const {
    return Bits(rt_o + rt_l - 1, rt_o);
  }

  inline int rdField() const {
    return Bits(rd_o + rd_l - 1, rd_o);
  }

  inline int saField() const {
    return Bits(sa_o + sa_l - 1, sa_o);
  }

  inline int functionField() const {
    return Bits(function_o + function_l - 1, function_o);
  }

  inline int fsField() const {
    return Bits(fs_o + rs_l - 1, fs_o);
  }

  inline int ftField() const {
    return Bits(ft_o + rs_l - 1, ft_o);
  }

  // Return the fields at their original place in the instruction encoding.
  inline Opcode OpcodeFieldRaw() const {
    return static_cast<Opcode>(InstructionBits() & OpcodeMask);
  }

  inline int rsFieldRaw() const {
    return InstructionBits() & rsFieldMask;
  }

  inline int rtFieldRaw() const {
    return InstructionBits() & rtFieldMask;
  }

  inline int rdFieldRaw() const {
    return InstructionBits() & rdFieldMask;
  }

  inline int saFieldRaw() const {
    return InstructionBits() & saFieldMask;
  }

  inline int functionFieldRaw() const {
    return InstructionBits() & functionFieldMask;
  }

  // Get the secondary field according to the opcode.
  inline int secondaryField() const {
    Opcode op = OpcodeFieldRaw();
    switch (op) {
      case SPECIAL:
      case SPECIAL2:
        return functionField();
        break;
      case COP1:
        return rsField();
        break;
      case REGIMM:
        return rtField();
        break;
      default:
        return NULLSF;
    }
  }

  inline int32_t Imm16Field() const {
    return Bits(imm16_o + imm16_l - 1, imm16_o);
  }

  inline int32_t Imm26Field() const {
    return Bits(imm16_o + imm26_l - 1, imm26_o);
  }

  // Say if the instruction should not be used in a branch delay slot.
  bool isForbiddenInBranchDelay();
  // Say if the instruction 'links'. eg: jal, bal
  bool isLinkingInstruction();
  // Say if the instruction is a break or a trap.
  bool isTrap();

  // Get the encoding type of the instruction.
  // type 1 is imm16 type
  // type 2 is register type
  // type 3 is imm26 type
  int instrType();

  // Instructions are read of out a code stream. The only way to get a
  // reference to an instruction is to convert a pointer. There is no way
  // to allocate or create instances of class Instruction.
  // Use the At(pc) function to create references to Instruction.
  static Instruction* At(byte* pc) {
    return reinterpret_cast<Instruction*>(pc);
  }

 private:
  // We need to prevent the creation of instances of class Instruction.
  DISALLOW_IMPLICIT_CONSTRUCTORS(Instruction);
};


// -----------------------------------------------------------------------------
// MIPS assembly various constants

static const int argsSlotsSize  = 4*Instruction::kInstructionSize;
static const int argsSlotsNum   = 4;


} }   // namespace assembler::mips

#endif    // #ifndef V8_MIPS_CONSTANTS_H_
