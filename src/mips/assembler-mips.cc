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


#include "v8.h"
#include "mips/assembler-mips-inl.h"
#include "serialize.h"


namespace v8 {
namespace internal {



const Register no_reg = { -1 };

const Register zero_reg = { 0 };
const Register at = { 1 };
const Register v0 = { 2 };
const Register v1 = { 3 };
const Register a0 = { 4 };
const Register a1 = { 5 };
const Register a2 = { 6 };
const Register a3 = { 7 };
const Register t0 = { 8 };
const Register t1 = { 9 };
const Register t2 = { 10 };
const Register t3 = { 11 };
const Register t4 = { 12 };
const Register t5 = { 13 };
const Register t6 = { 14 };
const Register t7 = { 15 };
const Register s0 = { 16 };
const Register s1 = { 17 };
const Register s2 = { 18 };
const Register s3 = { 19 };
const Register s4 = { 20 };
const Register s5 = { 21 };
const Register s6 = { 22 };
const Register s7 = { 23 };
const Register t8 = { 24 };
const Register t9 = { 25 };
const Register k0 = { 26 };
const Register k1 = { 27 };
const Register gp = { 28 };
const Register sp = { 29 };
const Register s8_fp = { 30 };
const Register ra = { 31 };


const CRegister no_creg = { -1 };

const CRegister f0 = { 0 };
const CRegister f1 = { 1 };
const CRegister f2 = { 2 };
const CRegister f3 = { 3 };
const CRegister f4 = { 4 };
const CRegister f5 = { 5 };
const CRegister f6 = { 6 };
const CRegister f7 = { 7 };
const CRegister f8 = { 8 };
const CRegister f9 = { 9 };
const CRegister f10 = { 10 };
const CRegister f11 = { 11 };
const CRegister f12 = { 12 };
const CRegister f13 = { 13 };
const CRegister f14 = { 14 };
const CRegister f15 = { 15 };
const CRegister f16 = { 16 };
const CRegister f17 = { 17 };
const CRegister f18 = { 18 };
const CRegister f19 = { 19 };
const CRegister f20 = { 20 };
const CRegister f21 = { 21 };
const CRegister f22 = { 22 };
const CRegister f23 = { 23 };
const CRegister f24 = { 24 };
const CRegister f25 = { 25 };
const CRegister f26 = { 26 };
const CRegister f27 = { 27 };
const CRegister f28 = { 28 };
const CRegister f29 = { 29 };
const CRegister f30 = { 30 };
const CRegister f31 = { 31 };

int ToNumber(Register reg) {
  ASSERT(reg.is_valid());
  const int kNumbers[] = {
		0,	// zero_reg
		1,	// at
		2,	// v0
		3,	// v1
		4,	// a0
		5,	// a1
		6,	// a2
		7,	// a3
		8,	// t0
		9,	// t1
		10,	// t2
		11,	// t3
		12,	// t4
		13,	// t5
		14,	// t6
		15,	// t7
		16,	// s0
		17,	// s1
		18,	// s2
		19,	// s3
		20,	// s4
		21,	// s5
		22,	// s6
		23,	// s7
		24,	// t8
		25,	// t9
		26,	// k0
		27,	// k1
		28,	// gp
		29,	// sp
		30,	// s8_fp
		31,	// ra
  };
  return kNumbers[reg.code()];
}

Register ToRegister(int num) {
  ASSERT(num >= 0 && num < kNumRegisters);
  const Register kRegisters[] = {
		zero_reg,
		at,
		v0,	v1,
		a0,	a1,	a2,	a3,
		t0,	t1,	t2,	t3,	t4,	t5,	t6,	t7,
		s0,	s1,	s2,	s3,	s4,	s5,	s6,	s7,
		t8,	t9,
		k0,	k1,
		gp,
		sp,
		s8_fp,
		ra
  };
  return kRegisters[num];
}

// -----------------------------------------------------------------------------
// Implementation of RelocInfo

// WII? cf assembler.h 233 :Modes affected by apply. Depends on arch.
const int RelocInfo::kApplyMask = 0;


// Patch the code at the current address with the supplied instructions.
void RelocInfo::PatchCode(byte* instructions, int instruction_count) {
  Instr* pc = reinterpret_cast<Instr*>(pc_);
  Instr* instr = reinterpret_cast<Instr*>(instructions);
  for (int i = 0; i < instruction_count; i++) {
    *(pc + i) = *(instr + i);
  }

  // Indicate that code has changed.
  CPU::FlushICache(pc_, instruction_count * Assembler::kInstrSize);
}


// Patch the code at the current PC with a call to the target address.
// Additional guard instructions can be added if required.
void RelocInfo::PatchCodeWithCall(Address target, int guard_bytes) {
  // Patch the code at the current address with a call to the target.
  UNIMPLEMENTED();
}



// -----------------------------------------------------------------------------
// Implementation of Operand and MemOperand
// See assembler-mips-inl.h for inlined constructors

Operand::Operand(Handle<Object> handle) {
  rm_ = no_reg;
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  ASSERT(!Heap::InNewSpace(obj));
  if (obj->IsHeapObject()) {
    imm32_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // no relocation needed
    imm32_ =  reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE;
  }
}

MemOperand::MemOperand(Register rm, int16_t offset) : Operand(rm) {
  offset_ = offset;
}


// -----------------------------------------------------------------------------
// Implementation of Assembler

static const int kMinimalBufferSize = 4*KB;
static byte* spare_buffer_ = NULL;

Assembler::Assembler(void* buffer, int buffer_size) {
  if (buffer == NULL) {
    // do our own buffer management
    if (buffer_size <= kMinimalBufferSize) {
      buffer_size = kMinimalBufferSize;

      if (spare_buffer_ != NULL) {
        buffer = spare_buffer_;
        spare_buffer_ = NULL;
      }
    }
    if (buffer == NULL) {
      buffer_ = NewArray<byte>(buffer_size);
    } else {
      buffer_ = static_cast<byte*>(buffer);
    }
    buffer_size_ = buffer_size;
    own_buffer_ = true;

  } else {
    // use externally provided buffer instead
    ASSERT(buffer_size > 0);
    buffer_ = static_cast<byte*>(buffer);
    buffer_size_ = buffer_size;
    own_buffer_ = false;
  }

  // setup buffer pointers
  ASSERT(buffer_ != NULL);
  pc_ = buffer_;
  reloc_info_writer.Reposition(buffer_ + buffer_size, pc_);
  current_statement_position_ = RelocInfo::kNoPosition;
  current_position_ = RelocInfo::kNoPosition;
  written_statement_position_ = current_statement_position_;
  written_position_ = current_position_;
}


Assembler::~Assembler() {
  if (own_buffer_) {
    if (spare_buffer_ == NULL && buffer_size_ == kMinimalBufferSize) {
      spare_buffer_ = buffer_;
    } else {
      DeleteArray(buffer_);
    }
  }
}


void Assembler::GetCode(CodeDesc* desc) {
  ASSERT(pc_ <= reloc_info_writer.pos());  // no overlap
  // setup desc
  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
}


void Assembler::Align(int m) {
  ASSERT(m >= 4 && IsPowerOf2(m));
  while ((pc_offset() & (m - 1)) != 0) {
    nop();
  }
}


// Labels refer to positions in the (to be) generated code.
// There are bound, linked, and unused labels.
//
// Bound labels refer to known positions in the already
// generated code. pos() is the position the label refers to.
//
// Linked labels refer to unknown positions in the code
// to be generated; pos() is the position of the last
// instruction using the label.


// The link chain is terminated by a negative code position (must be aligned)
const int kEndOfChain = -4;

bool Assembler::is_branch(Instr instr) {
  uint32_t opcode   = (( instr & OpcodeMask )) ;
  uint32_t rt_field = (( instr & rtFieldMask ));
  uint32_t rs_field = (( instr & rsFieldMask ));
// Checks if the instruction is a branch or a jump (jr and jalr excluded)
  return    opcode == BEQ
         || opcode == BNE
         || opcode == BLEZ
         || opcode == BGTZ
         || opcode == BEQL
         || opcode == BNEL
         || opcode == BLEZL
         || opcode == BGTZL
         || ( opcode == REGIMM &&  ( rt_field == BLTZ
                                  || rt_field == BGEZ 
                                  || rt_field == BLTZAL
                                  || rt_field == BGEZAL ) ) 
         || ( opcode == COP1   &&  rs_field == BC1 );    // Coprocessor branch
}

int Assembler::target_at(int32_t pos)  {
  Instr instr = instr_at(pos);
  if ((instr & ~Imm16Mask) == 0) {
    // Emitted label constant, not part of a branch.
    return instr - (Code::kHeaderSize - kHeapObjectTag);
  }
  // Check we have a branch instruction. (or jump when implemented)
  ASSERT(is_branch(instr));
  // Do NOT change this to <<2. We rely on arithmetic shifts here, assuming
  // the compiler uses arithmectic shifts for signed integers.
  int32_t imm18 = (((int32_t)instr & (int32_t)Imm16Mask) << 16) >>14;

  return pos + kBranchPCOffset + imm18;
}


void Assembler::target_at_put(int32_t pos, int32_t target_pos) {
  Instr instr = instr_at(pos);
  if ((instr & ~Imm16Mask) == 0) {
    ASSERT(target_pos == kEndOfChain || target_pos >= 0);
    // Emitted label constant, not part of a branch.
    // Make label relative to Code* of generated Code object.
    instr_at_put(pos, target_pos + (Code::kHeaderSize - kHeapObjectTag));
    return;
  }

  ASSERT(is_branch(instr));
  int32_t imm18 = target_pos - (pos + kBranchPCOffset);
  ASSERT( (imm18 & 3) == 0 );
  
  instr &= ~Imm16Mask;
  int32_t imm16 = imm18 >> 2;
  ASSERT(is_int16(imm16));
 
  instr_at_put(pos, instr | (imm16 & Imm16Mask));
}

// TODO : Upgrade this.
void Assembler::print(Label* L) {
  if (L->is_unused()) {
    PrintF("unused label\n");
  } else if (L->is_bound()) {
    PrintF("bound label to %d\n", L->pos());
  } else if (L->is_linked()) {
    Label l = *L;
    PrintF("unbound label");
    while (l.is_linked()) {
      PrintF("@ %d ", l.pos());
      Instr instr = instr_at(l.pos());
      if ((instr & ~Imm16Mask) == 0) {
        PrintF("value\n");
      } else {
        PrintF("%d\n", instr);
      }
      next(&l);
    }
  } else {
    PrintF("label in inconsistent state (pos = %d)\n", L->pos_);
  }
}


void Assembler::bind_to(Label* L, int pos) {
  ASSERT(0 <= pos && pos <= pc_offset());  // must have a valid binding position
  while (L->is_linked()) {
    int32_t fixup_pos = L->pos();
    next(L);  // call next before overwriting link with target at fixup_pos
    target_at_put(fixup_pos, pos);
  }
  L->bind_to(pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pos > last_bound_pos_)
    last_bound_pos_ = pos;
}


void Assembler::link_to(Label* L, Label* appendix) {
  if (appendix->is_linked()) {
    if (L->is_linked()) {
      // append appendix to L's list
      int fixup_pos;
      int link = L->pos();
      do {
        fixup_pos = link;
        link = target_at(fixup_pos);
      } while (link > 0);
      ASSERT(link == kEndOfChain);
      target_at_put(fixup_pos, appendix->pos());
    } else {
      // L is empty, simply use appendix
      *L = *appendix;
    }
  }
  appendix->Unuse();  // appendix should not be used anymore
}


void Assembler::bind(Label* L) {
  ASSERT(!L->is_bound());  // label can only be bound once
  bind_to(L, pc_offset());
}


void Assembler::next(Label* L) {
  ASSERT(L->is_linked());
  int link = target_at(L->pos());
  if (link > 0) {
    L->link_to(link);
  } else {
    ASSERT(link == kEndOfChain);
    L->Unuse();
  }
}



// We have to use the temporary register for things that can be relocated even
// if they can be encoded in the MIPS's 16 bits of immediate-offset instruction
// space.  There is no guarantee that the relocated location can be similarly
// encoded.
static bool MustUse_at(RelocInfo::Mode rmode) {
  if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
    return Serializer::enabled();
  } else if (rmode == RelocInfo::NONE) {
    return false;
  }
  return true;
}


void Assembler::instrmod1(  Opcode opcode,
                            Register r1,
                            Register r2,
                            Register r3,
                            uint16_t sa,
                            SecondaryField func) {
  ASSERT(is_uint5(sa));
  Instr instr = opcode | (r1.code()  << 21) | (r2.code()  << 16)
                       | (r3.code()  << 11) | (sa << 6) | func;
  emit(instr);
}
void Assembler::instrmod1(  Opcode opcode,
                            SecondaryField fmt,
                            CRegister ft,
                            CRegister fs,
                            CRegister fd,
                            SecondaryField func) {
  Instr instr = opcode | fmt | (ft.code()  << 16) | (fs.code()  << 11)
                             | (fd.code() << 6)   | func;
  emit(instr);
}
void Assembler::instrmod1(  Opcode opcode,
                            SecondaryField fmt,
                            Register rt,
                            CRegister fs,
                            CRegister fd,
                            SecondaryField func) {
  Instr instr = opcode | fmt | (rt.code()  << 16) | (fs.code()  << 11)
                             | (fd.code() << 6)   | func;
  emit(instr);
}

// Instructions with immediate value
// Registers are in the order of the instruction encoding, from left to right.
void Assembler::instrmod2(  Opcode opcode,
                            Register rs,
                            Register rt,
                            int16_t  j) {
  Instr instr = opcode | (rs.code()  << rs_o)
                       | (rt.code() << rt_o) | (j & Imm16Mask);
  emit(instr);
}

void Assembler::instrmod2(  Opcode opcode,
                            Register rs,
                            Register rt,
                            uint16_t  j) {
  Instr instr = opcode | (rs.code()  << rs_o)
                       | (rt.code() << rt_o) | (j & Imm16Mask);
  emit(instr);
}
void Assembler::instrmod2(  Opcode opcode,
                            Register rs,
                            SecondaryField SF,
                            int16_t  j) {
  Instr instr = opcode | (rs.code()  << rs_o) | SF | (j & Imm16Mask);
  emit(instr);
}
void Assembler::instrmod2(  Opcode opcode,
                            Register rs,
                            SecondaryField SF,
                            uint16_t  j) {
  Instr instr = opcode | (rs.code()  << rs_o) | SF | (j & Imm16Mask);
  emit(instr);
}

void Assembler::instrmod2(  Opcode opcode,
                            Register rs,
                            CRegister rt,
                            int16_t  j) {
  Instr instr = opcode | (rs.code()  << rs_o)
                       | (rt.code() << rt_o) | (j & Imm16Mask);
  emit(instr);
}

// Registers are in the order of the instruction encoding, from left to right.
void Assembler::instrmod3(  Opcode opcode,
                            Register r1,
                            uint32_t address) {
  ASSERT(is_uint26(address));
  Instr instr = opcode | r1.code() <<21 | address;
  emit(instr);

}


int32_t Assembler::branch_offset(Label* L, bool jump_elimination_allowed) {
  int32_t target_pos;
  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link
    } else {
      target_pos = kEndOfChain;
    }
    L->link_to(pc_offset());
  }

  int32_t offset = target_pos - (pc_offset() + kBranchPCOffset);
  return offset;
}


void Assembler::label_at_put(Label* L, int at_offset) {
  int target_pos;
  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link
    } else {
      target_pos = kEndOfChain;
    }
    L->link_to(at_offset);
    instr_at_put(at_offset, target_pos + (Code::kHeaderSize - kHeapObjectTag));
  }
}



//------- Branch and jump instructions --------

// Emulated condtional branches do not emit a nop in the branch delay slot.

// Trashes the at register if no scratch register is provided.
void Assembler::bcond(Condition cond, int16_t offset, Register rs,
                      const Operand& rt, Register scratch) {
  Register r2;
  if(rt.is_reg()) {
    // We don't want any other register but scratch clobbered.
    ASSERT(!scratch.is(rs) && !scratch.is(rt.rm_));
    r2 = rt.rm_;
  } else {
    // We don't want any other register but scratch clobbered.
    ASSERT(!scratch.is(rs));
    r2 = scratch;
    li(r2, rt);
  }

  switch(cond) {
    case cc_always:
      b(offset);
      break;
    case eq:
      beq(rs, r2, offset);
      break;
    case ne:
      bne(rs, r2, offset);
      break;
    
      // Signed comparison
    case greater:
      slt(scratch, r2, rs);
      bne(scratch, zero_reg, offset);
      break;
    case greater_equal:
      slt(scratch, rs, r2);
      beq(scratch, zero_reg, offset);
      break;
    case less:
      slt(scratch, rs, r2);
      bne(scratch, zero_reg, offset);
      break;
    case less_equal:
      slt(scratch, r2, rs);
      beq(scratch, zero_reg, offset);
      break;
    
      // Unsigned comparison.
    case Ugreater:
      sltu(scratch, r2, rs);
      bne(scratch, zero_reg, offset);
      break;
    case Ugreater_equal:
      sltu(scratch, rs, r2);
      beq(scratch, zero_reg, offset);
      break;
    case Uless:
      sltu(scratch, rs, r2);
      bne(scratch, zero_reg, offset);
      break;
    case Uless_equal:
      sltu(scratch, r2, rs);
      beq(scratch, zero_reg, offset);
      break;

    default:
      UNREACHABLE();
  }
}
void Assembler::bcond(Condition cond,  Label* L, Register rs,
                      const Operand& rt, Register scratch) {
  Register r2;
  if(rt.is_reg()) {
    r2 = rt.rm_;
  } else {
    r2 = scratch;
    li(r2, rt);
  }

  // We use branch_offset as an argument for the branch instructions to be sure
  // it is called just before generating the branch instruction, as needed.

  switch(cond) {
    case cc_always:
      b(branch_offset(L, false)>>2);
      break;
    case eq:
      beq(rs, r2, branch_offset(L, false)>>2);
      break;
    case ne:
      bne(rs, r2, branch_offset(L, false)>>2);
      break;
      
      // Signed comparison
    case greater:
      slt(scratch, r2, rs);
      bne(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case greater_equal:
      slt(scratch, rs, r2);
      beq(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case less:
      slt(scratch, rs, r2);
      bne(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case less_equal:
      slt(scratch, r2, rs);
      beq(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    
      // Unsigned comparison.
    case Ugreater:
      sltu(scratch, r2, rs);
      bne(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case Ugreater_equal:
      sltu(scratch, rs, r2);
      beq(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case Uless:
      sltu(scratch, rs, r2);
      bne(scratch, zero_reg, branch_offset(L, false)>>2);
      break;
    case Uless_equal:
      sltu(scratch, r2, rs);
      beq(scratch, zero_reg, branch_offset(L, false)>>2);
      break;

    default:
      UNREACHABLE();
  }
}

// Trashes the at register if no scratch register is provided.
// We need to use a bgezal or bltzal, but they can't be used directly with the
// slt instructions. We could use sub or add instead but we would miss overflow
// cases, so we keep slt and add an intermediate third instruction.
void Assembler::blcond(Condition cond, int16_t offset, Register rs,
                        const Operand& rt, Register scratch) {
  Register r2;
  if(rt.is_reg()) {
    r2 = rt.rm_;
  } else {
    r2 = scratch;
    li(r2, rt);
  }

  switch(cond) {
    case cc_always:
      bal(offset);
      break;
    case eq:
      bne(rs, r2, 2);
      nop();
      bal(offset);
      break;
    case ne:
      beq(rs, r2, 2);
      nop();
      bal(offset);
      break;
    
      // Signed comparison
    case greater:
      slt(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, offset);
      break;
    case greater_equal:
      slt(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, offset);
      break;
    case less:
      slt(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, offset);
      break;
    case less_equal:
      slt(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, offset);
      break;
    
      // Unsigned comparison.
    case Ugreater:
      sltu(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, offset);
      break;
    case Ugreater_equal:
      sltu(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, offset);
      break;
    case Uless:
      sltu(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, offset);
      break;
    case Uless_equal:
      sltu(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, offset);
      break;

    default:
      UNREACHABLE();
  }
}

void Assembler::blcond(Condition cond, Label* L, Register rs,
    const Operand& rt, Register scratch) {
  Register r2;
  if(rt.is_reg()) {
    r2 = rt.rm_;
  } else {
    r2 = scratch;
    li(r2, rt);
  }

  switch(cond) {
    case cc_always:
      bal(branch_offset(L, false)>>2);
      break;
    case eq:
      bne(rs, r2, 2);
      nop();
      bal(branch_offset(L, false)>>2);
      break;
    case ne:
      beq(rs, r2, 2);
      nop();
      bal(branch_offset(L, false)>>2);
      break;
    
      // Signed comparison
    case greater:
      slt(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, branch_offset(L, false)>>2);
      break;
    case greater_equal:
      slt(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, branch_offset(L, false)>>2);
      break;
    case less:
      slt(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, branch_offset(L, false)>>2);
      break;
    case less_equal:
      slt(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, branch_offset(L, false)>>2);
      break;
    
      // Unsigned comparison.
    case Ugreater:
      sltu(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, branch_offset(L, false)>>2);
      break;
    case Ugreater_equal:
      sltu(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, branch_offset(L, false)>>2);
      break;
    case Uless:
      sltu(scratch, rs, r2);
      addiu(scratch, scratch, Operand(-1));
      bgezal(scratch, branch_offset(L, false)>>2);
      break;
    case Uless_equal:
      sltu(scratch, r2, rs);
      addiu(scratch, scratch, Operand(-1));
      bltzal(scratch, branch_offset(L, false)>>2);
      break;

    default:
      UNREACHABLE();
  }
}

void Assembler::b(int16_t offset) {
  beq(zero_reg, zero_reg, offset);
}

void Assembler::bal(int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  bgezal(zero_reg, offset);
}

void Assembler::beq(Register rs, Register rt, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(BEQ, rs, rt, offset);
}

void Assembler::bgez(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(REGIMM, rs, BGEZ, offset);
}

void Assembler::bgezal(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(REGIMM, rs, BGEZAL, offset);
}

void Assembler::bgtz(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(BGTZ, rs, zero_reg, offset);
}

void Assembler::blez(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(BLEZ, rs, zero_reg, offset);
}

void Assembler::bltz(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(REGIMM, rs, BLTZ, offset);
}

void Assembler::bltzal(Register rs, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(REGIMM, rs, BLTZAL, offset);
}

//void Assembler::beql(Register rs, Register rt, int16_t offset) {
//  instrmod2(BEQL, rs, rt, offset);
//}

//void Assembler::blezl(Register rs, int16_t offset) {
//  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
//  instrmod2(BLEZL, rs, zero_reg, offset);
//}

//void Assembler::bgtzl(Register rs, int16_t offset) {
//  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
//  instrmod2(BGTZL, rs, zero_reg, offset);
//}

void Assembler::bne(Register rs, Register rt, int16_t offset) {
  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
  instrmod2(BNE, rs, rt, offset);
}

//void Assembler::bnel(Register rs, Register rt, int16_t offset) {
//  ASSERT(is_int16(offset)); // We check the offset can be used in a branch.
//  instrmod2(BNEL, rs, rt, offset);
//}


void Assembler::j(const Operand& rt) {
  ASSERT(!rt.is_reg() && is_uint26(rt.imm32_));
  if(!MustUse_at(rt.rmode_)) {
  emit(J | rt.imm32_);
  } else {
    li(at, rt);
    jr(Operand(at));
  }
}

void Assembler::jr(Register rs) {
  instrmod1(SPECIAL, rs, zero_reg, zero_reg, 0, JR);
}
void Assembler::jr(const Operand& rs) {
  ASSERT(rs.is_reg());
  jr(rs.rm());
}

void Assembler::jal(const Operand& rt) {
  ASSERT(!rt.is_reg() && is_uint26(rt.imm32_));
  if(!MustUse_at(rt.rmode_)) {
    emit(JAL | rt.imm32_);
  } else {
    li(at, rt);
    jalr(Operand(at));
  }
}

void Assembler::jalr(Register rs, Register rd) {
  instrmod1(SPECIAL, rs, zero_reg, rd, 0, JALR);
}
void Assembler::jalr(const Operand& rs, Register rd) {
  ASSERT(rs.is_reg());
  jalr(rs.rm(), rd);
}

void Assembler::jcond(const Operand& target,
                      Condition cond, Register rs, const Operand& rt) {
  if(target.is_reg()) {
    if(cond == cc_always) {
      jr(target);
    } else {
      bcond(NegateCondition(cond), 2, rs, rt);
      nop();
      jr(target);
    }
  } else {  // !target.is_reg()
    if(cond == cc_always) {
      j(target);
    } else {
      if(!MustUse_at(rt.rmode_)) {
        bcond(NegateCondition(cond), 2, rs, rt);
        nop();
        j(target);  // will generate only one instruction.
      } else {
        bcond(NegateCondition(cond), 4, rs, rt);
        nop();
        j(target);  // will generate exactly 3 instructions.
      }
    }
  }
}
void Assembler::jalcond(const Operand& target,
                        Condition cond, Register rs, const Operand& rt) {
  if(target.is_reg()) {
    if(cond == cc_always) {
      jalr(target);
    } else {
      bcond(NegateCondition(cond), 2, rs, rt);
      nop();
      jalr(target);
    }
  } else {  // !target.is_reg()
    if(cond == cc_always) {
      jal(target);
    } else {
      if(!MustUse_at(rt.rmode_)) {
        bcond(NegateCondition(cond), 2, rs, rt);
        nop();
        jal(target);  // will generate only one instruction.
      } else {
        bcond(NegateCondition(cond), 4, rs, rt);
        nop();
        jal(target);  // will generate exactly 3 instructions.
      }
    }
  }
}


//-------Data-processing-instructions---------

// Arithmetic

void Assembler::add(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    addi(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, ADD);
  }
}
void Assembler::addi(Register rd, Register rs, const Operand& rt) {
  if(is_int16(rt.imm32_)) {
    instrmod2(ADDI, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    add(rd, rs, at);
  }
}

void Assembler::addu(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    addiu(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, ADDU);
  }
}
void Assembler::addiu(Register rd, Register rs, const Operand& rt) {
  if(is_int16(rt.imm32_)) {
    instrmod2(ADDIU, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    addu(rd, rs, at);
  }
}

void Assembler::sub(Register rd, Register rs, const Operand& rt) {
  ASSERT(rt.is_reg());
  instrmod1(SPECIAL, rs, rt.rm(), rd, 0, SUB);
}
void Assembler::subu(Register rd, Register rs, const Operand& rt) {
  ASSERT(rt.is_reg());
  instrmod1(SPECIAL, rs, rt.rm(), rd, 0, SUBU);
}

void Assembler::mult(Register rs, const Operand& rt) {
  if(!rt.is_reg()) {
    li(at, Operand(rt));
    instrmod1(SPECIAL, rs, at, zero_reg, 0, MULT);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), zero_reg, 0, MULT);
  }
}
void Assembler::multu(Register rs, const Operand& rt) {
  if(!rt.is_reg()) {
    li(at, Operand(rt));
    instrmod1(SPECIAL, rs, at, zero_reg, 0, MULTU);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), zero_reg, 0, MULTU);
  }
}

void Assembler::div(Register rs, const Operand& rt) {
  if(!rt.is_reg()) {
    li(at, Operand(rt));
    instrmod1(SPECIAL, rs, at, zero_reg, 0, DIV);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), zero_reg, 0, DIV);
  }
}
void Assembler::divu(Register rs, const Operand& rt) {
  if(!rt.is_reg()) {
    li(at, Operand(rt));
    instrmod1(SPECIAL, rs, at, zero_reg, 0, DIVU);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), zero_reg, 0, DIVU);
  }
}

void Assembler::mul(Register rd, Register rs, const Operand& rt) {
  if(!rt.is_reg()) {
    li(at, Operand(rt));
    instrmod1(SPECIAL2, rs, at, rd, 0, MUL);
  } else {
    instrmod1(SPECIAL2, rs, rt.rm(), rd, 0, MUL);
  }
}

// Logical

void Assembler::and_(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    andi(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, AND);
  }
}
void Assembler::andi(Register rd, Register rs, const Operand& rt) {
  if(is_uint16(rt.imm32_)) {
    instrmod2(ANDI, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    and_(rd, rs, Operand(at));
  }
}

void Assembler::or_(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    ori(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, OR);
  }
}
void Assembler::ori(Register rt, Register rs, const Operand& j) {
  if(is_uint16(j.imm32_)) {
    instrmod2(ORI, rs, rt,  (int16_t) j.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, j);
    or_(rt, rs, Operand(at));
  }
}

void Assembler::xor_(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    xori(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, XOR);
  }
}
void Assembler::xori(Register rd, Register rs, const Operand& rt) {
  if(is_uint16(rt.imm32_)) {
    instrmod2(XORI, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    xor_(rd, rs, Operand(at));
  }
}

void Assembler::nor(Register rd, Register rs, const Operand& rt) {
  instrmod1(SPECIAL, rs, rt.rm(), rd, 0, NOR);
}

// Shifts
void Assembler::sll(Register rd, Register rt, uint16_t sa) {
  instrmod1(SPECIAL, zero_reg, rt, rd, sa, SLL);
}
void Assembler::sllv(Register rd, Register rt, Register rs) {
  instrmod1(SPECIAL, rs, rt, rd, 0, SLLV);
}

void Assembler::srl(Register rd, Register rt, uint16_t sa) {
  instrmod1(SPECIAL, zero_reg, rt, rd, sa, SRL);
}
void Assembler::srlv(Register rd, Register rt, Register rs) {
  instrmod1(SPECIAL, rs, rt, rd, 0, SRLV);
}

void Assembler::sra(Register rd, Register rt, uint16_t sa) {
  instrmod1(SPECIAL, zero_reg, rt, rd, sa, SRA);
}
void Assembler::srav(Register rd, Register rt, Register rs) {
  instrmod1(SPECIAL, rs, rt, rd, 0, SRAV);
}


//------------Memory-instructions-------------

void Assembler::lb(Register rd, const MemOperand& rs) {
  instrmod2(LB, rs.rm(), rd, rs.offset_);
}

void Assembler::lbu(Register rd, const MemOperand& rs) {
  instrmod2(LBU, rs.rm(), rd, rs.offset_);
}

void Assembler::lw(Register rd, const MemOperand& rs) {
  instrmod2(LW, rs.rm(), rd, rs.offset_);
}

void Assembler::sb(Register rd, const MemOperand& rs) {
  instrmod2(SB, rs.rm(), rd, rs.offset_);
}

void Assembler::sw(Register rd, const MemOperand& rs) {
  instrmod2(SW, rs.rm(), rd, rs.offset_);
}

void Assembler::lui(Register rd, uint16_t j) {
  instrmod2(LUI, zero_reg, rd, j);
}

//-------------Misc-instructions--------------

// Break / Trap instructions
void Assembler::break_(uint32_t code) {
  ASSERT( (code & ~0xfffff) == 0);
  Instr break_instr = SPECIAL | BREAK | (code<<6);
  emit(break_instr);
}

void Assembler::tge(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TGE | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}

void Assembler::tgeu(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TGEU | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}

void Assembler::tlt(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TLT | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}

void Assembler::tltu(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TLTU | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}


void Assembler::teq(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TEQ | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}

void Assembler::tne(Register rs, Register rt, uint16_t code) {
  ASSERT(is_uint10(code));
  Instr instr = SPECIAL | TNE | rs.code()<<21 | rt.code()<<16 | code<<6;
  emit(instr);
}

// Move from HI/LO register

void Assembler::mfhi(Register rd) {
  instrmod1(SPECIAL, zero_reg, zero_reg, rd, 0, MFHI);
}
void Assembler::mflo(Register rd) {
  instrmod1(SPECIAL, zero_reg, zero_reg, rd, 0, MFLO);
}

// Set on less than instructions
void Assembler::slt(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    slti(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, SLT);
  }
}

void Assembler::sltu(Register rd, Register rs, const Operand& rt) {
  if (!rt.is_reg()) {
    sltiu(rd, rs, rt);
  } else {
    instrmod1(SPECIAL, rs, rt.rm(), rd, 0, SLTU);
  }
}

void Assembler::slti(Register rd, Register rs, const Operand& rt) {
  ASSERT(!rt.is_reg());
  if(is_uint16(rt.imm32_)) {
    instrmod2(SLTI, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    slt(rd, rs, Operand(at));
  }
}

void Assembler::sltiu(Register rd, Register rs, const Operand& rt) {
  ASSERT(!rt.is_reg());
  if(is_uint16(rt.imm32_)) {
    instrmod2(SLTIU, rs, rd,  (int16_t) rt.imm32_);
  } else {
    CHECK(!rs.is(at));
    li(at, rt);
    sltu(rd, rs, Operand(at));
  }
}


//--------Coprocessor-instructions----------------

  // Load, store, move
  void Assembler::lwc1(CRegister fd, const MemOperand& src) {
    instrmod2(LWC1, src.rm(), fd, src.offset_ );
  }

  void Assembler::ldc1(CRegister fd, const MemOperand& src) {
    // TODO(MIPS.6)
    // MIPS architecture expect doubles to be 8-bytes aligned. However v8 has
    // currently everything aligned to 4 bytes.
    // The current solution is to use intermediate general purpose registers
    // when the value is not 8-byte aligned.

    // Load the address int at.
    addiu(at, src.rm(), Operand(src.offset_));
    andi(at, at, Operand(7));
    beq(at, zero_reg, 7);
    nop();

    // Address is not 8-byte aligned. Load manually.
    // mtc1 must be executed first. (see MIPS32 ISA)

    // Load first half of the double
    lw(at, MemOperand(src.rm(), src.offset_ + kPointerSize));
    mtc1(fd, at);
    // Load second half of the double
    lw(at, MemOperand(src.rm(), src.offset_));
    mthc1(fd, at);
    b(2);
    nop();

    // Address is 8-byte aligned.
    instrmod2(LDC1, src.rm(), fd, src.offset_ );
  }

  void Assembler::swc1(CRegister fd, const MemOperand& src) {
    instrmod2(SWC1, src.rm(), fd, src.offset_ );
  }

  void Assembler::sdc1(CRegister fd, const MemOperand& src) {
    // TODO(MIPS.6)
    // MIPS architecture expect doubles to be 8-bytes aligned. However v8 has
    // currently everything aligned to 4 bytes.
    // The current solution is to use intermediate general purpose registers
    // when the value is not 8-byte aligned.

    // Load the address int at.
    addiu(at, src.rm(), Operand(src.offset_));
    andi(at, at, Operand(7));
    beq(at, zero_reg, 7);
    //nop(); The following mfhc1 is harmless.

    // Address is not 8-byte aligned. Load manually
    // Load first half of the double
    mfhc1(fd, at);
    sw(at, MemOperand(src.rm(), src.offset_));
    // Load second half of the double
    mfc1(fd, at);
    sw(at, MemOperand(src.rm(), src.offset_ + kPointerSize));
    b(2);
    nop();

    // Address is 8-byte aligned.
    instrmod2(SDC1, src.rm(), fd, src.offset_ );
  }

  void Assembler::mtc1(CRegister fs, Register rt) {
    instrmod1(COP1, MTC1, rt, fs, f0 );
  }

  void Assembler::mthc1(CRegister fs, Register rt) {
    instrmod1(COP1, MTHC1, rt, fs, f0 );
  }

  void Assembler::mfc1(CRegister fs, Register rt) {
    instrmod1(COP1, MFC1, rt, fs, f0 );
  }

  void Assembler::mfhc1(CRegister fs, Register rt) {
    instrmod1(COP1, MFHC1, rt, fs, f0 );
  }

  // Conversions

  void Assembler::cvt_w_s(CRegister fd, CRegister fs) {
    instrmod1(COP1, S, f0, fs, fd, CVT_W_S);
  }

  void Assembler::cvt_w_d(CRegister fd, CRegister fs) {
    instrmod1(COP1, D, f0, fs, fd, CVT_W_D);
  }

  void Assembler::cvt_l_s(CRegister fd, CRegister fs) {
    instrmod1(COP1, S, f0, fs, fd, CVT_L_S);
  }

  void Assembler::cvt_l_d(CRegister fd, CRegister fs) {
    instrmod1(COP1, D, f0, fs, fd, CVT_L_D);
  }

  void Assembler::cvt_s_w(CRegister fd, CRegister fs) {
    instrmod1(COP1, W, f0, fs, fd, CVT_S_W);
  }

  void Assembler::cvt_s_l(CRegister fd, CRegister fs) {
    instrmod1(COP1, L, f0, fs, fd, CVT_S_L);
  }

  void Assembler::cvt_s_d(CRegister fd, CRegister fs) {
    instrmod1(COP1, D, f0, fs, fd, CVT_S_D);
  }

  void Assembler::cvt_d_w(CRegister fd, CRegister fs) {
    instrmod1(COP1, W, f0, fs, fd, CVT_D_W);
  }

  void Assembler::cvt_d_l(CRegister fd, CRegister fs) {
    instrmod1(COP1, L, f0, fs, fd, CVT_D_L);
  }

  void Assembler::cvt_d_s(CRegister fd, CRegister fs) {
    instrmod1(COP1, S, f0, fs, fd, CVT_D_S);
  }

  // Conditions
  void Assembler::c(C_Condition cond, SecondaryField fmt,
      CRegister ft, CRegister fs, uint16_t cc) {
    ASSERT(is_uint3(cc));
    ASSERT((fmt & ~(31<<21)) == 0);
    Instr instr = COP1 | fmt | ft.code()<<16 | fs.code()<<11 | cc<<8 | 3<<4 | cond;
    emit(instr);
  }

  void Assembler::bc1f(int16_t offset, uint16_t cc) {
    ASSERT(is_uint3(cc));
    Instr instr = COP1 | BC1 | cc<<18 | 0<<16 | (offset & Imm16Mask);
    emit(instr);
  }

  void Assembler::bc1t(int16_t offset, uint16_t cc) {
    ASSERT(is_uint3(cc));
    Instr instr = COP1 | BC1 | cc<<18 | 1<<16 | (offset & Imm16Mask);
    emit(instr);
  }

//------------Pseudo-instructions-------------

void Assembler::movn(Register rd, Register rt) {
  addiu(at, zero_reg, Operand(-1)); // Fill at with ones.
  xor_(rd, rt, Operand(at));
}

// load wartd in a register
void Assembler::li(Register rd, Operand j, bool gen2instr) {
  ASSERT(!j.is_reg());


  if(!MustUse_at(j.rmode_) && !gen2instr) {
    // Normal load of an immediate value which does not need Relocation Info.
    if(is_int16(j.imm32_))
      addiu(rd, zero_reg, j);
    else if(!(j.imm32_ & HIMask)) 
      ori(rd, zero_reg, j);
    else if(!(j.imm32_ & LOMask))
      lui(rd, (HIMask & j.imm32_)>>16);
    else {
      lui(rd, (HIMask & j.imm32_)>>16);
      ori(rd, rd, (LOMask & j.imm32_));
    }
  } else if ( MustUse_at(j.rmode_) ) {
    // We need Relocation Information here.
    RecordRelocInfo(j.rmode_, j.imm32_);
    // We need always the same number of instructions as we may need to patch
    // this code to load another value which may need 2 instructions to load.
    if(is_int16(j.imm32_)){
      nop();
      addiu(rd, zero_reg, j);
    } else if(!(j.imm32_ & HIMask)) {
      nop();
      ori(rd, zero_reg, j);
    } else if(!(j.imm32_ & LOMask)) {
      nop();
      lui(rd, (HIMask & j.imm32_)>>16);
    } else {
      lui(rd, (HIMask & j.imm32_)>>16);
      ori(rd, rd, (LOMask & j.imm32_));
    }
  } else if ( gen2instr ) {
    // We need always the same number of instructions as we may need to patch
    // this code to load another value which may need 2 instructions to load.
    if(is_int16(j.imm32_)){
      nop();
      addiu(rd, zero_reg, j);
    } else if(!(j.imm32_ & HIMask)) {
      nop();
      ori(rd, zero_reg, j);
    } else if(!(j.imm32_ & LOMask)) {
      nop();
      lui(rd, (HIMask & j.imm32_)>>16);
    } else {
      lui(rd, (HIMask & j.imm32_)>>16);
      ori(rd, rd, (LOMask & j.imm32_));
    }
  }
}


void Assembler::multi_push(RegList regs) {
  int16_t NumSaved = 0;
  int16_t NumToPush = NumberOfBitsSet(regs);
  
  addiu(sp, sp, Operand(-4*NumToPush));
  for (int16_t i = 0; i< kNumRegisters; i++) {
    if((regs & (1<<i)) != 0 ) {
      sw(ToRegister(i),
          MemOperand(sp, 4*(NumToPush - ++NumSaved)));
    }   
  }
}
void Assembler::multi_push_reversed(RegList regs) {
  int16_t NumSaved = 0;
  int16_t NumToPush = NumberOfBitsSet(regs);
  
  addiu(sp, sp, Operand(-4*NumToPush));
  for (int16_t i = kNumRegisters; --i>=0;) {
    if((regs & (1<<i)) != 0 ) {
      sw(ToRegister(i),
          MemOperand(sp, 4*(NumToPush - ++NumSaved)));
    }   
  }
}

void Assembler::multi_pop(RegList regs) {
  int16_t NumSaved = 0;

  for (int16_t i = kNumRegisters; --i>=0;) {
    if((regs & (1<<i)) != 0 ) {
      lw(ToRegister(i), MemOperand(sp, 4*(NumSaved++)));
    }   
  }
  addiu(sp, sp, Operand(4*NumSaved));
}
void Assembler::multi_pop_reversed(RegList regs) {
  int16_t NumSaved = 0;

  for (int16_t i = 0; i< kNumRegisters; i++) {
    if((regs & (1<<i)) != 0 ) {
      lw(ToRegister(i), MemOperand(sp, 4*(NumSaved++)));
    }   
  }
  addiu(sp, sp, Operand(4*NumSaved));
}

// Exception-generating instructions and debugging support
void Assembler::stop(const char* msg) {
  // TO_UPGRADE: Just a break for now. Maybe we could upgrade it.
  // We use the 0x54321 value to be able to find it easily when reading memory.
  break_(0x54321);
}




// Debugging
void Assembler::RecordJSReturn() {
  WriteRecordedPositions();
  CheckBuffer();
  RecordRelocInfo(RelocInfo::JS_RETURN);
}


void Assembler::RecordComment(const char* msg) {
  if (FLAG_debug_code) {
    CheckBuffer();
    RecordRelocInfo(RelocInfo::COMMENT, reinterpret_cast<intptr_t>(msg));
  }
}


void Assembler::RecordPosition(int pos) {
  if (pos == RelocInfo::kNoPosition) return;
  ASSERT(pos >= 0);
  current_position_ = pos;
}


void Assembler::RecordStatementPosition(int pos) {
  if (pos == RelocInfo::kNoPosition) return;
  ASSERT(pos >= 0);
  current_statement_position_ = pos;
}


void Assembler::WriteRecordedPositions() {
  // Write the statement position if it is different from what was written last
  // time.
  if (current_statement_position_ != written_statement_position_) {
    CheckBuffer();
    RecordRelocInfo(RelocInfo::STATEMENT_POSITION, current_statement_position_);
    written_statement_position_ = current_statement_position_;
  }

  // Write the position if it is different from what was written last time and
  // also different from the written statement position.
  if (current_position_ != written_position_ &&
      current_position_ != written_statement_position_) {
    CheckBuffer();
    RecordRelocInfo(RelocInfo::POSITION, current_position_);
    written_position_ = current_position_;
  }
}


void Assembler::GrowBuffer() {
  if (!own_buffer_) FATAL("external code buffer is too small");

  // compute new buffer size
  CodeDesc desc;  // the new buffer
  if (buffer_size_ < 4*KB) {
    desc.buffer_size = 4*KB;
  } else if (buffer_size_ < 1*MB) {
    desc.buffer_size = 2*buffer_size_;
  } else {
    desc.buffer_size = buffer_size_ + 1*MB;
  }
  CHECK_GT(desc.buffer_size, 0);  // no overflow

  // setup new buffer
  desc.buffer = NewArray<byte>(desc.buffer_size);

  desc.instr_size = pc_offset();
  desc.reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();

  // copy the data
  int pc_delta = desc.buffer - buffer_;
  int rc_delta = (desc.buffer + desc.buffer_size) - (buffer_ + buffer_size_);
  memmove(desc.buffer, buffer_, desc.instr_size);
  memmove(reloc_info_writer.pos() + rc_delta,
          reloc_info_writer.pos(), desc.reloc_size);

  // switch buffers
  DeleteArray(buffer_);
  buffer_ = desc.buffer;
  buffer_size_ = desc.buffer_size;
  pc_ += pc_delta;
  reloc_info_writer.Reposition(reloc_info_writer.pos() + rc_delta,
                               reloc_info_writer.last_pc() + pc_delta);


  // On ia32 or ARM pc relative addressing is used, and we thus need to apply a
  // shift by pc_delta. But on MIPS the target address it directly loaded, so
  // we do not need to relocate here.

  ASSERT(!overflow());
}


void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  RelocInfo rinfo(pc_, rmode, data);  // we do not try to reuse pool constants
  if (rmode >= RelocInfo::JS_RETURN && rmode <= RelocInfo::STATEMENT_POSITION) {
    // Adjust code for new modes
    ASSERT(RelocInfo::IsJSReturn(rmode)
           || RelocInfo::IsComment(rmode)
           || RelocInfo::IsPosition(rmode));
    // these modes do not need an entry in the constant pool
  }
  if (rinfo.rmode() != RelocInfo::NONE) {
    // Don't record external references unless the heap will be serialized.
    if (rmode == RelocInfo::EXTERNAL_REFERENCE &&
        !Serializer::enabled() &&
        !FLAG_debug_code) {
      return;
    }
    ASSERT(buffer_space() >= kMaxRelocSize);  // too late to grow buffer here
    reloc_info_writer.Write(&rinfo);
  }
}


Address Assembler::target_address_at(Address pc) {
//  return Memory::Address_at(target_address_address_at(pc));
  Instr instr1 = instr_at(pc);
  Instr instr2 = instr_at(pc + kInstrSize);
  // Check we have 2 instructions geneerated by li.
  ASSERT( ((instr1 & OpcodeMask)==LUI && (instr2 & OpcodeMask)==ORI) ||
          ((instr1==0) && ((instr2 & OpcodeMask)== ADDI  ||
                          (instr2 & OpcodeMask)== ORI    ||
                          (instr2 & OpcodeMask)== LUI    ))
        );
  // Interpret these 2 instructions.
  if(instr1==0) {
    if((instr2 & OpcodeMask)== ADDI) {
      return (Address)(((instr2 & Imm16Mask)<<16)>>16);
    } else if ((instr2 & OpcodeMask)== ORI) {
      return (Address)(instr2 & Imm16Mask);
    } else if ((instr2 & OpcodeMask)== LUI) {
      return(Address)((instr2 & Imm16Mask)<<16);
    }
  } else if((instr1 & OpcodeMask)==LUI && (instr2 & OpcodeMask)==ORI) {
    // 32 bits value.
    return (Address)((instr1 & Imm16Mask)<<16 | (instr2 & Imm16Mask));
  }

  // We should never get here.
  return (Address)0x0;

}


void Assembler::set_target_address_at(Address pc, Address target) {
//  Memory::Address_at(target_address_address_at(pc)) = target;
  // On MIPS we need to patch the code to generate.

  // First check we have a li
  // We use #define because using Instr would fail when building the release as
  // instr1 would be unused.
//  Instr instr1 = instr_at(pc);
//  Instr instr2 = instr_at(pc + kInstrSize);
#define instr1  instr_at(pc)
#define instr2  instr_at(pc + kInstrSize)

  // Check we have indeed the result from a li with MustUse_at true.
  ASSERT( ((instr1 & OpcodeMask)==LUI && (instr2 & OpcodeMask)==ORI) ||
          ((instr1==0) && ((instr2 & OpcodeMask)== ADDIU    ||
                          (instr2 & OpcodeMask)== ORI       ||
                          (instr2 & OpcodeMask)== LUI       ))
        );


//______________________________________________________________________________
  uint32_t rd_code = (instr2 & (31<<16));
  uint32_t* p = reinterpret_cast<uint32_t*>(pc);
  uint32_t itarget = (uint32_t)(target);

    if(is_int16(itarget)){
//      nop();
//      addiu(rd, zero_reg, j);
      *p       = 0x0;
      *(p+1)   = ADDIU | rd_code | (itarget&LOMask);
    } else if(!(itarget & HIMask)) {
//      nop();
//      ori(rd, zero_reg, j);
      *p       = 0x0;
      *(p+1)   = ORI | rd_code | (itarget&LOMask);
    } else if(!(itarget & LOMask)) {
//      nop();
//      lui(rd, (HIMask & itarget)>>16);
      *p       = 0x0;
      *(p+1)   = LUI | rd_code | ((itarget&HIMask)>>16);
    } else {
//      lui(rd, (HIMask & itarget)>>16);
//      ori(rd, rd, (LOMask & itarget));
      *p       = LUI | rd_code | ((itarget&HIMask)>>16);
      *(p+1)   = ORI | rd_code | (rd_code<<5) | (itarget&LOMask);
    }
//______________________________________________________________________________
  // Using this leads to an error in Object** HandleScope::Extend().
  // Disabling it for now.
//  static const int kLoadCodeSize = 2;
//
//  // Get the target register code.
//  // Currently all possible 2nd instructions encode the destination register at
//  // bit 16. (ori, addiu, lui)
//  uint16_t rd_code = (instr2 & 31<<16)>>16;
//
//  // Create a code patcher
//  CodePatcher patcher(pc, kLoadCodeSize);
//
//  // Add a label for checking the size of the code used for returning.
////#ifdef DEBUG
////  Label check_codesize;
////  patcher.masm()->bind(&check_codesize);
////#endif
//
//  // Patch the code.
//  patcher.masm()->li(RegisterAllocator::ToRegister(rd_code),
//                                        Operand((uint32_t)target), true);
//
//  // Check that the size of the code generated is as expected.
//  // The code seems to be patched correctly but the ASSERT fails.
//  // TODO: Have the ASSERT work or correct it.
////  ASSERT_EQ(kLoadCodeSize,
////            patcher.masm()->SizeOfCodeGeneratedSince(&check_codesize));
//______________________________________________________________________________

  CPU::FlushICache(pc, 2* sizeof(int32_t));
}


} }  // namespace v8::internal
