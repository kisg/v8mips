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

#include "bootstrapper.h"
#include "codegen-inl.h"
#include "debug.h"
#include "runtime.h"

namespace v8 {
namespace internal {

MacroAssembler::MacroAssembler(void* buffer, int size)
    : Assembler(buffer, size),
      unresolved_(0),
      generating_stub_(false),
      allow_stub_calls_(true),
      code_object_(Heap::undefined_value()) {
}



void MacroAssembler::Jump(Register target, Condition cond, Register r1, const Operand& r2) {
  jcond(Operand(target), cond, r1, r2);
}


void MacroAssembler::Jump(intptr_t target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  if(cond != cc_always) {
    // TODO: Implement Conditional Call. Need for conditional jump...
    UNIMPLEMENTED();
  }
  // TO_UPGRADE: Use a JAL instead of JALR if the target is in the pc region and
  // TO_UPGRADE: if the target does not need RelocInfo.
  // Currently 'li' handles the cases when target need to be relocated.
  li(t9, Operand(target, rmode));
  jr(Operand(t9));
  ASSERT(kCallTargetAddressOffset == 4 * kInstrSize);
}


void MacroAssembler::Jump(byte* target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  Jump(reinterpret_cast<intptr_t>(target), rmode, cond, r1, r2);
}


void MacroAssembler::Jump(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  Jump(reinterpret_cast<intptr_t>(code.location()), rmode, cond);
}


void MacroAssembler::Call(Register target,
                          Condition cond, Register r1, const Operand& r2) {
  jalcond(Operand(target), cond, r1, r2);
}


void MacroAssembler::Call(intptr_t target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  if(cond != cc_always) {
    // TODO: Implement Conditional Call. Need for conditional jump...
    UNIMPLEMENTED();
  }
  // TO_UPGRADE: Use a JAL instead of JALR if the target is in the pc region.
  // TO_UPGRADE: Use jalcond with always. (not implemented when writing this)
  // CAREFUL: Currently 'li' handles the cases when target need to be relocated.
  li(t9, Operand(target, rmode));
  jalr(Operand(t9));
  // We assume the jump is the last instruction generated. Some function use the
  // branch delay slots. (eg VirtualFrame::RawCallCodeObject)
  ASSERT(kCallTargetAddressOffset == 4 * kInstrSize);
}


void MacroAssembler::Call(byte* target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  Call(reinterpret_cast<intptr_t>(target), rmode, cond, r1, r2);
}


void MacroAssembler::Call(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  Call(reinterpret_cast<intptr_t>(code.location()), rmode, cond, r1, r2);
}


void MacroAssembler::Jump_was(Register target, Condition cond, Register r1, const Operand& r2) {
  printf("Using Jump_was. Be sure to update the stack on return.");
  jcond(Operand(target), cond, r1, r2);
  addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
}


void MacroAssembler::Jump_was(intptr_t target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  printf("Using Jump_was. Be sure to update the stack on return.");
  if(cond != cc_always) {
    // TODO: Implement Conditional Call. Need for conditional jump...
    UNIMPLEMENTED();
  }
  // TO_UPGRADE: Use a JAL instead of JALR if the target is in the pc region and
  // TO_UPGRADE: if the target does not need RelocInfo.
  // Currently 'li' handles the cases when target need to be relocated.
  li(t9, Operand(target, rmode));
  jr(Operand(t9));
  addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  ASSERT(kCallTargetAddressOffset == 4 * kInstrSize);
}


void MacroAssembler::Jump_was(byte* target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  Jump_was(reinterpret_cast<intptr_t>(target), rmode, cond, r1, r2);
}


void MacroAssembler::Jump_was(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  Jump_was(reinterpret_cast<intptr_t>(code.location()), rmode, cond);
}
// Call with arguments slots.
void MacroAssembler::Call_was(Register target,
                          Condition cond, Register r1, const Operand& r2) {

  jalcond(Operand(target), cond, r1, r2);
  // Make space for arguments slots. We use the branch delay slot.
  addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  // On return we free arguments slots. We have to care that nothing is passed
  // down on the stack.
  addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);
}


void MacroAssembler::Call_was(intptr_t target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  if(cond != cc_always) {
    // TODO: Implement Conditional Call.
    UNIMPLEMENTED();
  }
  // TO_UPGRADE: Use a JAL instead of JALR if the target is in the pc region.
  // CAREFUL: Currently 'li' handles the cases when target need to be relocated.
  li(t9, Operand(target, rmode));
  jalr(Operand(t9));
  addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);
  ASSERT(kCallTargetAddressOffset == 4 * kInstrSize);
}


void MacroAssembler::Call_was(byte* target, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  Call_was(reinterpret_cast<intptr_t>(target), rmode, cond, r1, r2);
}


void MacroAssembler::Call_was(Handle<Code> code, RelocInfo::Mode rmode,
                          Condition cond, Register r1, const Operand& r2) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  Call_was(reinterpret_cast<intptr_t>(code.location()), rmode, cond, r1, r2);
}


void MacroAssembler::Ret(Condition cond, Register r1, const Operand& r2) {
  jcond(Operand(ra), cond, r1, r2);
}


void MacroAssembler::SmiJumpTable(Register index, Vector<Label*> targets) {
  UNIMPLEMENTED();
//  // Empty the const pool.
//  CheckConstPool(true, true);
//  add(pc, pc, Operand(index,
//                      LSL,
//                      assembler::arm::Instr::kInstrSizeLog2 - kSmiTagSize));
//  BlockConstPoolBefore(pc_offset() + (targets.length() + 1) * kInstrSize);
//  nop();  // Jump table alignment.
//  for (int i = 0; i < targets.length(); i++) {
//    b(targets[i]);
//  }
}


void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index) {
  lw(destination, MemOperand(s4, index << kPointerSizeLog2));
}

void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index,
                              Condition cond, Register src1, const Operand& src2) {
  bcond( NegateCondition(cond), 2, src1, src2);
  nop();
  lw(destination, MemOperand(s4, index << kPointerSizeLog2));
}


// Will clobber 4 registers: object, offset, scratch, ip.  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object, Register offset,
                                 Register scratch) {
  // This is how much we shift the remembered set bit offset to get the
  // offset of the word in the remembered set.  We divide by kBitsPerInt (32,
  // shift right 5) and then multiply by kIntSize (4, shift left 2).
  const int kRSetWordShift = 3;

  Label fast, done;

  // First, test that the object is not in the new space.  We cannot set
  // remembered set bits in the new space.
  // object: heap object pointer (with tag)
  // offset: offset to store location from the object
//  and_(scratch, object, Operand(Heap::NewSpaceMask()));
//  cmp(scratch, Operand(ExternalReference::new_space_start()));
//  b(eq, &done);
  and_(scratch, object, Operand(Heap::NewSpaceMask()));
  bcond(eq, &done, scratch, Operand(ExternalReference::new_space_start()));
  nop();    // NOP_ADDED

  // Compute the bit offset in the remembered set.
  // object: heap object pointer (with tag)
  // offset: offset to store location from the object
//  mov(ip, Operand(Page::kPageAlignmentMask));  // load mask only once
//  and_(scratch, object, Operand(ip));  // offset into page of the object
//  add(offset, scratch, Operand(offset));  // add offset into the object
//  mov(offset, Operand(offset, LSR, kObjectAlignmentBits));
  li(ip, Operand(Page::kPageAlignmentMask));    // load mask only once
  and_(scratch, object, Operand(ip));  // offset into page of the object
  addu(offset, scratch, Operand(offset));  // add offset into the object
  srl(offset, offset, kObjectAlignmentBits);

  // Compute the page address from the heap object pointer.
  // object: heap object pointer (with tag)
  // offset: bit offset of store position in the remembered set
//  bic(object, object, Operand(ip));
  andi(object, object, Operand(~Page::kPageAlignmentMask));

  // If the bit offset lies beyond the normal remembered set range, it is in
  // the extra remembered set area of a large object.
  // object: page start
  // offset: bit offset of store position in the remembered set
//  cmp(offset, Operand(Page::kPageSize / kPointerSize));
//  b(lt, &fast);
  bcond(less, &fast, offset, Operand(Page::kPageSize / kPointerSize));
  nop(); // NOP_ADDED

  // Adjust the bit offset to be relative to the start of the extra
  // remembered set and the start address to be the address of the extra
  // remembered set.
//  sub(offset, offset, Operand(Page::kPageSize / kPointerSize));
  addiu(offset, offset, -1* Page::kPageSize / kPointerSize);
  // Load the array length into 'scratch' and multiply by four to get the
  // size in bytes of the elements.
//  ldr(scratch, MemOperand(object, Page::kObjectStartOffset
//                                  + FixedArray::kLengthOffset));
//  mov(scratch, Operand(scratch, LSL, kObjectAlignmentBits));
  lw(scratch, MemOperand(object, Page::kObjectStartOffset
                                  + FixedArray::kLengthOffset));
  sll(scratch, scratch, kObjectAlignmentBits);
  // Add the page header (including remembered set), array header, and array
  // body size to the page address.
  addiu(object, object, Page::kObjectStartOffset + FixedArray::kHeaderSize);
  addu(object, object, scratch);

  bind(&fast);
  // Get address of the rset word.
  // object: start of the remembered set (page start for the fast case)
  // offset: bit offset of store position in the remembered set
//  bic(scratch, offset, Operand(kBitsPerInt - 1));  // clear the bit offset
//  add(object, object, Operand(scratch, LSR, kRSetWordShift));
  andi(object, object, Operand(~(kBitsPerInt - 1)));
  sll(scratch, scratch, kRSetWordShift);
  addu(object, object, scratch);
  // Get bit offset in the rset word.
  // object: address of remembered set word
  // offset: bit offset of store position
//  and_(offset, offset, Operand(kBitsPerInt - 1));
  and_(offset, offset, Operand(kBitsPerInt - 1));

//  ldr(scratch, MemOperand(object));
//  mov(ip, Operand(1));
//  orr(scratch, scratch, Operand(ip, LSL, offset));
//  str(scratch, MemOperand(object));
  lw(scratch, MemOperand(object));
  li(ip, Operand(1));
  sllv(ip, ip, offset);
  or_(scratch, scratch, Operand(ip));
  sw(scratch, MemOperand(object));

  bind(&done);
}


void MacroAssembler::EnterFrame(StackFrame::Type type) {
  addiu(sp, sp, Operand(-5 * kPointerSize));
  li(t0, Operand(Smi::FromInt(type)));
  li(t1, Operand(CodeObject()));
  sw(ra, MemOperand(sp, 4 * kPointerSize));
  sw(fp, MemOperand(sp, 3 * kPointerSize));
  sw(cp, MemOperand(sp, 2 * kPointerSize));
  sw(t0, MemOperand(sp, 1 * kPointerSize));
  sw(t1, MemOperand(sp, 0 * kPointerSize));
  addiu(fp, sp, Operand(3 * kPointerSize));
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  mov(sp, fp);
  lw(fp, MemOperand(sp, 0 * kPointerSize));
  lw(ra, MemOperand(sp, 1 * kPointerSize));
  addiu(sp, Operand(2 * kPointerSize));

}


void MacroAssembler::SetupAlignedCall(Register scratch, int arg_count) {
  push(s3);                        // Save s3 on the stack
  mov(s3, sp);                     // Save sp
  
//  li(at, Operand(0x401c));
//  mov(t0, s3);
//  andi(t0, t0, Operand(0xffff));
//  teq(t0, at, 0.12);
  
  li(scratch, Operand(~7));        // Load sp mask
  and_(sp, sp, Operand(scratch));  // Align sp.

    // We are going to push (arg_count + 2)*4 on the stack. We make sure sp will
    // be 8 bytes aligned after this.
  if( (arg_count % 2) != 0) {
    addiu(sp, sp, -4);
  }
}

void MacroAssembler::ReturnFromAlignedCall() {
  mov(sp, s3);      // Restore sp.
  pop(s3);          // Restore s3
}


void MacroAssembler::EnterExitFrame(ExitFrame::Mode mode) {

  // Register use : r6->t6 r0->a0

  // Compute the argv pointer and keep it in a callee-saved register.
  // a0 is argc.
  sll(t0, a0, kPointerSizeLog2);
  add(s2, sp, t0);
  addi(s2, s2, Operand(-kPointerSize));

  // Compute callee's stack pointer before making changes and save it as
  // ip register so that it is restored as sp register on exit, thereby
  // popping the args.

  // ip = sp + kPointerSize * #args;
  add(ip, sp, t0);

  // Align the stack at this point.  After this point we have 5 pushes,
  // so in fact we have to unalign here!  See also the assert on the
  // alignment immediately below.
  AlignStack(1);

  // Save registers.
  // We save s3 as we sill need it to save sp in CEntryStub::GenerateCore.
  addiu(sp, sp, Operand(-16));
  sw(ip, MemOperand(sp, 12));
  sw(s3, MemOperand(sp,  8));
  sw(ra, MemOperand(sp,  4));
  sw(fp, MemOperand(sp,  0));
  mov(fp, sp);  // setup new frame pointer

  // Push debug marker.
  if (mode == ExitFrame::MODE_DEBUG) {
    li(ip, Operand(Smi::FromInt(0)));
  } else {
    li(ip, Operand(CodeObject()));
  }
  push(ip);

  // Save the frame pointer and the context in top.
//  mov(ip, Operand(ExternalReference(Top::k_c_entry_fp_address)));
//  str(fp, MemOperand(ip));
//  mov(ip, Operand(ExternalReference(Top::k_context_address)));
//  str(cp, MemOperand(ip));
  li(ip, Operand(ExternalReference(Top::k_c_entry_fp_address)));
  sw(fp, MemOperand(ip));
  li(ip, Operand(ExternalReference(Top::k_context_address)));   // CURRENT
  sw(cp, MemOperand(ip));

  // Setup argc and the builtin function in callee-saved registers.
  mov(s0, a0);
  mov(s1, a1);


#ifdef ENABLE_DEBUGGER_SUPPORT
  // Save the state of all registers to the stack from the memory
  // location. This is needed to allow nested break points.
//  if (mode == ExitFrame::MODE_DEBUG) {
    // Use sp as base to push.
    // ia32 seems to have a bug here. (Cf ia32 code TODO(1243899))
//    CopyRegistersFromMemoryToStack(sp, kJSCallerSaved);
//  }
#endif
}


void MacroAssembler::AlignStack(int offset) {
  int activation_frame_alignment = OS::ActivationFrameAlignment();
  if (activation_frame_alignment != kPointerSize) {
    // This code needs to be made more general if this assert doesn't hold.
    ASSERT(activation_frame_alignment == 2 * kPointerSize);
//    mov(r7, Operand(Smi::FromInt(0)));
//    tst(sp, Operand(activation_frame_alignment - offset));
//    push(r7, eq);  // Conditional push instruction.
    li(t3, Operand(Smi::FromInt(0)));
    // Register use: t0 ok ?
    andi(t0, sp,  Operand(activation_frame_alignment - 1));
    push(t3, eq, t0, zero_reg );
  }
}

void MacroAssembler::LeaveExitFrame(ExitFrame::Mode mode) {

#ifdef ENABLE_DEBUGGER_SUPPORT
  // Restore the memory copy of the registers by digging them out from
  // the stack. This is needed to allow nested break points.
  if (mode == ExitFrame::MODE_DEBUG) {
    // This code intentionally clobbers r2 and r3.
    const int kCallerSavedSize = kNumJSCallerSaved * kPointerSize;
    const int kOffset = ExitFrameConstants::kDebugMarkOffset - kCallerSavedSize;
    addi(a3, fp, Operand(kOffset));
    CopyRegistersFromStackToMemory(a3, a2, kJSCallerSaved);
  }
#endif

  // Clear top frame.
//  mov(r3, Operand(0));
//  mov(ip, Operand(ExternalReference(Top::k_c_entry_fp_address)));
//  str(r3, MemOperand(ip));
  li(a3, Operand(0));
  li(ip, Operand(ExternalReference(Top::k_c_entry_fp_address)));
  sw(a3, MemOperand(ip));

  // Restore current context from top and clear it in debug mode.
//  mov(ip, Operand(ExternalReference(Top::k_context_address)));
//  ldr(cp, MemOperand(ip));
  li(ip, Operand(ExternalReference(Top::k_context_address)));
  lw(cp, MemOperand(ip));
#ifdef DEBUG
//  str(r3, MemOperand(ip));
  sw(a3, MemOperand(ip));
#endif

  // Pop the arguments, restore registers, and return.
  mov(sp, fp);  // respect ABI stack constraint
//  ldm(ia, sp, fp.bit() | sp.bit() | pc.bit());
  lw(fp, MemOperand(sp,  0));
  lw(ra, MemOperand(sp,  4));
  lw(s3, MemOperand(sp,  8));
  lw(sp, MemOperand(sp, 12));
  jr(ra);
  nop();    // NOP_ADDED
}


void MacroAssembler::InvokePrologue(const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    InvokeFlag flag,
                                    bool withArgsSlots) {

  bool definitely_matches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  //  r0: actual arguments count
  //  r1: function (passed through to callee)
  //  r2: expected arguments count
  //  r3: callee code entry

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract if values are
  // passed in registers.
  ASSERT(actual.is_immediate() || actual.reg().is(a0));
  ASSERT(expected.is_immediate() || expected.reg().is(a2));
  ASSERT((!code_constant.is_null() && code_reg.is(no_reg)) || code_reg.is(a3));

  if (expected.is_immediate()) {
    ASSERT(actual.is_immediate());
    if (expected.immediate() == actual.immediate()) {
      definitely_matches = true;
    } else {
//      mov(r0, Operand(actual.immediate()));
      li(a0, Operand(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
//        mov(r2, Operand(expected.immediate()));
        li(a2, Operand(expected.immediate()));
      }
    }
  } else {
    if (actual.is_immediate()) {
//      cmp(expected.reg(), Operand(actual.immediate()));
//      b(eq, &regular_invoke);
//      mov(r0, Operand(actual.immediate()));
      bcond(eq, &regular_invoke, expected.reg(), Operand(actual.immediate()));
      nop(); // NOP_ADDED
      li(a0, Operand(actual.immediate()));
    } else {
//      cmp(expected.reg(), Operand(actual.reg()));
//      b(eq, &regular_invoke);
      bcond(eq, &regular_invoke, expected.reg(), Operand(actual.reg()));
      nop();    // NOP_ADDED
    }
  }

  if (!definitely_matches) {
    if (!code_constant.is_null()) {
//      mov(r3, Operand(code_constant));
//      add(r3, r3, Operand(Code::kHeaderSize - kHeapObjectTag));
      li(a3, Operand(code_constant));
      addiu(a3, a3, Operand(Code::kHeaderSize - kHeapObjectTag));
    }
//
    Handle<Code> adaptor =
        Handle<Code>(Builtins::builtin(Builtins::ArgumentsAdaptorTrampoline));
    // We use v1 to tell the adaptor if we need args slots.
    if(withArgsSlots) {
      li(v1, Operand(0));
    } else {
      li(v1, Operand(1));
    }
    if (flag == CALL_FUNCTION) {
      Call(adaptor, RelocInfo::CODE_TARGET);
      nop(); // NOP_ADDED
      b(done);
      nop();    // NOP_ADDED
    } else {
      Jump(adaptor, RelocInfo::CODE_TARGET);
      nop();  // NOP_ADDED
    }
    bind(&regular_invoke);
  }
}

void MacroAssembler::InvokeCode(Register code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                bool withArgsSlots) {
//  break_(0x5);

  Label done;

  InvokePrologue(expected, actual, Handle<Code>::null(), code,
                      &done, flag, withArgsSlots);
  nop(); // NOP_ADDED
  if (flag == CALL_FUNCTION) {
    Call(code);
  } else {
    ASSERT(flag == JUMP_FUNCTION);
    Jump(code);
  }

  // Because arguments slots may be needed and we need to ignore them in the
  // other case we allocate them here.
  if(withArgsSlots) {
//    addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  } else {
    nop();
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeCode(Handle<Code> code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                RelocInfo::Mode rmode,
                                InvokeFlag flag,
                                bool withArgsSlots) {
  Label done;

  InvokePrologue(expected, actual, code, no_reg,
                      &done, flag, withArgsSlots);
  nop(); // NOP_ADDED
  if (flag == CALL_FUNCTION) {
    Call(code, rmode);
  } else {
    Jump(code, rmode);
  }

  // Because arguments slots may be needed and we need to ignore them in the
  // other case we allocate them here.
  if(withArgsSlots) {
//    addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  } else {
    nop();
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeFunction(Register fun,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    bool withArgsSlots) {
  // Contract with called JS functions requires that function is passed in a1.
  ASSERT(fun.is(a1));

  Register expected_reg = a2;
  Register code_reg = a3;       // t9 ?

//  ldr(code_reg, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
//  ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
//  ldr(expected_reg,
//      FieldMemOperand(code_reg,
//                      SharedFunctionInfo::kFormalParameterCountOffset));
//  ldr(code_reg,
//      MemOperand(code_reg, SharedFunctionInfo::kCodeOffset - kHeapObjectTag));
//  add(code_reg, code_reg, Operand(Code::kHeaderSize - kHeapObjectTag));
  lw(code_reg, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  lw(cp, FieldMemOperand(a1, JSFunction::kContextOffset));
  lw(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  lw(code_reg,
      MemOperand(code_reg, SharedFunctionInfo::kCodeOffset - kHeapObjectTag));
  addiu(code_reg, code_reg, Operand(Code::kHeaderSize - kHeapObjectTag));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, withArgsSlots);
  // We want the branch delay slot to be free.
}


#ifdef ENABLE_DEBUGGER_SUPPORT
void MacroAssembler::SaveRegistersToMemory(RegList regs) {
  ASSERT((regs & ~kJSCallerSaved) == 0);
  // Copy the content of registers to memory location.
  for (int i = 0; i < kNumJSCallerSaved; i++) {
    int r = JSCallerSavedCode(i);
    if ((regs & (1 << r)) != 0) {
      Register reg = { r };
//      mov(ip, Operand(ExternalReference(Debug_Address::Register(i))));
//      str(reg, MemOperand(ip));
      li(ip, Operand(ExternalReference(Debug_Address::Register(i))));
      sw(reg, MemOperand(ip));
    }
  }
}


void MacroAssembler::RestoreRegistersFromMemory(RegList regs) {
  ASSERT((regs & ~kJSCallerSaved) == 0);
  // Copy the content of memory location to registers.
  for (int i = kNumJSCallerSaved; --i >= 0;) {
    int r = JSCallerSavedCode(i);
    if ((regs & (1 << r)) != 0) {
      Register reg = { r };
//      mov(ip, Operand(ExternalReference(Debug_Address::Register(i))));
//      ldr(reg, MemOperand(ip));
      li(ip, Operand(ExternalReference(Debug_Address::Register(i))));
      lw(reg, MemOperand(ip));
    }
  }
}


void MacroAssembler::CopyRegistersFromMemoryToStack(Register base,
                                                    RegList regs) {
  ASSERT((regs & ~kJSCallerSaved) == 0);
  int16_t ActualNumSaved = 0;
  // Copy the content of the memory location to the stack and adjust base.
  for (int i = kNumJSCallerSaved; --i >= 0;) {
    int r = JSCallerSavedCode(i);
    if ((regs & (1 << r)) != 0) {
//      mov(ip, Operand(ExternalReference(Debug_Address::Register(i))));
//      ldr(ip, MemOperand(ip));
//      str(ip, MemOperand(base, 4, NegPreIndex));
      li(ip, Operand(ExternalReference(Debug_Address::Register(i))));
      lw(ip, MemOperand(ip));
      sw(base, MemOperand(base, -4*(++ActualNumSaved) ));
    }
  }
  addi(base, Operand(-4*ActualNumSaved));
}


void MacroAssembler::CopyRegistersFromStackToMemory(Register base,
                                                    Register scratch,
                                                    RegList regs) {
  ASSERT((regs & ~kJSCallerSaved) == 0);
  int16_t ActualNumSaved = 0;
  // Copy the content of the stack to the memory location and adjust base.
  for (int i = 0; i < kNumJSCallerSaved; i++) {
    int r = JSCallerSavedCode(i);
    if ((regs & (1 << r)) != 0) {
//      mov(ip, Operand(ExternalReference(Debug_Address::Register(i))));
//      ldr(scratch, MemOperand(base, 4, PostIndex));
//      str(scratch, MemOperand(ip));
      li(ip, Operand(ExternalReference(Debug_Address::Register(i))));
      lw(scratch, MemOperand(base, 4*(ActualNumSaved++) ));
      sw(scratch, MemOperand(ip));
    }
  }
  addi(base, Operand(4*ActualNumSaved));
}
#endif

void MacroAssembler::PushTryHandler(CodeLocation try_location,
                                    HandlerType type) {
  // Adjust this code if not the case.
  ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);
  // The pc (return address) is passed in register lr.
  if (try_location == IN_JAVASCRIPT) {
    if (type == TRY_CATCH_HANDLER) {
      li(t0, Operand(StackHandler::TRY_CATCH));
    } else {
      li(t0, Operand(StackHandler::TRY_FINALLY));
    }
    ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize
           && StackHandlerConstants::kFPOffset == 2 * kPointerSize
           && StackHandlerConstants::kPCOffset == 3 * kPointerSize
           && StackHandlerConstants::kNextOffset == 0 * kPointerSize);

    // Save the current handler as the next handler.
    li(t2, Operand(ExternalReference(Top::k_handler_address)));
    lw(t1, MemOperand(t2));
    
    addiu(sp, sp, -StackHandlerConstants::kSize);
    sw(ra, MemOperand(sp, 12));
    sw(fp, MemOperand(sp, 8));
    sw(t0, MemOperand(sp, 4));
    sw(t1, MemOperand(sp, 0));
    
    // Link this handler as the new current one.
    sw(sp, MemOperand(t2));

  } else {
    ASSERT(try_location == IN_JS_ENTRY);
    ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize
           && StackHandlerConstants::kFPOffset == 2 * kPointerSize
           && StackHandlerConstants::kPCOffset == 3 * kPointerSize
           && StackHandlerConstants::kNextOffset == 0 * kPointerSize);

    // The frame pointer does not point to a JS frame so we save NULL
    // for fp. We expect the code throwing an exception to check fp
    // before dereferencing it to restore the context.
    li(t0, Operand(StackHandler::ENTRY));

    // Save the current handler as the next handler.
    li(t2, Operand(ExternalReference(Top::k_handler_address)));
    lw(t1, MemOperand(t2));

    // To optimize the code we don't use a multi_push like function.
    addiu(sp, sp, -StackHandlerConstants::kSize);
    sw(ra, MemOperand(sp, 12));
    sw(zero_reg, MemOperand(sp, 8));
    sw(t0, MemOperand(sp, 4));
    sw(t1, MemOperand(sp, 0));

    // Link this handler as the new current one.
    sw(sp, MemOperand(t2));
  }
}


Register MacroAssembler::CheckMaps(JSObject* object, Register object_reg,
                                   JSObject* holder, Register holder_reg,
                                   Register scratch,
                                   Label* miss) {
  // Make sure there's no overlap between scratch and the other
  // registers.
  ASSERT(!scratch.is(object_reg) && !scratch.is(holder_reg));

  // Keep track of the current object in register reg.
  Register reg = object_reg;
  int depth = 1;

  // Check the maps in the prototype chain.
  // Traverse the prototype chain from the object and do map checks.
  while (object != holder) {
    depth++;

    // Only global objects and objects that do not require access
    // checks are allowed in stubs.
    ASSERT(object->IsJSGlobalProxy() || !object->IsAccessCheckNeeded());

    // Get the map of the current object.
//    ldr(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));
//    cmp(scratch, Operand(Handle<Map>(object->map())));
    lw(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));

    // Branch on the result of the map check.
//    b(ne, miss);
    bcond(ne, miss, scratch, Operand(Handle<Map>(object->map())));
    nop();  // NOP_ADDED

    // Check access rights to the global object.  This has to happen
    // after the map check so that we know that the object is
    // actually a global object.
    if (object->IsJSGlobalProxy()) {
      CheckAccessGlobalProxy(reg, scratch, miss);
      // Restore scratch register to be the map of the object.  In the
      // new space case below, we load the prototype from the map in
      // the scratch register.
//      ldr(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));
      lw(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));
    }

    reg = holder_reg;  // from now the object is in holder_reg
    JSObject* prototype = JSObject::cast(object->GetPrototype());
    if (Heap::InNewSpace(prototype)) {
      // The prototype is in new space; we cannot store a reference
      // to it in the code. Load it from the map.
//      ldr(reg, FieldMemOperand(scratch, Map::kPrototypeOffset));
      lw(reg, FieldMemOperand(scratch, Map::kPrototypeOffset));
    } else {
      // The prototype is in old space; load it directly.
//      mov(reg, Operand(Handle<JSObject>(prototype)));
      li(reg, Operand(Handle<JSObject>(prototype)));
    }

    // Go to the next object in the prototype chain.
    object = prototype;
  }

  // Check the holder map.
//  ldr(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));
//  cmp(scratch, Operand(Handle<Map>(object->map())));
//  b(ne, miss);
  lw(scratch, FieldMemOperand(reg, HeapObject::kMapOffset));
  bcond(ne, miss, scratch, Operand(Handle<Map>(object->map())));
  nop(); // NOP_ADDED

  // Log the check depth.
  LOG(IntEvent("check-maps-depth", depth));

  // Perform security check for access to the global object and return
  // the holder register.
  ASSERT(object == holder);
  ASSERT(object->IsJSGlobalProxy() || !object->IsAccessCheckNeeded());
  if (object->IsJSGlobalProxy()) {
    CheckAccessGlobalProxy(reg, scratch, miss);
  }
  return reg;
}


void MacroAssembler::CheckAccessGlobalProxy(Register holder_reg,
                                            Register scratch,
                                            Label* miss) {
  Label same_contexts;

  ASSERT(!holder_reg.is(scratch));
  ASSERT(!holder_reg.is(ip));
  ASSERT(!scratch.is(ip));

  // Load current lexical context from the stack frame.
//  ldr(scratch, MemOperand(fp, StandardFrameConstants::kContextOffset));
  lw(scratch, MemOperand(fp, StandardFrameConstants::kContextOffset));
  // In debug mode, make sure the lexical context is set.
#ifdef DEBUG
//  cmp(scratch, Operand(0));
  Check(ne, "we should not have an empty lexical context", scratch, Operand(0));
#endif

  // Load the global context of the current context.
  int offset = Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
//  ldr(scratch, FieldMemOperand(scratch, offset));
//  ldr(scratch, FieldMemOperand(scratch, GlobalObject::kGlobalContextOffset));
  lw(scratch, FieldMemOperand(scratch, offset));
  lw(scratch, FieldMemOperand(scratch, GlobalObject::kGlobalContextOffset));

  // Check the context is a global context.
  if (FLAG_debug_code) {
    // TODO(119): avoid push(holder_reg)/pop(holder_reg)
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
    push(holder_reg);  // Temporarily save holder on the stack.
    // Read the first word and compare to the global_context_map.
//    ldr(holder_reg, FieldMemOperand(scratch, HeapObject::kMapOffset));
//    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
//    cmp(holder_reg, ip);
//    Check(eq, "JSGlobalObject::global_context should be a global context.");
//    pop(holder_reg);  // Restore holder.
    lw(holder_reg, FieldMemOperand(scratch, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
    Check(eq, "JSGlobalObject::global_context should be a global context.",
          holder_reg, Operand(ip));
    pop(holder_reg);  // Restore holder.
  }

  // Check if both contexts are the same.
//  ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
//  cmp(scratch, Operand(ip));
//  b(eq, &same_contexts);
  lw(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
  bcond(eq, &same_contexts, scratch, Operand(ip));
  nop(); // NOP_ADDED

  // Check the context is a global context.
  if (FLAG_debug_code) {
    // TODO(119): avoid push(holder_reg)/pop(holder_reg)
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
//    push(holder_reg);  // Temporarily save holder on the stack.
//    mov(holder_reg, ip);  // Move ip to its holding place.
//    LoadRoot(ip, Heap::kNullValueRootIndex);
//    cmp(holder_reg, ip);
//    Check(ne, "JSGlobalProxy::context() should not be null.");
    push(holder_reg);  // Temporarily save holder on the stack.
    mov(holder_reg, ip);  // Move ip to its holding place.
    LoadRoot(ip, Heap::kNullValueRootIndex);
    Check(ne, "JSGlobalProxy::context() should not be null.",
          holder_reg, Operand(ip));

//    ldr(holder_reg, FieldMemOperand(holder_reg, HeapObject::kMapOffset));
//    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
//    cmp(holder_reg, ip);
//    Check(eq, "JSGlobalObject::global_context should be a global context.");
    lw(holder_reg, FieldMemOperand(holder_reg, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
    Check(eq, "JSGlobalObject::global_context should be a global context.",
          holder_reg, Operand(ip));
    // Restore ip is not needed. ip is reloaded below.
//    pop(holder_reg);  // Restore holder.
    pop(holder_reg);  // Restore holder.
    // Restore ip to holder's context.
//    ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
    lw(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
  }

  // Check that the security token in the calling global object is
  // compatible with the security token in the receiving global
  // object.
  int token_offset = Context::kHeaderSize +
                     Context::SECURITY_TOKEN_INDEX * kPointerSize;

//  ldr(scratch, FieldMemOperand(scratch, token_offset));
//  ldr(ip, FieldMemOperand(ip, token_offset));
//  cmp(scratch, Operand(ip));
//  b(ne, miss);
  lw(scratch, FieldMemOperand(scratch, token_offset));
  lw(ip, FieldMemOperand(ip, token_offset));
  bcond(ne, miss, scratch, Operand(ip));
  nop(); // NOP_ADDED

  bind(&same_contexts);
}


void MacroAssembler::AllocateInNewSpace(int object_size,
                                              Register result,
                                              Register scratch1,
                                              Register scratch2,
                                              Label* gc_required,
                                              AllocationFlags flags) {
  ASSERT(!result.is(scratch1));
  ASSERT(!scratch1.is(scratch2));

  // Load address of new object into result and allocation top address into
  // scratch1.
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address();
  li(scratch1, Operand(new_space_allocation_top));
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    lw(result, MemOperand(scratch1));
  } else {
#ifdef DEBUG
    // Assert that result actually contains top on entry. scratch2 is used
    // immediately below so this use of scratch2 does not cause difference with
    // respect to register content between debug and release mode.
    lw(scratch2, MemOperand(scratch1));
    Check(eq, "Unexpected allocation top", result, Operand(scratch2));
#endif
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top.
  ExternalReference new_space_allocation_limit =
      ExternalReference::new_space_allocation_limit_address();
//  mov(scratch2, Operand(new_space_allocation_limit));
//  ldr(scratch2, MemOperand(scratch2));
//  add(result, result, Operand(object_size * kPointerSize));
//  cmp(result, Operand(scratch2));
//  b(hi, gc_required);
  li(scratch2, Operand(new_space_allocation_limit));
  lw(scratch2, MemOperand(scratch2));
  addiu(result, result, Operand(object_size * kPointerSize));
  bcond(Ugreater, gc_required, result, Operand(scratch2));
  nop(); // NOP_ADDED

  // Update allocation top. result temporarily holds the new top,
//  str(result, MemOperand(scratch1));
  sw(result, MemOperand(scratch1));

  // Tag and adjust back to start of new object.
  if ((flags & TAG_OBJECT) != 0) {
    addiu(result, result, Operand(-(object_size * kPointerSize) +
                                kHeapObjectTag));
  } else {
    addiu(result, result, Operand(-object_size * kPointerSize));
  }
}


void MacroAssembler::AllocateInNewSpace(Register object_size,
                                        Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Label* gc_required,
                                        AllocationFlags flags) {
  ASSERT(!result.is(scratch1));
  ASSERT(!scratch1.is(scratch2));

  // Load address of new object into result and allocation top address into
  // scratch1.
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address();
//  mov(scratch1, Operand(new_space_allocation_top));
  li(scratch1, Operand(new_space_allocation_top));
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
//    ldr(result, MemOperand(scratch1));
    lw(result, MemOperand(scratch1));
  } else {
#ifdef DEBUG
    // Assert that result actually contains top on entry. scratch2 is used
    // immediately below so this use of scratch2 does not cause difference with
    // respect to register content between debug and release mode.
//    ldr(scratch2, MemOperand(scratch1));
//    cmp(result, scratch2);
//    Check(eq, "Unexpected allocation top");
    lw(scratch2, MemOperand(scratch1));
    Check(eq, "Unexpected allocation top", result, Operand(scratch2));
#endif
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size is in words so a shift is required to
  // get the number of bytes
  ExternalReference new_space_allocation_limit =
      ExternalReference::new_space_allocation_limit_address();
//  mov(scratch2, Operand(new_space_allocation_limit));
//  ldr(scratch2, MemOperand(scratch2));
//  add(result, result, Operand(object_size, LSL, kPointerSizeLog2));
//  cmp(result, Operand(scratch2));
//  b(hi, gc_required);
  li(scratch2, Operand(new_space_allocation_limit));
  lw(scratch2, MemOperand(scratch2));
  sll(ip, object_size, kPointerSizeLog2);
  addu(result, result, Operand(ip));
  bcond(Ugreater, gc_required, result, Operand(scratch2));
  nop(); // NOP_ADDED

  // Update allocation top. result temporarily holds the new top,
//  str(result, MemOperand(scratch1));
  sw(result, MemOperand(scratch1));

  // Adjust back to start of new object.
//  sub(result, result, Operand(object_size, LSL, kPointerSizeLog2));
  sub(result, result, Operand(ip));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
//    add(result, result, Operand(kHeapObjectTag));
    addiu(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::UndoAllocationInNewSpace(Register object,
                                              Register scratch) {
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address();

  // Make sure the object has no tag before resetting top.
  andi(object, object, Operand(~kHeapObjectTagMask));
#ifdef DEBUG
  // Check that the object un-allocated is below the current top.
//  mov(scratch, Operand(new_space_allocation_top));
//  ldr(scratch, MemOperand(scratch));
//  cmp(object, scratch);
//  Check(lt, "Undo allocation of non allocated memory");
  li(scratch, Operand(new_space_allocation_top));
  lw(scratch, MemOperand(scratch));
  Check(less, "Undo allocation of non allocated memory", object, Operand(scratch));
#endif
  // Write the address of the object to un-allocate as the current top.
  li(scratch, Operand(new_space_allocation_top));
  sw(object, MemOperand(scratch));
}


void MacroAssembler::GetObjectType(Register function,
                                    Register map,
                                    Register type_reg) {
  lw(map, FieldMemOperand(function, HeapObject::kMapOffset));
  lbu(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
}

// REMOVED : code architecture does not fit MIPS. Use GetObjectType and bcond.
//void MacroAssembler::CompareObjectType(Register function,
//                                       Register map,
//                                       Register type_reg,
//                                       InstanceType type) {
//  ldr(map, FieldMemOperand(function, HeapObject::kMapOffset));
//  CompareInstanceType(map, type_reg, type);
//}

// REMOVED : code architecture does not fit MIPS. Use GetObjectType and bcond.
//void MacroAssembler::CompareInstanceType(Register map,
//                                         Register type_reg,
//                                         InstanceType type) {
//  ldrb(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
//  cmp(type_reg, Operand(type));
//}


void MacroAssembler::TryGetFunctionPrototype(Register function,
                                             Register result,
                                             Register scratch,
                                             Label* miss) {
  // Check that the receiver isn't a smi.
  BranchOnSmi(function, miss);
  nop(); // NOP_ADDED

  // Check that the function really is a function.  Load map into result reg.
//  CompareObjectType(function, result, scratch, JS_FUNCTION_TYPE);
//  b(ne, miss);
  GetObjectType(function, result, scratch);
  bcond(ne, miss, scratch, Operand(JS_FUNCTION_TYPE));
  nop(); // NOP_ADDED

  // Make sure that the function has an instance prototype.
  Label non_instance;
//  ldrb(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
//  tst(scratch, Operand(1 << Map::kHasNonInstancePrototype));
//  b(ne, &non_instance);
  lbu(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
  andi(scratch, scratch, Operand(1 << Map::kHasNonInstancePrototype));
  bcond(ne, &non_instance, scratch, Operand(zero_reg));
  nop(); // NOP_ADDED

  // Get the prototype or initial map from the function.
//  ldr(result,
//      FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));
  lw(result,
      FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));

  // If the prototype or initial map is the hole, don't return it and
  // simply miss the cache instead. This will allow us to allocate a
  // prototype object on-demand in the runtime system.
//  LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//  cmp(result, ip);
//  b(eq, miss);
  LoadRoot(ip, Heap::kTheHoleValueRootIndex);
  bcond(eq, miss, result, Operand(ip));
  nop(); // NOP_ADDED

  // If the function does not have an initial map, we're done.
  Label done;
//  CompareObjectType(result, scratch, scratch, MAP_TYPE);
//  b(ne, &done);
  GetObjectType(result, scratch, scratch);
  bcond(ne, &done, scratch, Operand(MAP_TYPE));
  nop(); // NOP_ADDED

  // Get the prototype from the initial map.
//  ldr(result, FieldMemOperand(result, Map::kPrototypeOffset));
//  jmp(&done);
  lw(result, FieldMemOperand(result, Map::kPrototypeOffset));
  b(&done);
  nop(); // NOP_ADDED

  // Non-instance prototype: Fetch prototype from constructor field
  // in initial map.
//  bind(&non_instance);
//  ldr(result, FieldMemOperand(result, Map::kConstructorOffset));
  bind(&non_instance);
  lw(result, FieldMemOperand(result, Map::kConstructorOffset));

  // All done.
  bind(&done);
}


void MacroAssembler::CallStub(CodeStub* stub, Condition cond,
                              Register r1, const Operand& r2) {
  ASSERT(allow_stub_calls());  // stub calls are not allowed in some stubs
  Call(stub->GetCode(), RelocInfo::CODE_TARGET, cond, r1, r2);
}


void MacroAssembler::StubReturn(int argc) {
  ASSERT(argc >= 1 && generating_stub());
  if (argc > 1)
    addiu(sp, sp, Operand((argc - 1) * kPointerSize));
  Ret();
  nop(); // NOP_ADDED
}


void MacroAssembler::IllegalOperation(int num_arguments) {
  UNIMPLEMENTED();
  break_(0x1232);
  if (num_arguments > 0) {
//    add(sp, sp, Operand(num_arguments * kPointerSize));
    addiu(sp, sp, Operand(num_arguments * kPointerSize));
  }
  LoadRoot(v0, Heap::kUndefinedValueRootIndex);
}


void MacroAssembler::CallRuntime(Runtime::Function* f, int num_arguments) {
  // All parameters are on the stack.  r0->v0 has the return value after call.

  // If the expected number of arguments of the runtime function is
  // constant, we check that the actual number of arguments match the
  // expectation.
  if (f->nargs >= 0 && f->nargs != num_arguments) {
    IllegalOperation(num_arguments);
    return;
  }

  Runtime::FunctionId function_id =
      static_cast<Runtime::FunctionId>(f->stub_id);
  RuntimeStub stub(function_id, num_arguments);
  CallStub(&stub);
}


void MacroAssembler::CallRuntime(Runtime::FunctionId fid, int num_arguments) {
  CallRuntime(Runtime::FunctionForId(fid), num_arguments);
}


void MacroAssembler::TailCallRuntime(const ExternalReference& ext,
                                     int num_arguments,
                                     int result_size) {
  // ARM TODO
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
//  mov(r0, Operand(num_arguments));
//  JumpToRuntime(ext);
  li(a0, Operand(num_arguments));
  JumpToRuntime(ext);
  nop(); // NOP_ADDED
}


void MacroAssembler::JumpToRuntime(const ExternalReference& builtin) {
//#if defined(__thumb__)
//  // Thumb mode builtin.
//  ASSERT((reinterpret_cast<intptr_t>(builtin.address()) & 1) == 1);
//#endif
//  mov(r1, Operand(builtin));
//  CEntryStub stub(1);
//  Jump(stub.GetCode(), RelocInfo::CODE_TARGET);
  li(a1, Operand(builtin));
  CEntryStub stub(1);
  Jump(stub.GetCode(), RelocInfo::CODE_TARGET);
}


Handle<Code> MacroAssembler::ResolveBuiltin(Builtins::JavaScript id,
                                            bool* resolved) {
  // Contract with compiled functions is that the function is passed in r1.
  int builtins_offset =
      JSBuiltinsObject::kJSBuiltinsOffset + (id * kPointerSize);
//  ldr(r1, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
//  ldr(r1, FieldMemOperand(r1, GlobalObject::kBuiltinsOffset));
//  ldr(r1, FieldMemOperand(r1, builtins_offset));
  lw(a1, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  lw(a1, FieldMemOperand(a1, GlobalObject::kBuiltinsOffset));
  lw(a1, FieldMemOperand(a1, builtins_offset));

  return Builtins::GetCode(id, resolved);
}


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id,
                                   InvokeJSFlags flags) {
  bool resolved;
  Handle<Code> code = ResolveBuiltin(id, &resolved);

  if (flags == CALL_JS) {
    Call(code, RelocInfo::CODE_TARGET);
  } else {
    ASSERT(flags == JUMP_JS);
    Jump(code, RelocInfo::CODE_TARGET);
  }
//  addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  // Arguments slots are removed in GenCode after frame->Exit().
//  addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);

  if (!resolved) {
    const char* name = Builtins::GetName(id);
    int argc = Builtins::GetArgumentsCount(id);
    uint32_t flags =
        Bootstrapper::FixupFlagsArgumentsCount::encode(argc) |
        Bootstrapper::FixupFlagsUseCodeObject::encode(false);
    Unresolved entry = { pc_offset() - kInstrSize, flags, name };
    unresolved_.Add(entry);
  }
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  bool resolved;
  Handle<Code> code = ResolveBuiltin(id, &resolved);

//  mov(target, Operand(code));
  // We may need to patch this code, so we have li generate 2 instructions.
  li(target, Operand(code), true);
  if (!resolved) {
    const char* name = Builtins::GetName(id);
    int argc = Builtins::GetArgumentsCount(id);
    uint32_t flags =
        Bootstrapper::FixupFlagsArgumentsCount::encode(argc) |
        Bootstrapper::FixupFlagsUseCodeObject::encode(true);
    // li generated 2 instructions, so we need a -2*kInstrSize offset.
    Unresolved entry = { pc_offset() - 2*kInstrSize, flags, name };
    unresolved_.Add(entry);
  }

//  add(target, target, Operand(Code::kHeaderSize - kHeapObjectTag));
  addiu(target, target, Operand(Code::kHeaderSize - kHeapObjectTag));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  if (FLAG_native_code_counters && counter->Enabled()) {
//    mov(scratch1, Operand(value));
//    mov(scratch2, Operand(ExternalReference(counter)));
//    str(scratch1, MemOperand(scratch2));
    li(scratch1, Operand(value));
    li(scratch2, Operand(ExternalReference(counter)));
    sw(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
//    mov(scratch2, Operand(ExternalReference(counter)));
//    ldr(scratch1, MemOperand(scratch2));
//    add(scratch1, scratch1, Operand(value));
//    str(scratch1, MemOperand(scratch2));
    li(scratch2, Operand(ExternalReference(counter)));
    lw(scratch1, MemOperand(scratch2));
    addiu(scratch1, scratch1, Operand(value));
    sw(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
//    mov(scratch2, Operand(ExternalReference(counter)));
//    ldr(scratch1, MemOperand(scratch2));
//    sub(scratch1, scratch1, Operand(value));
//    str(scratch1, MemOperand(scratch2));
    li(scratch2, Operand(ExternalReference(counter)));
    lw(scratch1, MemOperand(scratch2));
    addiu(scratch1, scratch1, Operand(-value));
    sw(scratch1, MemOperand(scratch2));
  }
}



void MacroAssembler::Assert(Condition cc, const char* msg, Register rs, Operand rt) {
  if (FLAG_debug_code)
    Check(cc, msg, rs, rt);
}


void MacroAssembler::Check(Condition cc, const char* msg, Register rs, Operand rt) {
  Label L;
  bcond(cc, &L, rs, rt);
  nop();
  Abort(msg);
  // will not return here
  bind(&L);
}


void MacroAssembler::Abort(const char* msg) {
  // We want to pass the msg string like a smi to avoid GC
  // problems, however msg is not guaranteed to be aligned
  // properly. Instead, we pass an aligned pointer that is
  // a proper v8 smi, but also pass the alignment difference
//  // from the real pointer as a smi.
  intptr_t p1 = reinterpret_cast<intptr_t>(msg);
  intptr_t p0 = (p1 & ~kSmiTagMask) + kSmiTag;
  ASSERT(reinterpret_cast<Object*>(p0)->IsSmi());
#ifdef DEBUG
  if (msg != NULL) {
    RecordComment("Abort message: ");
    RecordComment(msg);
  }
#endif
//  mov(r0, Operand(p0));
//  push(r0);
//  mov(r0, Operand(Smi::FromInt(p1 - p0)));
//  push(r0);
//  CallRuntime(Runtime::kAbort, 2);
  li(a0, Operand(p0));
  push(a0);
  li(a0, Operand(Smi::FromInt(p1 - p0)));
  push(a0);
  CallRuntime(Runtime::kAbort, 2);
  // will not return here
}


#ifdef ENABLE_DEBUGGER_SUPPORT
CodePatcher::CodePatcher(byte* address, int instructions)
    : address_(address),
      instructions_(instructions),
      size_(instructions * Assembler::kInstrSize),
      masm_(address, size_ + Assembler::kGap) {
  // Create a new macro assembler pointing to the address of the code to patch.
  // The size is adjusted with kGap on order for the assembler to generate size
  // bytes of instructions without failing with buffer size constraints.
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


CodePatcher::~CodePatcher() {
  // Indicate that code has changed.
  CPU::FlushICache(address_, size_);

  // Check that the code was patched as expected.
  ASSERT(masm_.pc_ == address_ + size_);
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


void CodePatcher::Emit(Instr x) {
  masm()->emit(x);
}


void CodePatcher::Emit(Address addr) {
  masm()->emit(reinterpret_cast<Instr>(addr));
}
#endif  // ENABLE_DEBUGGER_SUPPORT


} }  // namespace v8::internal
