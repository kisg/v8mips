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

#include "codegen-inl.h"
#include "register-allocator-inl.h"
#include "scopes.h"

namespace v8 {
namespace internal {

// -------------------------------------------------------------------------
// VirtualFrame implementation.

#define __ ACCESS_MASM(masm())


// On entry to a function, the virtual frame already contains the
// receiver and the parameters.  All initial frame elements are in
// memory.
VirtualFrame::VirtualFrame()
    : elements_(parameter_count() + local_count() + kPreallocatedElements),
      stack_pointer_(parameter_count()) {  // 0-based index of TOS.
  for (int i = 0; i <= stack_pointer_; i++) {
    elements_.Add(FrameElement::MemoryElement());
  }
  for (int i = 0; i < RegisterAllocator::kNumRegisters; i++) {
    register_locations_[i] = kIllegalIndex;
  }
}


void VirtualFrame::SyncElementBelowStackPointer(int index) {
  UNREACHABLE();
}


void VirtualFrame::SyncElementByPushing(int index) {
  UNREACHABLE();
}


// TOCHECK WII? : on mips ?
void VirtualFrame::SyncRange(int begin, int end) {
  // All elements are in memory on ARM (ie, synced).
#ifdef DEBUG
  for (int i = begin; i <= end; i++) {
    ASSERT(elements_[i].is_synced());
  }
#endif
}


void VirtualFrame::MergeTo(VirtualFrame* expected) {
  // ARM frames are currently always in memory.
  // TOCHECK: I presume it is the same on MIPS
  ASSERT(Equals(expected));
}


//void VirtualFrame::MergeMoveRegistersToMemory(VirtualFrame* expected) {
//  UNREACHABLE();
//}


//void VirtualFrame::MergeMoveRegistersToRegisters(VirtualFrame* expected) {
//  UNREACHABLE();
//}


//void VirtualFrame::MergeMoveMemoryToRegisters(VirtualFrame* expected) {
//  UNREACHABLE();
//}

void VirtualFrame::Enter() {
  Comment cmnt(masm(), "[ Enter JS frame");

#ifdef DEBUG
  // Verify that a1 contains a JS function.  The following code relies
  if (FLAG_debug_code) {
    Label map_check, done;

    __ andi(t0, a1, Operand(kSmiTagMask));
    __ bcond(ne, &map_check, zero_reg, Operand(t0));
    __ stop("VirtualFrame::Enter - a1 is not a function (smi check).");
    __ bind(&map_check);

//    __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
//    __ b(eq, &done);
    // TOCHECK : Register use.
    __ GetObjectType(a1, a2, a2);
    __ bcond(eq, &done, a2, Operand(JS_FUNCTION_TYPE));
    __ stop("VirtualFrame::Enter - a1 is not a function (map check).");
    __ bind(&done);
  }
#endif  // DEBUG

  // We are about to push four values to the frame.
  Adjust(4);
//  __ stm(db_w, sp, r1.bit() | cp.bit() | fp.bit() | lr.bit());
//  __ multi_push(a1.bit() | cp.bit() | fp.bit() | ra.bit());
  // We don't use multi_push to optimize the code.
  __ addiu(sp,sp,-16);
  __ sw(ra, MemOperand(sp, 12));
  __ sw(fp, MemOperand(sp, 8));
  __ sw(cp, MemOperand(sp, 4));
  __ sw(a1, MemOperand(sp, 0));

  // Adjust FP to point to saved FP.
//  __ add(fp, sp, Operand(2 * kPointerSize));
  __ addiu(fp, sp, Operand(2 * kPointerSize));

//  cgen()->allocator()->Unuse(t9);
  cgen()->allocator()->Unuse(a1);
  cgen()->allocator()->Unuse(ra);
}


void VirtualFrame::Exit() {
  Comment cmnt(masm(), "[ Exit JS frame");
  // Record the location of the JS exit code for patching when setting
  // break point.
  __ RecordJSReturn();

  // Drop the execution stack down to the frame pointer and restore the caller
  // frame pointer and return address.
  __ mov(sp, fp);

  // Cf end of Enter() function.
//  __ ldm(ia_w, sp, fp.bit() | lr.bit());
  __ lw(fp, MemOperand(sp, 0));
  __ lw(ra, MemOperand(sp, 4));
  __ addiu(sp,sp,8); // Restore sp at its position before the register save area.
}


void VirtualFrame::AllocateStackSlots() {
  int count = local_count();
  if (count > 0) {
    Comment cmnt(masm(), "[ Allocate space for locals");
    Adjust(count);
      // Initialize stack slots with 'undefined' value.
    __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
  }
//  if (FLAG_check_stack) {
////    __ LoadRoot(r2, Heap::kStackLimitRootIndex);
//    __ LoadRoot(a2, Heap::kStackLimitRootIndex);
//  }
  __ addiu(sp, sp, -count*kPointerSize);
  for (int i = 0; i < count; i++) {
//    __ push(ip);
    __ sw(ip, MemOperand(sp, (count-i-1)*kPointerSize));
  }
//  if (FLAG_check_stack) {
//    // TODO : implement this. FLAG_check_stack "if"
//    // ARM comment
//    // Put the lr setup instruction in the delay slot.  The kInstrSize is added
//    // to the implicit 8 byte offset that always applies to operations with pc
//    // and gives a return address 12 bytes down.
//    masm()->add(lr, pc, Operand(Assembler::kInstrSize));
//    masm()->cmp(sp, Operand(r2));
//    StackCheckStub stub;
//    // Call the stub if lower.
//    masm()->mov(pc,
//                Operand(reinterpret_cast<intptr_t>(stub.GetCode().location()),
//                        RelocInfo::CODE_TARGET),
//                LeaveCC,
//                lo);
//  }
}


void VirtualFrame::SaveContextRegister() {
  UNIMPLEMENTED();
}


void VirtualFrame::RestoreContextRegister() {
  UNIMPLEMENTED();
}


void VirtualFrame::PushReceiverSlotAddress() {
  UNIMPLEMENTED();
}


int VirtualFrame::InvalidateFrameSlotAt(int index) {
  return kIllegalIndex;
}


void VirtualFrame::TakeFrameSlotAt(int index) {
  UNIMPLEMENTED();
}


void VirtualFrame::StoreToFrameSlotAt(int index) {
  UNIMPLEMENTED();
}


void VirtualFrame::PushTryHandler(HandlerType type) {
  // Grow the expression stack by handler size less one (the return
  // address in lr is already counted by a call instruction).
  Adjust(kHandlerSize - 1);
  __ PushTryHandler(IN_JAVASCRIPT, type);
}


void VirtualFrame::RawCallStub(CodeStub* stub) {
  ASSERT(cgen()->HasValidEntryRegisters());
  __ CallStub(stub);
}


void VirtualFrame::CallStub(CodeStub* stub, Result* arg) {
  PrepareForCall(0, 0);
  arg->Unuse();
  RawCallStub(stub);
}


void VirtualFrame::CallStub(CodeStub* stub, Result* arg0, Result* arg1) {
  PrepareForCall(0, 0);
  arg0->Unuse();
  arg1->Unuse();
  RawCallStub(stub);
}


void VirtualFrame::CallRuntime(Runtime::Function* f, int arg_count) {
  PrepareForCall(arg_count, arg_count);
  ASSERT(cgen()->HasValidEntryRegisters());
  __ CallRuntime(f, arg_count);
}


void VirtualFrame::CallRuntime(Runtime::FunctionId id, int arg_count) {
  PrepareForCall(arg_count, arg_count);
  ASSERT(cgen()->HasValidEntryRegisters());
  __ CallRuntime(id, arg_count);
}


void VirtualFrame::CallAlignedRuntime(Runtime::Function* f, int arg_count) {
  UNIMPLEMENTED();
  PrepareForCall(arg_count, arg_count);
  ASSERT(cgen()->HasValidEntryRegisters());
  __ CallRuntime(f, arg_count);
}


void VirtualFrame::CallAlignedRuntime(Runtime::FunctionId id, int arg_count) {
  UNIMPLEMENTED();
  PrepareForCall(arg_count, arg_count);
  ASSERT(cgen()->HasValidEntryRegisters());
  __ CallRuntime(id, arg_count);
}


void VirtualFrame::InvokeBuiltin(Builtins::JavaScript id,
                                 InvokeJSFlags flags,
                                 Result* arg_count_register,
                                 int arg_count) {
  ASSERT(arg_count_register->reg().is(a0));
  PrepareForCall(arg_count, arg_count);
  arg_count_register->Unuse();
  __ InvokeBuiltin(id, flags);
}


void VirtualFrame::RawCallCodeObject(Handle<Code> code,
                                       RelocInfo::Mode rmode) {
  ASSERT(cgen()->HasValidEntryRegisters());
  // We don't align here because it may cause the arguments to be unaligned with
  // sp when entering the function.
  __ Call(code, rmode);             // Call the code
}


void VirtualFrame::CallCodeObject(Handle<Code> code,
                                  RelocInfo::Mode rmode,
                                  int dropped_args) {
  int spilled_args = 0;
  switch (code->kind()) {
    case Code::CALL_IC:
      spilled_args = dropped_args + 1;
      break;
    case Code::FUNCTION:
      spilled_args = dropped_args + 1;
      break;
    case Code::KEYED_LOAD_IC:
      ASSERT(dropped_args == 0);
      spilled_args = 2;
      break;
    default:
      // The other types of code objects are called with values
      // in specific registers, and are handled in functions with
      // a different signature.
      UNREACHABLE();
      break;
  }
  PrepareForCall(spilled_args, dropped_args);
  RawCallCodeObject(code, rmode);
  // Do NOT add a nop. Some function use the branch delay slot (eg VisitCall)
}


void VirtualFrame::CallCodeObject(Handle<Code> code,
                                  RelocInfo::Mode rmode,
                                  Result* arg,
                                  int dropped_args) {
  int spilled_args = 0;
  switch (code->kind()) {
    case Code::LOAD_IC:
      ASSERT(arg->reg().is(a2));
      ASSERT(dropped_args == 0);
      spilled_args = 1;
      break;
    case Code::KEYED_STORE_IC:
      ASSERT(arg->reg().is(a0));
      ASSERT(dropped_args == 0);
      spilled_args = 2;
      break;
    default:
      // No other types of code objects are called with values
      // in exactly one register.
      UNREACHABLE();
      break;
  }
//  PrepareForCall(spilled_args, dropped_args);
  arg->Unuse();
  RawCallCodeObject(code, rmode);
  // Do NOT add a nop. Some function use the branch delay slot (eg VisitCall)
}


void VirtualFrame::CallCodeObject(Handle<Code> code,
                                  RelocInfo::Mode rmode,
                                  Result* arg0,
                                  Result* arg1,
                                  int dropped_args,
                                  bool set_auto_args_slots) {
  int spilled_args = 1;
  switch (code->kind()) {
    case Code::STORE_IC:
//      ASSERT(arg0->reg().is(r0));
//      ASSERT(arg1->reg().is(r2));
      ASSERT(arg0->reg().is(a0));
      ASSERT(arg1->reg().is(a2));
      ASSERT(dropped_args == 0);
      spilled_args = 1;
      break;
    case Code::BUILTIN:
      ASSERT(*code == Builtins::builtin(Builtins::JSConstructCall));
//      ASSERT(arg0->reg().is(r0));
//      ASSERT(arg1->reg().is(r1));
      ASSERT(arg0->reg().is(a0));
      ASSERT(arg1->reg().is(a1));
      spilled_args = dropped_args + 1;
      break;
    default:
      // No other types of code objects are called with values
      // in exactly two registers.
      UNREACHABLE();
      break;
  }
  PrepareForCall(spilled_args, dropped_args);
  arg0->Unuse();
  arg1->Unuse();
  RawCallCodeObject(code, rmode);
  if(set_auto_args_slots && !(code->kind() == Code::BUILTIN)) {
//    __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  }
}


void VirtualFrame::Drop(int count) {
  ASSERT(count >= 0);
  ASSERT(height() >= count);
  int num_virtual_elements = (element_count() - 1) - stack_pointer_;

  // Emit code to lower the stack pointer if necessary.
  if (num_virtual_elements < count) {
    int num_dropped = count - num_virtual_elements;
    stack_pointer_ -= num_dropped;
    __ addiu(sp, sp, Operand(num_dropped * kPointerSize));
  }

  // Discard elements from the virtual frame and free any registers.
  for (int i = 0; i < count; i++) {
    FrameElement dropped = elements_.RemoveLast();
    if (dropped.is_register()) {
      Unuse(dropped.reg());
    }
  }
}


void VirtualFrame::DropFromVFrameOnly(int count) {
  ASSERT(count >= 0);
  ASSERT(height() >= count);
  int num_virtual_elements = (element_count() - 1) - stack_pointer_;

  // Emit code to lower the stack pointer if necessary.
  if (num_virtual_elements < count) {
    int num_dropped = count - num_virtual_elements;
    stack_pointer_ -= num_dropped;
  }

  // Discard elements from the virtual frame and free any registers.
  for (int i = 0; i < count; i++) {
    FrameElement dropped = elements_.RemoveLast();
    if (dropped.is_register()) {
      Unuse(dropped.reg());
    }
  }
}


Result VirtualFrame::Pop() {
  return Result();
}


void VirtualFrame::EmitPop(Register reg) {
  ASSERT(stack_pointer_ == element_count() - 1);
  stack_pointer_--;
  elements_.RemoveLast();
  __ pop(reg);
}

void VirtualFrame::EmitMultiPop(RegList regs) {
  ASSERT(stack_pointer_ == element_count() - 1);
  for (int16_t i = RegisterAllocatorConstants::kNumRegisters; --i>=0;) {
    if((regs & (1<<i)) != 0 ) {
  stack_pointer_--;
  elements_.RemoveLast();
    }
  }
  __ multi_pop(regs);
}
void VirtualFrame::EmitMultiPopReversed(RegList regs) {
  ASSERT(stack_pointer_ == element_count() - 1);
  for (int16_t i = RegisterAllocatorConstants::kNumRegisters; --i>=0;) {
    if((regs & (1<<i)) != 0 ) {
  stack_pointer_--;
  elements_.RemoveLast();
    }
  }
  __ multi_pop_reversed(regs);
}


void VirtualFrame::EmitPush(Register reg) {
  ASSERT(stack_pointer_ == element_count() - 1);
  elements_.Add(FrameElement::MemoryElement());
  stack_pointer_++;
  __ push(reg);
}

void VirtualFrame::EmitMultiPush(RegList regs) {
  ASSERT(stack_pointer_ == element_count() - 1);
  for (int16_t i = 0; i< RegisterAllocatorConstants::kNumRegisters; i++) {
    if((regs & (1<<i)) != 0 ) {
      elements_.Add(FrameElement::MemoryElement());
      stack_pointer_++;
    }
  }
  __ multi_push(regs);
}
void VirtualFrame::EmitMultiPushReversed(RegList regs) {
  ASSERT(stack_pointer_ == element_count() - 1);
  for (int16_t i = 0; i< RegisterAllocatorConstants::kNumRegisters; i++) {
    if((regs & (1<<i)) != 0 ) {
      elements_.Add(FrameElement::MemoryElement());
      stack_pointer_++;
    }
  }
  __ multi_push_reversed(regs);
}

void VirtualFrame::EmitArgumentSlots(RegList reglist) {
  ASSERT(stack_pointer_ == element_count() - 1);
  ASSERT(is_uint4(reglist));
  __ addiu(sp, sp, 4*kPointerSize);
  for (int i=0; i<4; i++) {
    elements_.Add(FrameElement::MemoryElement());
    stack_pointer_++;
    if(reglist & 1<<i) { __ sw(RegisterAllocator::ToRegister(i+4), MemOperand(sp, 4-i));}
  }
}

#undef __

} }  // namespace v8::internal
