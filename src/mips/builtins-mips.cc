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
#include "debug.h"
#include "runtime.h"

namespace v8 {
namespace internal {


#define __ ACCESS_MASM(masm)


void Builtins::Generate_Adaptor(MacroAssembler* masm, CFunctionId id) {
  // TODO(428): Don't pass the function in a static variable.
//  __ mov(ip, Operand(ExternalReference::builtin_passed_function()));
//  __ str(r1, MemOperand(ip, 0));
  __ li(ip, Operand(ExternalReference::builtin_passed_function()));
  __ sw(a1, MemOperand(ip, 0));

  // The actual argument count has already been loaded into register
  // r0, but JumpToBuiltin expects r0 to contain the number of
  // arguments including the receiver.
//  __ add(r0, r0, Operand(1));
//  __ JumpToRuntime(ExternalReference(id));
  __ addi(a0, a0, Operand(1));
  // This is really ugly...
  // in Builtins::Generate_JSConstructStubGeneric we allocate some args slots
  // because some functions need it. But here we do not expect them.
//  __ break_(0x00001);
//  __ addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);
  __ JumpToRuntime(ExternalReference(id));
  __ nop(); // NOP_ADDED
}


void Builtins::Generate_ArrayCode(MacroAssembler* masm) {
  // Just jump to the generic array code.
//  __ break_(0x00002);
  Code* code = Builtins::builtin(Builtins::ArrayCodeGeneric);
  Handle<Code> array_code(code);
  __ Jump(array_code, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED
}


void Builtins::Generate_ArrayConstructCode(MacroAssembler* masm) {
  // Just jump to the generic construct code.
  Code* code = Builtins::builtin(Builtins::JSConstructStubGeneric);
  Handle<Code> generic_construct_stub(code);
  __ Jump(generic_construct_stub, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED
}


void Builtins::Generate_JSConstructCall(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- a0     : number of arguments
  //  -- a1     : constructor function
  //  -- ra     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------

  __ li(at, Operand(0xe));
  __ teq(a1, at, 0x123);
  Label non_function_call;
  // Check that the function is not a smi.
  __ andi(t0, a1, Operand(kSmiTagMask));
  __ beq(t0, zero_reg, &non_function_call);
  __ nop(); // NOP_ADDED
  // Check that the function is a JSFunction.
  __ GetObjectType(a1, a2, a2);
  __ bcond(ne, &non_function_call, a2, Operand(JS_FUNCTION_TYPE));
  __ nop(); // NOP_ADDED

  // Jump to the function-specific construct stub.
//  __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
//  __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kConstructStubOffset));
//  __ add(pc, r2, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ lw(a2, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  __ lw(a2, FieldMemOperand(a2, SharedFunctionInfo::kConstructStubOffset));
  __ addiu(t9, a2, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ jr(Operand(t9));
  __ nop(); // NOP_ADDED

  // r0: number of arguments
  // r1: called object
  __ bind(&non_function_call);
//  __ break_(0x04301);

  // Set expected number of arguments to zero (not changing r0).
//  __ mov(r2, Operand(0));
//  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION_AS_CONSTRUCTOR);
//  __ Jump(Handle<Code>(builtin(ArgumentsAdaptorTrampoline)),
//          RelocInfo::CODE_TARGET);
  __ li(a2, Operand(0));
  __ GetBuiltinEntry(a3, Builtins::CALL_NON_FUNCTION_AS_CONSTRUCTOR);
//  __ break_(0x124);
  __ li(v1, Operand(0)); // Tell ArgumentsAdaptorTrampoline we need args slots
  __ Jump(Handle<Code>(builtin(ArgumentsAdaptorTrampoline)),
          RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {


  // Enter a construct frame.
  __ EnterConstructFrame();

  // Preserve the two incoming parameters on the stack.
  // a0: Smi-tagged arguments count. a1: Constructor function.
  __ sll(a0, a0, kSmiTagSize);
  __ multi_push(a0.bit() | a1.bit());

  // Use t7 to hold undefined, which is used in several places below.
  __ LoadRoot(t7, Heap::kUndefinedValueRootIndex);

  Label rt_call, allocated;
  // Try to allocate the object without transitioning into C code. If any of the
  // preconditions is not met, the code bails out to the runtime call.
  if (FLAG_inline_new) {
    Label undo_allocation;
#ifdef ENABLE_DEBUGGER_SUPPORT
    ExternalReference debug_step_in_fp =
        ExternalReference::debug_step_in_fp_address();
    __ li(a2, Operand(debug_step_in_fp));
    __ lw(a2, MemOperand(a2));
    __ bne(a2, zero_reg, &rt_call);
    __ nop(); // NOP_ADDED
#endif

    // Load the initial map and verify that it is in fact a map.
    // r1->a1: constructor function
    // r7->t7: undefined
//    __ ldr(r2, FieldMemOperand(r1, JSFunction::kPrototypeOrInitialMapOffset));
//    __ tst(r2, Operand(kSmiTagMask));
//    __ b(eq, &rt_call);
//    __ CompareObjectType(r2, r3, r4, MAP_TYPE);
//    __ b(ne, &rt_call);
    __ lw(a2, FieldMemOperand(a1, JSFunction::kPrototypeOrInitialMapOffset));
    __ andi(t0, a2, Operand(kSmiTagMask));
    __ beq(t0, zero_reg, &rt_call);
    __ nop();   // NOP_ADDED
    __ GetObjectType(a2, a3, t4);
    __ bcond(ne, &rt_call, t4, Operand(MAP_TYPE));
    __ nop();   // NOP_ADDED

    // Check that the constructor is not constructing a JSFunction (see comments
    // in Runtime_NewObject in runtime.cc). In which case the initial map's
    // instance type would be JS_FUNCTION_TYPE.
    // r1->a1: constructor function
    // r2->a2: initial map
    // r7->t7: undefined
//    __ CompareInstanceType(r2, r3, JS_FUNCTION_TYPE);
    __ lbu(a3, FieldMemOperand(a2, Map::kInstanceTypeOffset));
    __ bcond(eq, &rt_call, a3, Operand(JS_FUNCTION_TYPE));
    __ nop();   // NOP_ADDED

    // Now allocate the JSObject on the heap.
    // r1->a1: constructor function
    // r2->a2: initial map
    // r7->t7: undefined
//    __ ldrb(r3, FieldMemOperand(r2, Map::kInstanceSizeOffset));
    __ lbu(a3, FieldMemOperand(a2, Map::kInstanceSizeOffset));
    __ AllocateInNewSpace(a3, t4, t5, t6, &rt_call, NO_ALLOCATION_FLAGS);

    // Allocated the JSObject, now initialize the fields. Map is set to initial
    // map and properties and elements are set to empty fixed array.
    // r1->a1: constructor function
    // r2->a2: initial map
    // r3->a3: object size
    // r4->t4: JSObject (not tagged)
    // r7->t7: undefined
//    __ LoadRoot(r6, Heap::kEmptyFixedArrayRootIndex);
//    __ mov(r5, r4);
    __ LoadRoot(t6, Heap::kEmptyFixedArrayRootIndex);
    __ mov(t5, t4);
//    __ str(r2, MemOperand(r5, kPointerSize, PostIndex));
//    __ str(r6, MemOperand(r5, kPointerSize, PostIndex));
//    __ str(r6, MemOperand(r5, kPointerSize, PostIndex));
    __ sw(a2, MemOperand(t5, JSObject::kMapOffset));
    __ sw(t6, MemOperand(t5, JSObject::kPropertiesOffset));
    __ sw(t6, MemOperand(t5, JSObject::kElementsOffset));
    __ addiu(t5, t5, Operand(3*kPointerSize));

    ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
    ASSERT_EQ(1 * kPointerSize, JSObject::kPropertiesOffset);
    ASSERT_EQ(2 * kPointerSize, JSObject::kElementsOffset);


    // Fill all the in-object properties with undefined.
    // r1->a1: constructor function
    // r2->a2: initial map
    // r3->a3: object size (in words)
    // r4->t4: JSObject (not tagged)
    // r5->t5: First in-object property of JSObject (not tagged)
    // r7->t7: undefined
//    __ add(r6, r4, Operand(r3, LSL, kPointerSizeLog2));  // End of object.
    __ sll(t0, a3, kPointerSizeLog2);
    __ add(t6, t4, t0);                 // End of object.
    ASSERT_EQ(3 * kPointerSize, JSObject::kHeaderSize);
    { Label loop, entry;
      __ b(&entry);
      __ nop(); // NOP_ADDED
      __ bind(&loop);
//      __ str(r7, MemOperand(r5, kPointerSize, PostIndex));
      __ sw(t7, MemOperand(t5, 0));
      __ addiu(t5, Operand(kPointerSize));
      __ bind(&entry);
//      __ cmp(r5, Operand(r6));
//      __ b(lt, &loop);
      __ bcond(less, &loop, t5, Operand(t6));
      __ nop(); // NOP_ADDED
    }

    // Add the object tag to make the JSObject real, so that we can continue and
    // jump into the continuation code at any time from now on. Any failures
    // need to undo the allocation, so that the heap is in a consistent state
    // and verifiable.
//    __ add(r4, r4, Operand(kHeapObjectTag));
    __ addiu(t4, t4, Operand(kHeapObjectTag));

    // Check if a non-empty properties array is needed. Continue with allocated
    // object if not fall through to runtime call if it is.
    // r1->a1: constructor function
    // r4->t0: JSObject
    // r5->t1: start of next object (not tagged)
    // r7->t7: undefined
//    __ ldrb(r3, FieldMemOperand(r2, Map::kUnusedPropertyFieldsOffset));
    __ lbu(a3, FieldMemOperand(a2, Map::kUnusedPropertyFieldsOffset));
    // The field instance sizes contains both pre-allocated property fields and
    // in-object properties.
//    __ ldr(r0, FieldMemOperand(r2, Map::kInstanceSizesOffset));
//    __ and_(r6,
//            r0,
//            Operand(0x000000FF << Map::kPreAllocatedPropertyFieldsByte * 8));
//    __ add(r3, r3, Operand(r6, LSR, Map::kPreAllocatedPropertyFieldsByte * 8));
//    __ and_(r6, r0, Operand(0x000000FF << Map::kInObjectPropertiesByte * 8));
//    __ sub(r3, r3, Operand(r6, LSR, Map::kInObjectPropertiesByte * 8), SetCC);
    __ lw(a0, FieldMemOperand(a2, Map::kInstanceSizesOffset));
    __ andi(t6,
            a0,
            Operand(0x000000FF << Map::kPreAllocatedPropertyFieldsByte * 8));
    __ srl(t0, t6, Map::kPreAllocatedPropertyFieldsByte * 8);
    __ addu(a3, a3, Operand(t0));
    __ and_(t6, a0, Operand(0x000000FF << Map::kInObjectPropertiesByte * 8));
    __ srl(t0, t6, Map::kInObjectPropertiesByte * 8);
    __ sub(a3, a3, Operand(t0));

    // Done if no extra properties are to be allocated.
//    __ b(eq, &allocated);
    __ beq(a3, zero_reg, &allocated);
    __ nop(); // NOP_ADDED
    __ Assert(greater_equal, "Property allocation count failed.", a3, Operand(zero_reg));

    // Scale the number of elements by pointer size and add the header for
    // FixedArrays to the start of the next object calculation from above.
    // r1->a1: constructor
    // r3->a3: number of elements in properties array
    // r4->t4: JSObject
    // r5->t5: start of next object
    // r7->t7: undefined
//    __ add(r0, r3, Operand(FixedArray::kHeaderSize / kPointerSize));
    __ addiu(a0, a3, Operand(FixedArray::kHeaderSize / kPointerSize));
    __ AllocateInNewSpace(a0,
                          t5,
                          t6,
                          a2,
                          &undo_allocation,
                          RESULT_CONTAINS_TOP);

    // Initialize the FixedArray.
    // r1->a1: constructor
    // r3->a3: number of elements in properties array
    // r4->t4: JSObject
    // r5->t5: start of next object
    // r7->t7: undefined
    __ LoadRoot(t6, Heap::kFixedArrayMapRootIndex);
//    __ mov(r2, r5);
    __ mov(a2, t5);
    ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
//    __ str(r6, MemOperand(r2, kPointerSize, PostIndex));
    __ sw(t6, MemOperand(a2, JSObject::kMapOffset));
    ASSERT_EQ(1 * kPointerSize, Array::kLengthOffset);
//    __ str(r3, MemOperand(r2, kPointerSize, PostIndex));
    __ sw(a3, MemOperand(a2, Array::kLengthOffset));

    __ addiu(a2, a2, Operand(2*kPointerSize));

    // Initialize the fields to undefined.
    // r1->a1: constructor
    // r2->a2: First element of FixedArray (not tagged)
    // r3->a3: number of elements in properties array
    // r4->t4: JSObject
    // r5->t5: FixedArray (not tagged)
    // r7->t7: undefined
//    __ add(r6, r2, Operand(r3, LSL, kPointerSizeLog2));  // End of object.
    __ sll(t3, a3, kPointerSizeLog2);
    __ add(t6, a2, Operand(t3));  // End of object.
    ASSERT_EQ(2 * kPointerSize, FixedArray::kHeaderSize);
    { Label loop, entry;
      __ b(&entry);
      __ nop(); // NOP_ADDED
      __ bind(&loop);
//      __ str(r7, MemOperand(r2, kPointerSize, PostIndex));
//      __ bind(&entry);
//      __ cmp(r2, Operand(r6));
//      __ b(lt, &loop);
      __ sw(t7, MemOperand(a2));
      __ addiu(a2, a2, kPointerSize);
      __ bind(&entry);
      __ bcond(less, &loop, a2, Operand(t6));
      __ nop(); // NOP_ADDED
    }

    // Store the initialized FixedArray into the properties field of
    // the JSObject
    // r1->a1: constructor function
    // r4->t4: JSObject
    // r5->t5: FixedArray (not tagged)
//    __ add(r5, r5, Operand(kHeapObjectTag));  // Add the heap tag.
//    __ str(r5, FieldMemOperand(r4, JSObject::kPropertiesOffset));
    __ addiu(t5, t5, Operand(kHeapObjectTag));  // Add the heap tag.
    __ sw(t5, FieldMemOperand(t4, JSObject::kPropertiesOffset));

    // Continue with JSObject being successfully allocated
    // r1: constructor function
    // r4: JSObject
//    __ jmp(&allocated);
    __ b(&allocated);
    __ nop();   // NOP_ADDED

    // Undo the setting of the new top so that the heap is verifiable. For
    // example, the map's unused properties potentially do not match the
    // allocated objects unused properties.
    // r4: JSObject (previous new top)
    __ bind(&undo_allocation);
    __ UndoAllocationInNewSpace(t4, t5);
  }

  __ bind(&rt_call);
  // Allocate the new receiver object using the runtime call.
  // r1->a1: constructor function
  __ push(a1);  // argument for Runtime_NewObject
  __ CallRuntime(Runtime::kNewObject, 1);
  __ nop();
//  __ mov(r4, r0)
  __ mov(t4, v0);

  // Receiver for constructor call allocated.
  // r4->t0: JSObject
  __ bind(&allocated);
//  __ push(r4);
  __ push(t4);

  // Push the function and the allocated receiver from the stack.
  // sp[0]: receiver (newly allocated object)
  // sp[1]: constructor function
  // sp[2]: number of arguments (smi-tagged)
//  __ ldr(r1, MemOperand(sp, kPointerSize));
//  __ push(r1);  // Constructor function.
//  __ push(r4);  // Receiver.
  __ lw(a1, MemOperand(sp, kPointerSize));
  __ push(a1);  // Constructor function.
  __ push(t4);  // Receiver.

  // Reload the number of arguments from the stack.
  // r1->a1: constructor function
  // sp[0]: receiver
  // sp[1]: constructor function
  // sp[2]: receiver
  // sp[3]: constructor function
  // sp[4]: number of arguments (smi-tagged)
//  __ ldr(r3, MemOperand(sp, 4 * kPointerSize));
  __ lw(a3, MemOperand(sp, 4 * kPointerSize));

  // Setup pointer to last argument.
//  __ add(r2, fp, Operand(StandardFrameConstants::kCallerSPOffset));
  __ addiu(a2, fp, Operand(StandardFrameConstants::kCallerSPOffset));
//                          + StandardFrameConstants::kRegularArgsSlotsSize));

  // Setup number of arguments for function call below
//  __ mov(r0, Operand(r3, LSR, kSmiTagSize));
  __ srl(a0, a3, kSmiTagSize);

//  __ break_(0x04205);
  // TODO(MIPS.4): Optimize 'Copy args and receiver to the expression stack.'
  // Copy arguments and receiver to the expression stack.
  // a0: number of arguments
  // a1: constructor function
  // a2: address of last argument (caller sp)
  // a3: number of arguments (smi-tagged)
  // sp[0]: receiver
  // sp[1]: constructor function
  // sp[2]: receiver
  // sp[3]: constructor function
  // sp[4]: number of arguments (smi-tagged)
  Label loop, entry;
  __ b(&entry);
  __ bind(&loop);   // LOOP
  __ sll(t0, a3, kPointerSizeLog2 - kSmiTagSize);
  __ add(t0, t0, Operand(a2));
  __ lw(ip, MemOperand(t0));
  __ push(ip);
  __ bind(&entry);  // ENTRY
  __ addiu(a3, a3, Operand(-2));
  __ bcond(greater_equal, &loop, a3, Operand(zero_reg));
  __ nop();

  // Call the function.
  // a0: number of arguments
  // a1: constructor function
  ParameterCount actual(a0);
  __ InvokeFunction(a1, actual, CALL_FUNCTION, false);
  __ nop(); // NOP_ADDED

  // Pop the function from the stack.
  // sp[0]: constructor function
  // sp[2]: receiver
  // sp[3]: constructor function
  // sp[4]: number of arguments (smi-tagged)
  __ pop();

  // Restore context from the frame.
  // v0: result
  // sp[0]: receiver
  // sp[1]: constructor function
  // sp[2]: number of arguments (smi-tagged)
//  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ lw(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));

  // If the result is an object (in the ECMA sense), we should get rid
  // of the receiver and use the result; see ECMA-262 section 13.2.2-7
  // on page 74.
  Label use_receiver, exit;

  // If the result is a smi, it is *not* an object in the ECMA sense.
  // v0: result
  // sp[0]: receiver (newly allocated object)
  // sp[1]: constructor function
  // sp[2]: number of arguments (smi-tagged)
//  __ tst(r0, Operand(kSmiTagMask));
//  __ b(eq, &use_receiver);
  __ andi(t0, v0, Operand(kSmiTagMask));
  __ beq(t0, zero_reg, &use_receiver);
  __ nop(); // NOP_ADDED


  // If the type of the result (stored in its map) is less than
  // FIRST_JS_OBJECT_TYPE, it is not an object in the ECMA sense.
//  __ CompareObjectType(r0, r3, r3, FIRST_JS_OBJECT_TYPE);
//  __ b(ge, &exit);
  __ GetObjectType(v0, a3, a3);
  __ bcond(greater_equal, &exit, a3, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED

  // Throw away the result of the constructor invocation and use the
  // on-stack receiver as the result.
  __ bind(&use_receiver);
//  __ ldr(r0, MemOperand(sp));
  __ lw(v0, MemOperand(sp));

  // Remove receiver from the stack, remove caller arguments, and
  // return.
  __ bind(&exit);
  // v0: result
  // sp[0]: receiver (newly allocated object)
  // sp[1]: constructor function
  // sp[2]: number of arguments (smi-tagged)
//  __ ldr(r1, MemOperand(sp, 2 * kPointerSize));
//  __ LeaveConstructFrame();
//  __ add(sp, sp, Operand(r1, LSL, kPointerSizeLog2 - 1));
//  __ add(sp, sp, Operand(kPointerSize));
//  __ IncrementCounter(&Counters::constructed_objects, 1, r1, r2);
//  __ Jump(lr);
  __ lw(a1, MemOperand(sp, 2 * kPointerSize));
  __ LeaveConstructFrame();
  __ sll(t0, a1, kPointerSizeLog2 - 1);
  __ add(sp, sp, t0);
  __ add(sp, sp, Operand(kPointerSize));
  __ IncrementCounter(&Counters::constructed_objects, 1, a1, a2);
  __ Jump(ra);
  __ nop(); // NOP_ADDED
}


static void Generate_JSEntryTrampolineHelper(MacroAssembler* masm,
                                             bool is_construct) {
  // Called from JSEntryStub::GenerateBody
  // a0: code entry
  // a1: function
  // a2: receiver
  // a3: argc
  // s0: argv


  // Clear the context before we push it when entering the JS frame.
  __ li(cp, Operand(0));
  
  // Enter an internal frame.
  __ EnterInternalFrame();

  // Set up the context from the function argument.
  __ lw(cp, FieldMemOperand(a1, JSFunction::kContextOffset));

  // Set up the roots register.
  ExternalReference roots_address = ExternalReference::roots_address();
  __ li(s4, Operand(roots_address));

  // Push the function and the receiver onto the stack.
  __ multi_push(a1.bit() | a2.bit());


  // Copy arguments to the stack in a loop.
  // a1: function
  // a3: argc
  // s0: argv, ie points to first arg
  Label loop, entry;
  __ sll(t0, a3, kPointerSizeLog2);
  __ add(a2, s0, t0);
  // a2 points past last arg.
  __ b(&entry);
  __ nop(); // NOP_ADDED
  __ bind(&loop);
  __ lw(a0, MemOperand(s0));  // read next parameter
  __ addiu(s0, Operand(kPointerSize));
  __ lw(a0, MemOperand(a0));  // dereference handle
  __ push(a0);  // push parameter.
  __ bind(&entry);
  __ bcond(ne, &loop, s0, Operand(a2));
  __ nop(); // NOP_ADDED

  // Initialize all JavaScript callee-saved registers, since they will be seen
  // by the garbage collector as part of handlers.
  __ LoadRoot(t4, Heap::kUndefinedValueRootIndex);
  __ mov(s1, t4);
  __ mov(s2, t4);
  __ mov(s3, t4);   // s3 is used for sp temp save.
//  __ mov(s4, s4); // s4 holds the root address. Do not init.
  __ mov(s5, t4);   // Used for condition evaluation.
  __ mov(s6, t4);   // Used for condition evaluation.
//  __ mov(s7, s7); // s7 is cp. Do not init.

  // Invoke the code and pass argc as a0.
  __ mov(a0, a3);
  if (is_construct) {
    __ break_(0x594);
    // We don't need arguments slots since we jump to our code.
    __ Call(Handle<Code>(Builtins::builtin(Builtins::JSConstructCall)),
            RelocInfo::CODE_TARGET);
    __ nop();
  } else {
    ParameterCount actual(a0);
    __ InvokeFunction(a1, actual, CALL_FUNCTION, false);
    // Args slots are allocated in InvokeFunction.
  }

  // Exit the JS frame and remove the parameters (except function), and return.
  // Respect ABI stack constraint.
  __ LeaveInternalFrame();

  __ Jump(ra);
  __ nop(); // NOP_ADDED

  // v0: result

}


void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, false);
}


void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, true);
}


void Builtins::Generate_FunctionCall(MacroAssembler* masm) {
  UNIMPLEMENTED();
  __ nop();
  __ break_(0x77711);
//  __ break_(0x12345);
//  // 1. Make sure we have at least one argument.
//  // r0: actual number of argument
//  { Label done;
//    __ tst(r0, Operand(r0));
//    __ b(ne, &done);
//    __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
//    __ push(r2);
//    __ add(r0, r0, Operand(1));
//    __ bind(&done);
//  }
//
//  // 2. Get the function to call from the stack.
//  // r0: actual number of argument
//  { Label done, non_function, function;
//    __ ldr(r1, MemOperand(sp, r0, LSL, kPointerSizeLog2));
//    __ tst(r1, Operand(kSmiTagMask));
//    __ b(eq, &non_function);
//    __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
//    __ b(eq, &function);
//
//    // Non-function called: Clear the function to force exception.
//    __ bind(&non_function);
//    __ mov(r1, Operand(0));
//    __ b(&done);
//
//    // Change the context eagerly because it will be used below to get the
//    // right global object.
//    __ bind(&function);
//    __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
//
//    __ bind(&done);
//  }
//
//  // 3. Make sure first argument is an object; convert if necessary.
//  // r0: actual number of arguments
//  // r1: function
//  { Label call_to_object, use_global_receiver, patch_receiver, done;
//    __ add(r2, sp, Operand(r0, LSL, kPointerSizeLog2));
//    __ ldr(r2, MemOperand(r2, -kPointerSize));
//
//    // r0: actual number of arguments
//    // r1: function
//    // r2: first argument
//    __ tst(r2, Operand(kSmiTagMask));
//    __ b(eq, &call_to_object);
//
//    __ LoadRoot(r3, Heap::kNullValueRootIndex);
//    __ cmp(r2, r3);
//    __ b(eq, &use_global_receiver);
//    __ LoadRoot(r3, Heap::kUndefinedValueRootIndex);
//    __ cmp(r2, r3);
//    __ b(eq, &use_global_receiver);
//
//    __ CompareObjectType(r2, r3, r3, FIRST_JS_OBJECT_TYPE);
//    __ b(lt, &call_to_object);
//    __ cmp(r3, Operand(LAST_JS_OBJECT_TYPE));
//    __ b(le, &done);
//
//    __ bind(&call_to_object);
//    __ EnterInternalFrame();
//
//    // Store number of arguments and function across the call into the runtime.
//    __ mov(r0, Operand(r0, LSL, kSmiTagSize));
//    __ push(r0);
//    __ push(r1);
//
//    __ push(r2);
//    __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
//    __ mov(r2, r0);
//
//    // Restore number of arguments and function.
//    __ pop(r1);
//    __ pop(r0);
//    __ mov(r0, Operand(r0, ASR, kSmiTagSize));
//
//    __ LeaveInternalFrame();
//    __ b(&patch_receiver);
//
//    // Use the global receiver object from the called function as the receiver.
//    __ bind(&use_global_receiver);
//    const int kGlobalIndex =
//        Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
//    __ ldr(r2, FieldMemOperand(cp, kGlobalIndex));
//    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalReceiverOffset));
//
//    __ bind(&patch_receiver);
//    __ add(r3, sp, Operand(r0, LSL, kPointerSizeLog2));
//    __ str(r2, MemOperand(r3, -kPointerSize));
//
//    __ bind(&done);
//  }
//
//  // 4. Shift stuff one slot down the stack
//  // r0: actual number of arguments (including call() receiver)
//  // r1: function
//  { Label loop;
//    // Calculate the copy start address (destination). Copy end address is sp.
//    __ add(r2, sp, Operand(r0, LSL, kPointerSizeLog2));
//    __ add(r2, r2, Operand(kPointerSize));  // copy receiver too
//
//    __ bind(&loop);
//    __ ldr(ip, MemOperand(r2, -kPointerSize));
//    __ str(ip, MemOperand(r2));
//    __ sub(r2, r2, Operand(kPointerSize));
//    __ cmp(r2, sp);
//    __ b(ne, &loop);
//  }
//
//  // 5. Adjust the actual number of arguments and remove the top element.
//  // r0: actual number of arguments (including call() receiver)
//  // r1: function
//  __ sub(r0, r0, Operand(1));
//  __ add(sp, sp, Operand(kPointerSize));
//
//  // 6. Get the code for the function or the non-function builtin.
//  //    If number of expected arguments matches, then call. Otherwise restart
//  //    the arguments adaptor stub.
//  // r0: actual number of arguments
//  // r1: function
//  { Label invoke;
//    __ tst(r1, r1);
//    __ b(ne, &invoke);
//    __ mov(r2, Operand(0));  // expected arguments is 0 for CALL_NON_FUNCTION
//    __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
//    __ Jump(Handle<Code>(builtin(ArgumentsAdaptorTrampoline)),
//                         RelocInfo::CODE_TARGET);
//
//    __ bind(&invoke);
//    __ ldr(r3, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
//    __ ldr(r2,
//           FieldMemOperand(r3,
//                           SharedFunctionInfo::kFormalParameterCountOffset));
//    __ ldr(r3,
//           MemOperand(r3, SharedFunctionInfo::kCodeOffset - kHeapObjectTag));
//    __ add(r3, r3, Operand(Code::kHeaderSize - kHeapObjectTag));
//    __ cmp(r2, r0);  // Check formal and actual parameter counts.
//    __ Jump(Handle<Code>(builtin(ArgumentsAdaptorTrampoline)),
//                         RelocInfo::CODE_TARGET, ne);
//
//    // 7. Jump to the code in r3 without checking arguments.
//    ParameterCount expected(0);
//    __ InvokeCode(r3, expected, expected, JUMP_FUNCTION);
//  }
}


void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  UNIMPLEMENTED();
  __ nop();
  __ break_(0x77712);
//  const int kIndexOffset    = -5 * kPointerSize;
//  const int kLimitOffset    = -4 * kPointerSize;
//  const int kArgsOffset     =  2 * kPointerSize;
//  const int kRecvOffset     =  3 * kPointerSize;
//  const int kFunctionOffset =  4 * kPointerSize;
//
//  __ EnterInternalFrame();
//
//  __ ldr(r0, MemOperand(fp, kFunctionOffset));  // get the function
//  __ push(r0);
//  __ ldr(r0, MemOperand(fp, kArgsOffset));  // get the args array
//  __ push(r0);
//  __ InvokeBuiltin(Builtins::APPLY_PREPARE, CALL_JS);
//
//  Label no_preemption, retry_preemption;
//  __ bind(&retry_preemption);
//  ExternalReference stack_guard_limit_address =
//      ExternalReference::address_of_stack_guard_limit();
//  __ mov(r2, Operand(stack_guard_limit_address));
//  __ ldr(r2, MemOperand(r2));
//  __ cmp(sp, r2);
//  __ b(hi, &no_preemption);
//
//  // We have encountered a preemption or stack overflow already before we push
//  // the array contents.  Save r0 which is the Smi-tagged length of the array.
//  __ push(r0);
//
//  // Runtime routines expect at least one argument, so give it a Smi.
//  __ mov(r0, Operand(Smi::FromInt(0)));
//  __ push(r0);
//  __ CallRuntime(Runtime::kStackGuard, 1);
//
//  // Since we returned, it wasn't a stack overflow.  Restore r0 and try again.
//  __ pop(r0);
//  __ b(&retry_preemption);
//
//  __ bind(&no_preemption);
//
//  // Eagerly check for stack-overflow before starting to push the arguments.
//  // r0: number of arguments.
//  // r2: stack limit.
//  Label okay;
//  __ sub(r2, sp, r2);
//
//  __ cmp(r2, Operand(r0, LSL, kPointerSizeLog2 - kSmiTagSize));
//  __ b(hi, &okay);
//
//  // Out of stack space.
//  __ ldr(r1, MemOperand(fp, kFunctionOffset));
//  __ push(r1);
//  __ push(r0);
//  __ InvokeBuiltin(Builtins::APPLY_OVERFLOW, CALL_JS);
//
//  // Push current limit and index.
//  __ bind(&okay);
//  __ push(r0);  // limit
//  __ mov(r1, Operand(0));  // initial index
//  __ push(r1);
//
//  // Change context eagerly to get the right global object if necessary.
//  __ ldr(r0, MemOperand(fp, kFunctionOffset));
//  __ ldr(cp, FieldMemOperand(r0, JSFunction::kContextOffset));
//
//  // Compute the receiver.
//  Label call_to_object, use_global_receiver, push_receiver;
//  __ ldr(r0, MemOperand(fp, kRecvOffset));
//  __ tst(r0, Operand(kSmiTagMask));
//  __ b(eq, &call_to_object);
//  __ LoadRoot(r1, Heap::kNullValueRootIndex);
//  __ cmp(r0, r1);
//  __ b(eq, &use_global_receiver);
//  __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
//  __ cmp(r0, r1);
//  __ b(eq, &use_global_receiver);
//
//  // Check if the receiver is already a JavaScript object.
//  // r0: receiver
//  __ CompareObjectType(r0, r1, r1, FIRST_JS_OBJECT_TYPE);
//  __ b(lt, &call_to_object);
//  __ cmp(r1, Operand(LAST_JS_OBJECT_TYPE));
//  __ b(le, &push_receiver);
//
//  // Convert the receiver to a regular object.
//  // r0: receiver
//  __ bind(&call_to_object);
//  __ push(r0);
//  __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
//  __ b(&push_receiver);
//
//  // Use the current global receiver object as the receiver.
//  __ bind(&use_global_receiver);
//  const int kGlobalOffset =
//      Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
//  __ ldr(r0, FieldMemOperand(cp, kGlobalOffset));
//  __ ldr(r0, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));
//
//  // Push the receiver.
//  // r0: receiver
//  __ bind(&push_receiver);
//  __ push(r0);
//
//  // Copy all arguments from the array to the stack.
//  Label entry, loop;
//  __ ldr(r0, MemOperand(fp, kIndexOffset));
//  __ b(&entry);
//
//  // Load the current argument from the arguments array and push it to the
//  // stack.
//  // r0: current argument index
//  __ bind(&loop);
//  __ ldr(r1, MemOperand(fp, kArgsOffset));
//  __ push(r1);
//  __ push(r0);
//
//  // Call the runtime to access the property in the arguments array.
//  __ CallRuntime(Runtime::kGetProperty, 2);
//  __ push(r0);
//
//  // Use inline caching to access the arguments.
//  __ ldr(r0, MemOperand(fp, kIndexOffset));
//  __ add(r0, r0, Operand(1 << kSmiTagSize));
//  __ str(r0, MemOperand(fp, kIndexOffset));
//
//  // Test if the copy loop has finished copying all the elements from the
//  // arguments object.
//  __ bind(&entry);
//  __ ldr(r1, MemOperand(fp, kLimitOffset));
//  __ cmp(r0, r1);
//  __ b(ne, &loop);
//
//  // Invoke the function.
//  ParameterCount actual(r0);
//  __ mov(r0, Operand(r0, ASR, kSmiTagSize));
//  __ ldr(r1, MemOperand(fp, kFunctionOffset));
//  __ InvokeFunction(r1, actual, CALL_FUNCTION);
//
//  // Tear down the internal frame and remove function, receiver and args.
//  __ LeaveInternalFrame();
//  __ add(sp, sp, Operand(3 * kPointerSize));
//  __ Jump(lr);
}

static void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
//  __ mov(r0, Operand(r0, LSL, kSmiTagSize));
//  __ mov(r4, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
//  __ stm(db_w, sp, r0.bit() | r1.bit() | r4.bit() | fp.bit() | lr.bit());
//  __ add(fp, sp, Operand(3 * kPointerSize));
  __ sll(a0, a0, kSmiTagSize);
  __ li(t0, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ multi_push_reversed(a0.bit() | a1.bit() | t0.bit() | fp.bit() | ra.bit());
  __ add(fp, sp, Operand(3 * kPointerSize));
}


static void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
//  __ ldr(r1, MemOperand(fp, -3 * kPointerSize));
//  __ mov(sp, fp);
//  __ ldm(ia_w, sp, fp.bit() | lr.bit());
//  __ add(sp, sp, Operand(r1, LSL, kPointerSizeLog2 - kSmiTagSize));
//  __ add(sp, sp, Operand(kPointerSize));  // adjust for receiver
  __ lw(a1, MemOperand(fp, -3 * kPointerSize));
  __ mov(sp, fp);
  __ multi_pop_reversed(fp.bit() | ra.bit());
  __ sll(ip, a1, kPointerSizeLog2 - kSmiTagSize);
//  __ addiu(ip, ip, StandardFrameConstants::kRArgsSlotsSize);
  __ addu(sp, sp, ip);
  __ addiu(sp, sp, Operand(kPointerSize));  // adjust for receiver
}


void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0->a0 : actual number of arguments
  //  -- r1->a1 : function (passed through to callee)
  //  -- r2->a2 : expected number of arguments
  //  -- r3->a3 : code entry to call
//  //  -- v1     : use args slots (==0) or not (!=0)
  // -----------------------------------

  Label invoke, dont_adapt_arguments;

  Label enough, too_few;
//  __ cmp(r0, Operand(r2));
//  __ b(lt, &too_few);
//  __ cmp(r2, Operand(SharedFunctionInfo::kDontAdaptArgumentsSentinel));
//  __ b(eq, &dont_adapt_arguments);
  __ bcond(less, &too_few, a0, Operand(a2));
  __ nop(); // NOP_ADDED
  __ bcond(eq, &dont_adapt_arguments, a2, Operand(SharedFunctionInfo::kDontAdaptArgumentsSentinel));
  __ nop(); // NOP_ADDED

  {  // Enough parameters: actual >= expected
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address into r2.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
//    __ add(r0, fp, Operand(r0, LSL, kPointerSizeLog2 - kSmiTagSize));
    // adjust for return address and receiver
//    __ add(r0, r0, Operand(2 * kPointerSize));
//    __ sub(r2, r0, Operand(r2, LSL, kPointerSizeLog2));
    __ sll(a0, a0, kPointerSizeLog2 - kSmiTagSize);
    __ addu(a0, a0, fp);
    // adjust for return address and receiver
    __ addu(a0, a0, Operand(2 * kPointerSize));
    __ sll(a2, a2, kPointerSizeLog2);
//    __ break_(0x11175);
    __ subu(a2, a0, Operand(a2));

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: copy end address
    // r3: code entry to call

    Label copy;
    __ bind(&copy);
//    __ ldr(ip, MemOperand(r0, 0));
//    __ push(ip);
//    __ cmp(r0, r2);  // Compare before moving to next argument.
//    __ sub(r0, r0, Operand(kPointerSize));
//    __ b(ne, &copy);
    __ lw(ip, MemOperand(a0, 0));
    __ push(ip);
    __ bcond(ne, &copy, a0, Operand(a2));
    __ addiu(a0, a0, Operand(-kPointerSize));

//    __ b(&invoke);
    __ b(&invoke);
    __ nop();
  }

  {  // Too few parameters: Actual < expected
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address is fp.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
//    __ add(r0, fp, Operand(r0, LSL, kPointerSizeLog2 - kSmiTagSize));
    __ sll(a0, a0, kPointerSizeLog2 - kSmiTagSize);
    __ addu(a0, a0, fp);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    Label copy;
    __ bind(&copy);
    // Adjust load for return address and receiver.
//    __ ldr(ip, MemOperand(r0, 2 * kPointerSize));
//    __ push(ip);
//    __ cmp(r0, fp);  // Compare before moving to next argument.
//    __ sub(r0, r0, Operand(kPointerSize));
//    __ b(ne, &copy);
    __ lw(ip, MemOperand(a0, 2 * kPointerSize));
    __ push(ip);
    __ bcond(ne, &copy, a0, Operand(fp));
    __ addiu(a0, a0, Operand(-kPointerSize));

    // Fill the remaining expected arguments with undefined.
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//    __ sub(r2, fp, Operand(r2, LSL, kPointerSizeLog2));
//    __ sub(r2, r2, Operand(4 * kPointerSize));  // Adjust for frame.
    __ sll(a2, a2, kPointerSizeLog2);
    __ subu(a2, fp, Operand(a2));
    __ addiu(a2, a2, Operand(-4 * kPointerSize));  // Adjust for frame.

    Label fill;
    __ bind(&fill);
    __ push(ip);
//    __ cmp(sp, r2);
//    __ b(ne, &fill);
    __ bcond(ne, &fill, sp, Operand(a2));
    __ nop(); // NOP_ADDED
  }

  // Call the entry point.
  __ bind(&invoke);

  // Check if we need args slots
  // We don't restore sp on return because LeaveArgumentsAdaptorFrame does the
  // job for us.
//  __ bne(v1, zero_reg, 3);
//  __ nop();
////  __ break_(0x1076);
//  __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);

//  __ Call(r3);
  __ Call(a3);
  __ nop(); // NOP_ADDED

  // We don't restore because LeaveArgumentsAdaptorFrame does it.
//  __ addiu(sp, sp, x-StandardFrameConstants::kRArgsSlotsSize);

  // Exit frame and return.
  LeaveArgumentsAdaptorFrame(masm);
//  __ Jump(lr);
  __ Jump(ra);
  __ nop(); // NOP_ADDED


  // -------------------------------------------
  // Dont adapt arguments.
  // -------------------------------------------
  __ bind(&dont_adapt_arguments);
//  __ Jump(r3);
  __ Jump(a3);
  __ nop(); // NOP_ADDED
}


#undef __

} }  // namespace v8::internal
