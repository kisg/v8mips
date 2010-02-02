#include "v8.h"

#include "ic-inl.h"
#include "codegen-inl.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)


static void ProbeTable(MacroAssembler* masm,
                       Code::Flags flags,
                       StubCache::Table table,
                       Register name,
                       Register offset) {
  ExternalReference key_offset(SCTableReference::keyReference(table));
  ExternalReference value_offset(SCTableReference::valueReference(table));

  Label miss;

  // Save the offset on the stack.
  __ push(offset);

  // Check that the key in the entry matches the name.
//  __ mov(ip, Operand(key_offset));
//  __ ldr(ip, MemOperand(ip, offset, LSL, 1));
//  __ cmp(name, Operand(ip));
//  __ b(ne, &miss);
  __ li(ip, Operand(key_offset));
  __ sll(t0, offset, 1);    // Don't clobber t0 we use it again below.
  __ addu(t1, ip, t0);
  __ lw(ip, MemOperand(t1));
  __ bcond(ne, &miss, name, Operand(ip));
  __ nop(); // NOP_ADDED

  // Get the code entry from the cache.
//  __ mov(ip, Operand(value_offset));
//  __ ldr(offset, MemOperand(ip, offset, LSL, 1));
  __ li(ip, Operand(value_offset));
//  __ sll(t0, offset, 1);  // Done a few instructions above.
  __ addu(t1, ip, t0);
  __ lw(offset, MemOperand(t1));

  // Check that the flags match what we're looking for.
//  __ ldr(offset, FieldMemOperand(offset, Code::kFlagsOffset));
//  __ and_(offset, offset, Operand(~Code::kFlagsNotUsedInLookup));
//  __ cmp(offset, Operand(flags));
//  __ b(ne, &miss);
  __ lw(offset, FieldMemOperand(offset, Code::kFlagsOffset));
  __ and_(offset, offset, Operand(~Code::kFlagsNotUsedInLookup));
  __ bcond(ne, &miss, offset, Operand(flags));
  __ nop(); // NOP_ADDED

  // Restore offset and re-load code entry from cache.
  __ pop(offset);
//  __ mov(ip, Operand(value_offset));
//  __ ldr(offset, MemOperand(ip, offset, LSL, 1));
  __ li(ip, Operand(value_offset));
//  __ sll(t0, offset, 1);  // Done a few instructions above.
  __ addu(t1, ip, t0);
  __ lw(ip, MemOperand(t1));

  // Jump to the first instruction in the code stub.
//  __ add(offset, offset, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ addu(offset, offset, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ Jump(offset);
  __ nop(); // NOP_ADDED

  // Miss: Restore offset and fall through.
  __ bind(&miss);
  __ pop(offset);
}


void StubCache::GenerateProbe(MacroAssembler* masm,
                              Code::Flags flags,
                              Register receiver,
                              Register name,
                              Register scratch,
                              Register extra) {
  Label miss;

  // Make sure that code is valid. The shifting code relies on the
  // entry size being 8.
  ASSERT(sizeof(Entry) == 8);

  // Make sure the flags does not name a specific type.
  ASSERT(Code::ExtractTypeFromFlags(flags) == 0);
//
  // Make sure that there are no register conflicts.
  ASSERT(!scratch.is(receiver));
  ASSERT(!scratch.is(name));

  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, &miss);
  __ andi(scratch, receiver, Operand(kSmiTagMask));
  __ bcond(eq, &miss, scratch, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Get the map of the receiver and compute the hash.
//  __ ldr(scratch, FieldMemOperand(name, String::kLengthOffset));
//  __ ldr(ip, FieldMemOperand(receiver, HeapObject::kMapOffset));
//  __ add(scratch, scratch, Operand(ip));
//  __ eor(scratch, scratch, Operand(flags));
//  __ and_(scratch,
//          scratch,
//          Operand((kPrimaryTableSize - 1) << kHeapObjectTagSize));
  __ lw(scratch, FieldMemOperand(name, String::kLengthOffset));
  __ lw(ip, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ addu(scratch, scratch, Operand(ip));
  __ xor_(scratch, scratch, Operand(flags));
  __ and_(scratch,
          scratch,
          Operand((kPrimaryTableSize - 1) << kHeapObjectTagSize));

  // Probe the primary table.
  ProbeTable(masm, flags, kPrimary, name, scratch);

  // Primary miss: Compute hash for secondary probe.
//  __ sub(scratch, scratch, Operand(name));
//  __ add(scratch, scratch, Operand(flags));
//  __ and_(scratch,
//          scratch,
//          Operand((kSecondaryTableSize - 1) << kHeapObjectTagSize));
  __ subu(scratch, scratch, Operand(name));
  __ addu(scratch, scratch, Operand(flags));
  __ and_(scratch,
          scratch,
          Operand((kSecondaryTableSize - 1) << kHeapObjectTagSize));

  // Probe the secondary table.
  ProbeTable(masm, flags, kSecondary, name, scratch);

  // Cache miss: Fall-through and let caller handle the miss by
  // entering the runtime system.
  __ bind(&miss);
}


void StubCompiler::GenerateLoadGlobalFunctionPrototype(MacroAssembler* masm,
                                                       int index,
                                                       Register prototype) {
  // Load the global or builtins object from the current context.
//  __ ldr(prototype, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ lw(prototype, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  // Load the global context from the global or builtins object.
//  __ ldr(prototype,
//         FieldMemOperand(prototype, GlobalObject::kGlobalContextOffset));
  __ lw(prototype,
         FieldMemOperand(prototype, GlobalObject::kGlobalContextOffset));
  // Load the function from the global context.
//  __ ldr(prototype, MemOperand(prototype, Context::SlotOffset(index)));
  __ lw(prototype, MemOperand(prototype, Context::SlotOffset(index)));
  // Load the initial map.  The global functions all have initial maps.
//  __ ldr(prototype,
//         FieldMemOperand(prototype, JSFunction::kPrototypeOrInitialMapOffset));
  __ lw(prototype,
         FieldMemOperand(prototype, JSFunction::kPrototypeOrInitialMapOffset));
  // Load the prototype from the initial map.
  __ lw(prototype, FieldMemOperand(prototype, Map::kPrototypeOffset));
}


// Load a fast property out of a holder object (src). In-object properties
// are loaded directly otherwise the property is loaded from the properties
// fixed array.
void StubCompiler::GenerateFastPropertyLoad(MacroAssembler* masm,
                                            Register dst, Register src,
                                            JSObject* holder, int index) {
  // Adjust for the number of properties stored in the holder.
  index -= holder->map()->inobject_properties();
  if (index < 0) {
    // Get the property straight out of the holder.
    int offset = holder->map()->instance_size() + (index * kPointerSize);
//    __ ldr(dst, FieldMemOperand(src, offset));
    __ lw(dst, FieldMemOperand(src, offset));
  } else {
    // Calculate the offset into the properties array.
    int offset = index * kPointerSize + FixedArray::kHeaderSize;
//    __ ldr(dst, FieldMemOperand(src, JSObject::kPropertiesOffset));
//    __ ldr(dst, FieldMemOperand(dst, offset));
    __ lw(dst, FieldMemOperand(src, JSObject::kPropertiesOffset));
    __ lw(dst, FieldMemOperand(dst, offset));
  }
}


void StubCompiler::GenerateLoadArrayLength(MacroAssembler* masm,
                                           Register receiver,
                                           Register scratch,
                                           Label* miss_label) {
  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, miss_label);
  __ andi(scratch, receiver, Operand(kSmiTagMask));
  __ bcond(eq, miss_label, scratch, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Check that the object is a JS array.
//  __ CompareObjectType(receiver, scratch, scratch, JS_ARRAY_TYPE);
//  __ b(ne, miss_label);
  __ GetObjectType(receiver, scratch, scratch);
  __ bcond(ne, miss_label, scratch, Operand(JS_ARRAY_TYPE));
  __ nop(); // NOP_ADDED

  // Load length directly from the JS array.
//  __ ldr(r0, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ lw(v0, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ Ret();
  __ nop(); // NOP_ADDED
}


// Generate code to check if an object is a string.  If the object is
// a string, the map's instance type is left in the scratch1 register.
//static void GenerateStringCheck(MacroAssembler* masm,
//                                Register receiver,
//                                Register scratch1,
//                                Register scratch2,
//                                Label* smi,
//                                Label* non_string_object) {
//  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, smi);
//
//  // Check that the object is a string.
//  __ ldr(scratch1, FieldMemOperand(receiver, HeapObject::kMapOffset));
//  __ ldrb(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
//  __ and_(scratch2, scratch1, Operand(kIsNotStringMask));
//  // The cast is to resolve the overload for the argument of 0x0.
//  __ cmp(scratch2, Operand(static_cast<int32_t>(kStringTag)));
//  __ b(ne, non_string_object);
//}


// Generate code to load the length from a string object and return the length.
// If the receiver object is not a string or a wrapped string object the
// execution continues at the miss label. The register containing the
// receiver is potentially clobbered.
void StubCompiler::GenerateLoadStringLength2(MacroAssembler* masm,
                                             Register receiver,
                                             Register scratch1,
                                             Register scratch2,
                                             Label* miss) {
  UNIMPLEMENTED();
  __ break_(0x212);
//  Label check_string, check_wrapper;
//
//  __ bind(&check_string);
//  // Check if the object is a string leaving the instance type in the
//  // scratch1 register.
//  GenerateStringCheck(masm, receiver, scratch1, scratch2,
//                      miss, &check_wrapper);
//
//  // Load length directly from the string.
//  __ and_(scratch1, scratch1, Operand(kStringSizeMask));
//  __ add(scratch1, scratch1, Operand(String::kHashShift));
//  __ ldr(r0, FieldMemOperand(receiver, String::kLengthOffset));
//  __ mov(r0, Operand(r0, LSR, scratch1));
//  __ mov(r0, Operand(r0, LSL, kSmiTagSize));
//  __ Ret();
//
//  // Check if the object is a JSValue wrapper.
//  __ bind(&check_wrapper);
//  __ cmp(scratch1, Operand(JS_VALUE_TYPE));
//  __ b(ne, miss);
//
//  // Unwrap the value in place and check if the wrapped value is a string.
//  __ ldr(receiver, FieldMemOperand(receiver, JSValue::kValueOffset));
//  __ b(&check_string);
}


void StubCompiler::GenerateLoadFunctionPrototype(MacroAssembler* masm,
                                                 Register receiver,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Label* miss_label) {
//  __ TryGetFunctionPrototype(receiver, scratch1, scratch2, miss_label);
//  __ mov(r0, scratch1);
//  __ Ret();
  __ TryGetFunctionPrototype(receiver, scratch1, scratch2, miss_label);
  __ mov(v0, scratch1);
  __ Ret();
  __ nop(); // NOP_ADDED
}


// Generate StoreField code, value is passed in r0 register.
// After executing generated code, the receiver_reg and name_reg
// may be clobbered.
void StubCompiler::GenerateStoreField(MacroAssembler* masm,
                                      Builtins::Name storage_extend,
                                      JSObject* object,
                                      int index,
                                      Map* transition,
                                      Register receiver_reg,
                                      Register name_reg,
                                      Register scratch,
                                      Label* miss_label) {
  // a0 : value
  Label exit;

  // Check that the receiver isn't a smi.
//  __ tst(receiver_reg, Operand(kSmiTagMask));
//  __ b(eq, miss_label);
  __ andi(t0, receiver_reg, Operand(kSmiTagMask));
  __ bcond(eq, miss_label, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Check that the map of the receiver hasn't changed.
//  __ ldr(scratch, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
//  __ cmp(scratch, Operand(Handle<Map>(object->map())));
//  __ b(ne, miss_label);
  __ lw(scratch, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
  __ bcond(ne, miss_label, scratch, Operand(Handle<Map>(object->map())));
  __ nop(); // NOP_ADDED

  // Perform global security token check if needed.
  if (object->IsJSGlobalProxy()) {
    __ CheckAccessGlobalProxy(receiver_reg, scratch, miss_label);
  }

  // Stub never generated for non-global objects that require access
  // checks.
  ASSERT(object->IsJSGlobalProxy() || !object->IsAccessCheckNeeded());

  // Perform map transition for the receiver if necessary.
  if ((transition != NULL) && (object->map()->unused_property_fields() == 0)) {
    // The properties must be extended before we can store the value.
    // We jump to a runtime call that extends the properties array.
//    __ mov(r2, Operand(Handle<Map>(transition)));
    __ li(a2, Operand(Handle<Map>(transition)));
    // Please note, if we implement keyed store for arm we need
    // to call the Builtins::KeyedStoreIC_ExtendStorage.
    Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_ExtendStorage));
    __ Jump(ic, RelocInfo::CODE_TARGET);
    __ nop(); // NOP_ADDED
    return;
  }

  if (transition != NULL) {
    // Update the map of the object; no write barrier updating is
    // needed because the map is never in new space.
//    __ mov(ip, Operand(Handle<Map>(transition)));
//    __ str(ip, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
    __ li(ip, Operand(Handle<Map>(transition)));
    __ sw(ip, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
  }

  // Adjust for the number of properties stored in the object. Even in the
  // face of a transition we can use the old map here because the size of the
  // object and the number of in-object properties is not going to change.
  index -= object->map()->inobject_properties();

  if (index < 0) {
    // Set the property straight into the object.
    int offset = object->map()->instance_size() + (index * kPointerSize);
//    __ str(r0, FieldMemOperand(receiver_reg, offset));
    __ sw(a0, FieldMemOperand(receiver_reg, offset));

    // Skip updating write barrier if storing a smi.
//    __ tst(r0, Operand(kSmiTagMask));
//    __ b(eq, &exit);
    __ andi(t0, a0, Operand(kSmiTagMask));
    __ bcond(eq, &exit, t0, Operand(zero_reg));
    __ nop();   // NOP_ADDED

    // Update the write barrier for the array address.
    // Pass the value being stored in the now unused name_reg.
//    __ mov(name_reg, Operand(offset));
    __ li(name_reg, Operand(offset));
    __ RecordWrite(receiver_reg, name_reg, scratch);
  } else {
    // Write to the properties array.
    int offset = index * kPointerSize + FixedArray::kHeaderSize;
    // Get the properties array
//    __ ldr(scratch, FieldMemOperand(receiver_reg, JSObject::kPropertiesOffset));
//    __ str(r0, FieldMemOperand(scratch, offset));
    __ lw(scratch, FieldMemOperand(receiver_reg, JSObject::kPropertiesOffset));
    __ sw(a0, FieldMemOperand(scratch, offset));

    // Skip updating write barrier if storing a smi.
//    __ tst(r0, Operand(kSmiTagMask));
//    __ b(eq, &exit);
    __ andi(t0, a0, Operand(kSmiTagMask));
    __ bcond(eq, &exit, t0, Operand(zero_reg));
    __ nop();   // NOP_ADDED

    // Update the write barrier for the array address.
    // Ok to clobber receiver_reg and name_reg, since we return.
//    __ mov(name_reg, Operand(offset));
    __ li(name_reg, Operand(offset));
    __ RecordWrite(scratch, name_reg, receiver_reg);
  }

  // Return the value (register v0).
  __ bind(&exit);
  __ mov(v0, a0);
  __ Ret();
  __ nop(); // NOP_ADDED
}


void StubCompiler::GenerateLoadMiss(MacroAssembler* masm, Code::Kind kind) {
  ASSERT(kind == Code::LOAD_IC || kind == Code::KEYED_LOAD_IC);
  Code* code = NULL;
  if (kind == Code::LOAD_IC) {
    code = Builtins::builtin(Builtins::LoadIC_Miss);
  } else {
    code = Builtins::builtin(Builtins::KeyedLoadIC_Miss);
  }

  Handle<Code> ic(code);
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop();
}


#undef __
#define __ ACCESS_MASM(masm())


Register StubCompiler::CheckPrototypes(JSObject* object,
                                       Register object_reg,
                                       JSObject* holder,
                                       Register holder_reg,
                                       Register scratch,
                                       String* name,
                                       Label* miss) {
  // Check that the maps haven't changed.
  Register result =
      masm()->CheckMaps(object, object_reg, holder, holder_reg, scratch, miss);

  // If we've skipped any global objects, it's not enough to verify
  // that their maps haven't changed.
  while (object != holder) {
    if (object->IsGlobalObject()) {
      GlobalObject* global = GlobalObject::cast(object);
      Object* probe = global->EnsurePropertyCell(name);
      if (probe->IsFailure()) {
        set_failure(Failure::cast(probe));
        return result;
      }
      JSGlobalPropertyCell* cell = JSGlobalPropertyCell::cast(probe);
      ASSERT(cell->value()->IsTheHole());
//      __ mov(scratch, Operand(Handle<Object>(cell)));
//      __ ldr(scratch,
//             FieldMemOperand(scratch, JSGlobalPropertyCell::kValueOffset));
//      __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//      __ cmp(scratch, ip);
//      __ b(ne, miss);
      __ li(scratch, Operand(Handle<Object>(cell)));
      __ lw(scratch,
             FieldMemOperand(scratch, JSGlobalPropertyCell::kValueOffset));
      __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
      __ bcond(ne, miss, scratch, Operand(ip));
      __ nop(); // NOP_ADDED
    }
    object = JSObject::cast(object->GetPrototype());
  }

  // Return the register containin the holder.
  return result;
}


void StubCompiler::GenerateLoadField(JSObject* object,
                                     JSObject* holder,
                                     Register receiver,
                                     Register scratch1,
                                     Register scratch2,
                                     int index,
                                     String* name,
                                     Label* miss) {
  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, miss);
  __ andi(t0, receiver, Operand(kSmiTagMask));
  __ bcond(eq, miss, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Check that the maps haven't changed.
  Register reg =
      CheckPrototypes(object, receiver, holder, scratch1, scratch2, name, miss);
  GenerateFastPropertyLoad(masm(), v0, reg, holder, index);
  __ Ret();
  __ nop(); // NOP_ADDED
}


void StubCompiler::GenerateLoadConstant(JSObject* object,
                                        JSObject* holder,
                                        Register receiver,
                                        Register scratch1,
                                        Register scratch2,
                                        Object* value,
                                        String* name,
                                        Label* miss) {
  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, miss);
  __ andi(t0, receiver, Operand(kSmiTagMask));
  __ bcond(eq, miss, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Check that the maps haven't changed.
  Register reg =
      CheckPrototypes(object, receiver, holder, scratch1, scratch2, name, miss);

  // Return the constant value.
//  __ mov(r0, Operand(Handle<Object>(value)));
  __ li(v0, Operand(Handle<Object>(value)));
  __ Ret();
  __ nop(); // NOP_ADDED
}


void StubCompiler::GenerateLoadCallback(JSObject* object,
                                        JSObject* holder,
                                        Register receiver,
                                        Register name_reg,
                                        Register scratch1,
                                        Register scratch2,
                                        AccessorInfo* callback,
                                        String* name,
                                        Label* miss) {
  UNIMPLEMENTED();
  __ break_(0x470);
//  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, miss);
//
//  // Check that the maps haven't changed.
//  Register reg =
//      CheckPrototypes(object, receiver, holder, scratch1, scratch2, name, miss);
//
//  // Push the arguments on the JS stack of the caller.
//  __ push(receiver);  // receiver
//  __ push(reg);  // holder
//  __ mov(ip, Operand(Handle<AccessorInfo>(callback)));  // callback data
//  __ push(ip);
//  __ ldr(reg, FieldMemOperand(ip, AccessorInfo::kDataOffset));
//  __ push(reg);
//  __ push(name_reg);  // name
//
//  // Do tail-call to the runtime system.
//  ExternalReference load_callback_property =
//      ExternalReference(IC_Utility(IC::kLoadCallbackProperty));
//  __ TailCallRuntime(load_callback_property, 5, 1);
}


void StubCompiler::GenerateLoadInterceptor(JSObject* object,
                                           JSObject* holder,
                                           LookupResult* lookup,
                                           Register receiver,
                                           Register name_reg,
                                           Register scratch1,
                                           Register scratch2,
                                           String* name,
                                           Label* miss) {
  UNIMPLEMENTED();
  __ break_(0x505);
//  // Check that the receiver isn't a smi.
//  __ tst(receiver, Operand(kSmiTagMask));
//  __ b(eq, miss);
//
//  // Check that the maps haven't changed.
//  Register reg =
//      CheckPrototypes(object, receiver, holder, scratch1, scratch2, name, miss);
//
//  // Push the arguments on the JS stack of the caller.
//  __ push(receiver);  // receiver
//  __ push(reg);  // holder
//  __ push(name_reg);  // name
//
//  InterceptorInfo* interceptor = holder->GetNamedInterceptor();
//  ASSERT(!Heap::InNewSpace(interceptor));
//  __ mov(scratch1, Operand(Handle<Object>(interceptor)));
//  __ push(scratch1);
//  __ ldr(scratch2, FieldMemOperand(scratch1, InterceptorInfo::kDataOffset));
//  __ push(scratch2);
//
//  // Do tail-call to the runtime system.
//  ExternalReference load_ic_property =
//      ExternalReference(IC_Utility(IC::kLoadPropertyWithInterceptorForLoad));
//  __ TailCallRuntime(load_ic_property, 5, 1);
}


Object* StubCompiler::CompileLazyCompile(Code::Flags flags) {
  // ----------- S t a t e -------------
  //  -- r1->a1: function
  //  -- ls->ra: return address
  // -----------------------------------

  // Enter an internal frame.
  __ EnterInternalFrame();

  // Preserve the function.
  __ push(a1);

  // Setup aligned call. arg_count=1 as we will only push the function (a1).
  __ SetupAlignedCall(t0, 1);

  // Push the function on the stack as the argument to the runtime function.
  __ push(a1);

  // Call the runtime function
  __ CallRuntime(Runtime::kLazyCompile, 1);
  __ nop(); // NOP_ADDED

  // Calculate the entry point.
  __ addiu(t9, v0, Operand(Code::kHeaderSize - kHeapObjectTag));

  __ ReturnFromAlignedCall();

  // Restore saved function. 
  __ pop(a1);

  // Tear down temporary frame.
  __ LeaveInternalFrame();

  // Do a tail-call of the compiled function.
  __ Jump(t9);
//  __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
//  __ addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);
  __ nop();

  return GetCodeWithFlags(flags, "LazyCompileStub");
}


Object* CallStubCompiler::CompileCallField(Object* object,
                                           JSObject* holder,
                                           int index,
                                           String* name) {
  // ----------- S t a t e -------------
  //  -- ra: return address
  // -----------------------------------
  Label miss;

  const int argc = arguments().immediate();

  __ break_(0x649);

  // Get the receiver of the function from the stack into a0.
//  __ ldr(r0, MemOperand(sp, argc * kPointerSize));
  __ lw(a0, MemOperand(sp, argc * kPointerSize
                            + StandardFrameConstants::kRArgsSlotsSize));
  // Check that the receiver isn't a smi.
//  __ tst(r0, Operand(kSmiTagMask));
//  __ b(eq, &miss);
  __ andi(t0, a0, Operand(kSmiTagMask));
  __ bcond(eq, &miss, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Do the right check and compute the holder register.
  Register reg =
      CheckPrototypes(JSObject::cast(object), a0, holder, a3, a2, name, &miss);
  GenerateFastPropertyLoad(masm(), a1, reg, holder, index);

  // Check that the function really is a function.
//  __ tst(r1, Operand(kSmiTagMask));
//  __ b(eq, &miss);
  __ andi(t1, a1, Operand(kSmiTagMask));
  __ bcond(eq, &miss, t1, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  // Get the map.
//  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
//  __ b(ne, &miss);
  __ GetObjectType(a1, a2, a2);
  __ bcond(ne, &miss, a2, Operand(JS_FUNCTION_TYPE));
  __ nop(); // NOP_ADDED

  // Patch the receiver on the stack with the global proxy if
  // necessary.
  if (object->IsGlobalObject()) {
//    __ ldr(r3, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));
//    __ str(r3, MemOperand(sp, argc * kPointerSize));
    __ lw(a3, FieldMemOperand(a0, GlobalObject::kGlobalReceiverOffset));
    __ break_(0x645);
    __ sw(a3, MemOperand(sp, argc * kPointerSize
                            + StandardFrameConstants::kRArgsSlotsSize));
  }

  __ break_(0x688);
  // Invoke the function.
  __ InvokeFunction(a1, arguments(), JUMP_FUNCTION);
  __ nop(); // NOP_ADDED

  // Handle call cache miss.
  __ bind(&miss);
  Handle<Code> ic = ComputeCallMiss(arguments().immediate());
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode(FIELD, name);
}


Object* CallStubCompiler::CompileCallConstant(Object* object,
                                              JSObject* holder,
                                              JSFunction* function,
                                              String* name,
                                              CheckType check) {
  // ----------- S t a t e -------------
  //  -- ra: return address
  // -----------------------------------
  Label miss;

  // Get the receiver from the stack
  const int argc = arguments().immediate();
//  __ ldr(r1, MemOperand(sp, argc * kPointerSize));
  __ lw(a1, MemOperand(sp, argc * kPointerSize
                            + StandardFrameConstants::kRArgsSlotsSize));

  // Check that the receiver isn't a smi.
  if (check != NUMBER_CHECK) {
//    __ tst(r1, Operand(kSmiTagMask));
//    __ b(eq, &miss);
    __ andi(t1, a1, Operand(kSmiTagMask));
    __ bcond(eq, &miss, t1, Operand(zero_reg));
    __ nop(); // NOP_ADDED
  }

  // Make sure that it's okay not to patch the on stack receiver
  // unless we're doing a receiver map check.
  ASSERT(!object->IsGlobalObject() || check == RECEIVER_MAP_CHECK);

  switch (check) {
    case RECEIVER_MAP_CHECK:
      // Check that the maps haven't changed.
      CheckPrototypes(JSObject::cast(object), a1, holder, a3, a2, name, &miss);

      // Patch the receiver on the stack with the global proxy if
      // necessary.
      if (object->IsGlobalObject()) {
        __ break_(0x745);
//        __ ldr(r3, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
//        __ str(r3, MemOperand(sp, argc * kPointerSize));
        __ lw(a3, FieldMemOperand(a1, GlobalObject::kGlobalReceiverOffset));
        __ sw(a3, MemOperand(sp, argc * kPointerSize
                                  + StandardFrameConstants::kRArgsSlotsSize));
      }
      break;

    case STRING_CHECK:
      // Check that the object is a two-byte string or a symbol.
//      __ CompareObjectType(r1, r2, r2, FIRST_NONSTRING_TYPE);
//      __ b(hs, &miss);
      __ GetObjectType(a1, a2, a2);
      __ bcond(Ugreater_equal, &miss, a2, Operand(FIRST_NONSTRING_TYPE));
      __ nop(); // NOP_ADDED
      // Check that the maps starting from the prototype haven't changed.
      GenerateLoadGlobalFunctionPrototype(masm(),
                                          Context::STRING_FUNCTION_INDEX,
                                          a2);
      CheckPrototypes(JSObject::cast(object->GetPrototype()), a2, holder, a3,
                      a1, name, &miss);
      break;

    case NUMBER_CHECK: {
      Label fast;
      // Check that the object is a smi or a heap number.
//      __ tst(r1, Operand(kSmiTagMask));
//      __ b(eq, &fast);
//      __ CompareObjectType(r1, r2, r2, HEAP_NUMBER_TYPE);
//      __ b(ne, &miss);
//      __ bind(&fast);
      __ andi(t1, a1, Operand(kSmiTagMask));
      __ bcond(eq, &fast, t1, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      __ GetObjectType(a1, a2, a2);
      __ bcond(ne, &miss, a2, Operand(HEAP_NUMBER_TYPE));
      __ nop(); // NOP_ADDED
      __ bind(&fast);
      // Check that the maps starting from the prototype haven't changed.
      GenerateLoadGlobalFunctionPrototype(masm(),
                                          Context::NUMBER_FUNCTION_INDEX,
                                          a2);
      CheckPrototypes(JSObject::cast(object->GetPrototype()), a2, holder, a3,
                      a1, name, &miss);
      break;
    }
//
    case BOOLEAN_CHECK: {
      Label fast;
      // Check that the object is a boolean.
//      __ LoadRoot(ip, Heap::kTrueValueRootIndex);
//      __ cmp(r1, ip);
//      __ b(eq, &fast);
//      __ LoadRoot(ip, Heap::kFalseValueRootIndex);
//      __ cmp(r1, ip);
//      __ b(ne, &miss);
//      __ bind(&fast);
      __ LoadRoot(ip, Heap::kTrueValueRootIndex);
      __ bcond(eq, &fast, a1, Operand(ip));
      __ nop(); // NOP_ADDED
      __ LoadRoot(ip, Heap::kFalseValueRootIndex);
      __ bcond(ne, &miss, a1, Operand(ip));
      __ nop(); // NOP_ADDED
      __ bind(&fast);
      // Check that the maps starting from the prototype haven't changed.
      GenerateLoadGlobalFunctionPrototype(masm(),
                                          Context::BOOLEAN_FUNCTION_INDEX,
                                          a2);
      CheckPrototypes(JSObject::cast(object->GetPrototype()), a2, holder, a3,
                      a1, name, &miss);
      break;
    }

    case JSARRAY_HAS_FAST_ELEMENTS_CHECK:
      CheckPrototypes(JSObject::cast(object), a1, holder, a3, a2, name, &miss);
      // Make sure object->HasFastElements().
      // Get the elements array of the object.
//      __ ldr(r3, FieldMemOperand(r1, JSObject::kElementsOffset));
      __ lw(a3, FieldMemOperand(a1, JSObject::kElementsOffset));
      // Check that the object is in fast mode (not dictionary).
//      __ ldr(r2, FieldMemOperand(r3, HeapObject::kMapOffset));
//      __ LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
//      __ cmp(r2, ip);
//      __ b(ne, &miss);
      __ lw(a2, FieldMemOperand(a3, HeapObject::kMapOffset));
      __ LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
      __ bcond(ne, &miss, a2, Operand(ip));
      __ nop(); // NOP_ADDED
      break;
//
    default:
      UNREACHABLE();
  }

  // Get the function and setup the context.
//  __ mov(r1, Operand(Handle<JSFunction>(function)));
//  __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
  __ li(a1, Operand(Handle<JSFunction>(function)));
  __ lw(cp, FieldMemOperand(a1, JSFunction::kContextOffset));

  // Jump to the cached code (tail call).
  ASSERT(function->is_compiled());
  Handle<Code> code(function->code());
  ParameterCount expected(function->shared()->formal_parameter_count());
  __ InvokeCode(code, expected, arguments(),
                RelocInfo::CODE_TARGET, JUMP_FUNCTION);
  __ nop(); // NOP_ADDED

  // Handle call cache miss.
  __ bind(&miss);
  Handle<Code> ic = ComputeCallMiss(arguments().immediate());
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  String* function_name = NULL;
  if (function->shared()->name()->IsString()) {
    function_name = String::cast(function->shared()->name());
  }
  return GetCode(CONSTANT_FUNCTION, function_name);
}


Object* CallStubCompiler::CompileCallInterceptor(Object* object,
                                                 JSObject* holder,
                                                 String* name) {
  UNIMPLEMENTED();
  __ break_(0x782);
//  // ----------- S t a t e -------------
//  //  -- lr: return address
//  // -----------------------------------
//  Label miss;
//
//  // TODO(1224669): Implement.
//
//  // Handle call cache miss.
//  __ bind(&miss);
//  Handle<Code> ic = ComputeCallMiss(arguments().immediate());
//  __ Jump(ic, RelocInfo::CODE_TARGET);
//  __ nop(); // NOP_ADDED
//
//  // Return the generated code.
  return GetCode(INTERCEPTOR, name);
}


Object* CallStubCompiler::CompileCallGlobal(JSObject* object,
                                            GlobalObject* holder,
                                            JSGlobalPropertyCell* cell,
                                            JSFunction* function,
                                            String* name) {
  // ----------- S t a t e -------------
  //  -- ra: return address
  // -----------------------------------
  Label miss;

  // Get the number of arguments.
  const int argc = arguments().immediate();

  // Get the receiver from the stack.
//  __ ldr(r0, MemOperand(sp, argc * kPointerSize));
//  __ lw(a0, MemOperand(sp, argc * kPointerSize));
  __ teq(fp, zero_reg, 0x111);
  __ lw(a0, MemOperand(sp, argc * kPointerSize));
//                            + StandardFrameConstants::kRArgsSlotsSize));

  // If the object is the holder then we know that it's a global
  // object which can only happen for contextual calls. In this case,
  // the receiver cannot be a smi.
  if (object != holder) {
//    __ tst(r0, Operand(kSmiTagMask));
//    __ b(eq, &miss);
    __ andi(t0, a0, Operand(kSmiTagMask));
    __ bcond(eq, &miss, t0, Operand(zero_reg));
    __ nop(); // NOP_ADDED
  }

  // Check that the maps haven't changed.
  CheckPrototypes(object, a0, holder, a3, a2, name, &miss);

  // Get the value from the cell.
//  __ mov(r3, Operand(Handle<JSGlobalPropertyCell>(cell)));
//  __ ldr(r1, FieldMemOperand(r3, JSGlobalPropertyCell::kValueOffset));
  __ li(a3, Operand(Handle<JSGlobalPropertyCell>(cell)));
  __ lw(a1, FieldMemOperand(a3, JSGlobalPropertyCell::kValueOffset));

  // Check that the cell contains the same function.
//  __ cmp(r1, Operand(Handle<JSFunction>(function)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a1, Operand(Handle<JSFunction>(function)));
  __ nop(); // NOP_ADDED

  // Patch the receiver on the stack with the global proxy if
  // necessary.
  if (object->IsGlobalObject()) {
//    __ ldr(r3, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));
//    __ str(r3, MemOperand(sp, argc * kPointerSize));
    __ lw(a3, FieldMemOperand(a0, GlobalObject::kGlobalReceiverOffset));
    __ sw(a3, MemOperand(sp, argc * kPointerSize));
//                            + StandardFrameConstants::kRArgsSlotsSize));
  }

  // Setup the context (function already in r1).
//  __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
  __ lw(cp, FieldMemOperand(a1, JSFunction::kContextOffset));

  // Jump to the cached code (tail call).
  __ IncrementCounter(&Counters::call_global_inline, 1, a2, a3);
  ASSERT(function->is_compiled());
  Handle<Code> code(function->code());
  ParameterCount expected(function->shared()->formal_parameter_count());
  __ InvokeCode(code, expected, arguments(),
                RelocInfo::CODE_TARGET, JUMP_FUNCTION);
  __ nop(); // NOP_ADDED

  // Handle call cache miss.
  __ bind(&miss);
  __ IncrementCounter(&Counters::call_global_inline_miss, 1, a1, a3);
  Handle<Code> ic = ComputeCallMiss(arguments().immediate());
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode(NORMAL, name);
}


Object* StoreStubCompiler::CompileStoreField(JSObject* object,
                                             int index,
                                             Map* transition,
                                             String* name) {
  // ----------- S t a t e -------------
  //  -- a0    : value
  //  -- a2    : name
  //  -- ra    : return address
  //  -- [sp]  : receiver
  // -----------------------------------
  Label miss;

  // Get the receiver from the stack.
//  __ ldr(r3, MemOperand(sp, 0 * kPointerSize));
  __ lw(a3, MemOperand(sp, 0 * kPointerSize));

  // name register might be clobbered.
  GenerateStoreField(masm(),
                     Builtins::StoreIC_ExtendStorage,
                     object,
                     index,
                     transition,
                     a3, a2, a1,
                     &miss);
  __ bind(&miss);
//  __ mov(r2, Operand(Handle<String>(name)));  // restore name
  __ li(a2, Operand(Handle<String>(name)));  // restore name
  Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Miss));
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode(transition == NULL ? FIELD : MAP_TRANSITION, name);
}


Object* StoreStubCompiler::CompileStoreCallback(JSObject* object,
                                                AccessorInfo* callback,
                                                String* name) {
  UNIMPLEMENTED();
  __ break_(0x906);
//  // ----------- S t a t e -------------
//  //  -- r0    : value
//  //  -- r2    : name
//  //  -- lr    : return address
//  //  -- [sp]  : receiver
//  // -----------------------------------
//  Label miss;
//
//  // Get the object from the stack.
//  __ ldr(r3, MemOperand(sp, 0 * kPointerSize));
//
//  // Check that the object isn't a smi.
//  __ tst(r3, Operand(kSmiTagMask));
//  __ b(eq, &miss);
//
//  // Check that the map of the object hasn't changed.
//  __ ldr(r1, FieldMemOperand(r3, HeapObject::kMapOffset));
//  __ cmp(r1, Operand(Handle<Map>(object->map())));
//  __ b(ne, &miss);
//
//  // Perform global security token check if needed.
//  if (object->IsJSGlobalProxy()) {
//    __ CheckAccessGlobalProxy(r3, r1, &miss);
//  }
//
//  // Stub never generated for non-global objects that require access
//  // checks.
//  ASSERT(object->IsJSGlobalProxy() || !object->IsAccessCheckNeeded());
//
//  __ ldr(ip, MemOperand(sp));  // receiver
//  __ push(ip);
//  __ mov(ip, Operand(Handle<AccessorInfo>(callback)));  // callback info
//  __ push(ip);
//  __ push(r2);  // name
//  __ push(r0);  // value
//
//  // Do tail-call to the runtime system.
//  ExternalReference store_callback_property =
//      ExternalReference(IC_Utility(IC::kStoreCallbackProperty));
//  __ TailCallRuntime(store_callback_property, 4, 1);
//
//  // Handle store cache miss.
//  __ bind(&miss);
//  __ mov(r2, Operand(Handle<String>(name)));  // restore name
//  Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Miss));
//  __ Jump(ic, RelocInfo::CODE_TARGET);
//
//  // Return the generated code.
  return GetCode(CALLBACKS, name);
}


Object* StoreStubCompiler::CompileStoreInterceptor(JSObject* receiver,
                                                   String* name) {
  UNIMPLEMENTED();
  __ break_(0x962);
//  // ----------- S t a t e -------------
//  //  -- r0    : value
//  //  -- r2    : name
//  //  -- lr    : return address
//  //  -- [sp]  : receiver
//  // -----------------------------------
//  Label miss;
//
//  // Get the object from the stack.
//  __ ldr(r3, MemOperand(sp, 0 * kPointerSize));
//
//  // Check that the object isn't a smi.
//  __ tst(r3, Operand(kSmiTagMask));
//  __ b(eq, &miss);
//
//  // Check that the map of the object hasn't changed.
//  __ ldr(r1, FieldMemOperand(r3, HeapObject::kMapOffset));
//  __ cmp(r1, Operand(Handle<Map>(receiver->map())));
//  __ b(ne, &miss);
//
//  // Perform global security token check if needed.
//  if (receiver->IsJSGlobalProxy()) {
//    __ CheckAccessGlobalProxy(r3, r1, &miss);
//  }
//
//  // Stub never generated for non-global objects that require access
//  // checks.
//  ASSERT(receiver->IsJSGlobalProxy() || !receiver->IsAccessCheckNeeded());
//
//  __ ldr(ip, MemOperand(sp));  // receiver
//  __ push(ip);
//  __ push(r2);  // name
//  __ push(r0);  // value
//
//  // Do tail-call to the runtime system.
//  ExternalReference store_ic_property =
//      ExternalReference(IC_Utility(IC::kStoreInterceptorProperty));
//  __ TailCallRuntime(store_ic_property, 3, 1);
//
//  // Handle store cache miss.
//  __ bind(&miss);
//  __ mov(r2, Operand(Handle<String>(name)));  // restore name
//  Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Miss));
//  __ Jump(ic, RelocInfo::CODE_TARGET);
//
//  // Return the generated code.
  return GetCode(INTERCEPTOR, name);
}


Object* StoreStubCompiler::CompileStoreGlobal(GlobalObject* object,
                                              JSGlobalPropertyCell* cell,
                                              String* name) {
  // ----------- S t a t e -------------
  //  -- a0    : value
  //  -- a2    : name
  //  -- ra    : return address
  //  -- [sp]  : receiver
  // -----------------------------------
  Label miss;

  // Check that the map of the global has not changed.
//  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
//  __ ldr(r3, FieldMemOperand(r1, HeapObject::kMapOffset));
//  __ cmp(r3, Operand(Handle<Map>(object->map())));
//  __ b(ne, &miss);
  __ lw(a1, MemOperand(sp, 0 * kPointerSize));
  __ lw(a3, FieldMemOperand(a1, HeapObject::kMapOffset));
  __ bcond(ne, &miss, a3, Operand(Handle<Map>(object->map())));
  __ nop(); // NOP_ADDED

  // Store the value in the cell.
//  __ mov(r2, Operand(Handle<JSGlobalPropertyCell>(cell)));
//  __ str(r0, FieldMemOperand(r2, JSGlobalPropertyCell::kValueOffset));
  __ li(a2, Operand(Handle<JSGlobalPropertyCell>(cell)));
  __ sw(a0, FieldMemOperand(a2, JSGlobalPropertyCell::kValueOffset));

  __ IncrementCounter(&Counters::named_store_global_inline, 1, a1, a3);
  __ Ret();
  __ nop(); // NOP_ADDED

  // Handle store cache miss.
  __ bind(&miss);
//  __ IncrementCounter(&Counters::named_store_global_inline_miss, 1, r1, r3);
//  Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Miss));
//  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ IncrementCounter(&Counters::named_store_global_inline_miss, 1, a1, a3);
  Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Miss));
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode(NORMAL, name);
}


Object* LoadStubCompiler::CompileLoadField(JSObject* object,
                                           JSObject* holder,
                                           int index,
                                           String* name) {
  UNIMPLEMENTED();
  __ break_(0x1063);
//  // ----------- S t a t e -------------
//  //  -- r2    : name
//  //  -- lr    : return address
//  //  -- [sp]  : receiver
//  // -----------------------------------
//  Label miss;
//
//  __ ldr(r0, MemOperand(sp, 0));
//
//  GenerateLoadField(object, holder, r0, r3, r1, index, name, &miss);
//  __ bind(&miss);
//  GenerateLoadMiss(masm(), Code::LOAD_IC);
//
//  // Return the generated code.
  return GetCode(FIELD, name);
}


Object* LoadStubCompiler::CompileLoadCallback(JSObject* object,
                                              JSObject* holder,
                                              AccessorInfo* callback,
                                              String* name) {
  UNIMPLEMENTED();
  __ break_(0x1087);
//  // ----------- S t a t e -------------
//  //  -- r2    : name
//  //  -- lr    : return address
//  //  -- [sp]  : receiver
//  // -----------------------------------
//  Label miss;
//
//  __ ldr(r0, MemOperand(sp, 0));
//  GenerateLoadCallback(object, holder, r0, r2, r3, r1, callback, name, &miss);
//  __ bind(&miss);
//  GenerateLoadMiss(masm(), Code::LOAD_IC);
//
//  // Return the generated code.
  return GetCode(CALLBACKS, name);
}


Object* LoadStubCompiler::CompileLoadConstant(JSObject* object,
                                              JSObject* holder,
                                              Object* value,
                                              String* name) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- [sp] : receiver
  // -----------------------------------
  Label miss;

//  __ ldr(r0, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, 0));

//  GenerateLoadConstant(object, holder, r0, r3, r1, value, name, &miss);
  GenerateLoadConstant(object, holder, a0, a3, a1, value, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(CONSTANT_FUNCTION, name);
}


Object* LoadStubCompiler::CompileLoadInterceptor(JSObject* object,
                                                 JSObject* holder,
                                                 String* name) {
  UNIMPLEMENTED();
  __ break_(0x1133);
//  // ----------- S t a t e -------------
//  //  -- r2    : name
//  //  -- lr    : return address
//  //  -- [sp]  : receiver
//  // -----------------------------------
//  Label miss;
//
//  __ ldr(r0, MemOperand(sp, 0));
//
//  LookupResult lookup;
//  holder->LocalLookupRealNamedProperty(name, &lookup);
//  GenerateLoadInterceptor(object,
//                          holder,
//                          &lookup,
//                          r0,
//                          r2,
//                          r3,
//                          r1,
//                          name,
//                          &miss);
//  __ bind(&miss);
//  GenerateLoadMiss(masm(), Code::LOAD_IC);
//
//  // Return the generated code.
  return GetCode(INTERCEPTOR, name);
}


Object* LoadStubCompiler::CompileLoadGlobal(JSObject* object,
                                            GlobalObject* holder,
                                            JSGlobalPropertyCell* cell,
                                            String* name,
                                            bool is_dont_delete) {
  // ----------- S t a t e -------------
  //  -- a2    : name
  //  -- ra    : return address
  //  -- [sp]  : receiver
  // -----------------------------------
  Label miss;

  // Get the receiver from the stack.
//  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
  __ lw(a1, MemOperand(sp, 0 * kPointerSize));

  // If the object is the holder then we know that it's a global
  // object which can only happen for contextual calls. In this case,
  // the receiver cannot be a smi.
  if (object != holder) {
//    __ tst(r1, Operand(kSmiTagMask));
//    __ b(eq, &miss);
    __ andi(t0, a1, Operand(kSmiTagMask));
    __ bcond(eq, &miss, t0, Operand(zero_reg));
    __ nop();
  }

  // Check that the map of the global has not changed.
  CheckPrototypes(object, a1, holder, a3, a0, name, &miss);

  // Get the value from the cell.
//  __ mov(r3, Operand(Handle<JSGlobalPropertyCell>(cell)));
//  __ ldr(r0, FieldMemOperand(r3, JSGlobalPropertyCell::kValueOffset));
  __ li(a3, Operand(Handle<JSGlobalPropertyCell>(cell)));
  __ lw(v0, FieldMemOperand(a3, JSGlobalPropertyCell::kValueOffset));

  // Check for deleted property if property can actually be deleted.
  if (!is_dont_delete) {
//    __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//    __ cmp(r0, ip);
//    __ b(eq, &miss);
    __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
    __ bcond(eq, &miss, v0, Operand(ip));
    __ nop(); // NOP_ADDED
  }

//  __ IncrementCounter(&Counters::named_load_global_inline, 1, r1, r3);
  __ IncrementCounter(&Counters::named_load_global_inline, 1, a1, a3);
  __ Ret();
  __ nop(); // NOP_ADDED

  __ bind(&miss);
//  __ IncrementCounter(&Counters::named_load_global_inline_miss, 1, r1, r3);
  __ IncrementCounter(&Counters::named_load_global_inline_miss, 1, a1, a3);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(NORMAL, name);
}


Object* KeyedLoadStubCompiler::CompileLoadField(String* name,
                                                JSObject* receiver,
                                                JSObject* holder,
                                                int index) {
  // ----------- S t a t e -------------
  //  -- ra    : return address
  //  -- sp[0] : key
  //  -- sp[4] : receiver
  // -----------------------------------
  Label miss;

//  __ ldr(r2, MemOperand(sp, 0));
//  __ ldr(r0, MemOperand(sp, kPointerSize));
  __ lw(a2, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, kPointerSize));

//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

//  GenerateLoadField(receiver, holder, r0, r3, r1, index, name, &miss);
  GenerateLoadField(receiver, holder, a0, a3, a1, index, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  return GetCode(FIELD, name);
}


Object* KeyedLoadStubCompiler::CompileLoadCallback(String* name,
                                                   JSObject* receiver,
                                                   JSObject* holder,
                                                   AccessorInfo* callback) {
  // ----------- S t a t e -------------
  //  -- ra    : return address
  //  -- sp[0] : key
  //  -- sp[4] : receiver
  // -----------------------------------
  Label miss;

//  __ ldr(r2, MemOperand(sp, 0));
//  __ ldr(r0, MemOperand(sp, kPointerSize));
  __ lw(a2, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, kPointerSize));

//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

//  GenerateLoadCallback(receiver, holder, r0, r2, r3, r1, callback, name, &miss);
  GenerateLoadCallback(receiver, holder, a0, a2, a3, a1, callback, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  return GetCode(CALLBACKS, name);
}


Object* KeyedLoadStubCompiler::CompileLoadConstant(String* name,
                                                   JSObject* receiver,
                                                   JSObject* holder,
                                                   Object* value) {
  // ----------- S t a t e -------------
  //  -- ra    : return address
  //  -- sp[0] : key
  //  -- sp[4] : receiver
  // -----------------------------------
  Label miss;

  // Check the key is the cached one
//  __ ldr(r2, MemOperand(sp, 0));
//  __ ldr(r0, MemOperand(sp, kPointerSize));
  __ lw(a2, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, kPointerSize));

//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

//  GenerateLoadConstant(receiver, holder, r0, r3, r1, value, name, &miss);
  GenerateLoadConstant(receiver, holder, a0, a3, a1, value, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  // Return the generated code.
  return GetCode(CONSTANT_FUNCTION, name);
}


Object* KeyedLoadStubCompiler::CompileLoadInterceptor(JSObject* receiver,
                                                      JSObject* holder,
                                                      String* name) {
  // ----------- S t a t e -------------
  //  -- ra    : return address
  //  -- sp[0] : key
  //  -- sp[4] : receiver
  // -----------------------------------
  Label miss;

  // Check the key is the cached one
//  __ ldr(r2, MemOperand(sp, 0));
//  __ ldr(r0, MemOperand(sp, kPointerSize));
  __ lw(a2, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, kPointerSize));

//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

  LookupResult lookup;
  holder->LocalLookupRealNamedProperty(name, &lookup);
  GenerateLoadInterceptor(receiver,
                          holder,
                          &lookup,
                          a0,
                          a2,
                          a3,
                          a1,
                          name,
                          &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  return GetCode(INTERCEPTOR, name);
}


Object* KeyedLoadStubCompiler::CompileLoadArrayLength(String* name) {
  // ----------- S t a t e -------------
  //  -- ra    : return address
  //  -- sp[0] : key
  //  -- sp[4] : receiver
  // -----------------------------------
  Label miss;

  // Check the key is the cached one
//  __ ldr(r2, MemOperand(sp, 0));
//  __ ldr(r0, MemOperand(sp, kPointerSize));
  __ lw(a2, MemOperand(sp, 0));
  __ lw(a0, MemOperand(sp, kPointerSize));

//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

  GenerateLoadArrayLength(masm(), a0, a3, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  return GetCode(CALLBACKS, name);
}


Object* KeyedLoadStubCompiler::CompileLoadStringLength(String* name) {
  UNIMPLEMENTED();
  __ break_(0x1370);
//  // ----------- S t a t e -------------
//  //  -- lr    : return address
//  //  -- sp[0] : key
//  //  -- sp[4] : receiver
//  // -----------------------------------
//  Label miss;
//  __ IncrementCounter(&Counters::keyed_load_string_length, 1, r1, r3);
//
//  __ ldr(r2, MemOperand(sp));
//  __ ldr(r0, MemOperand(sp, kPointerSize));  // receiver
//
//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
//
//  GenerateLoadStringLength2(masm(), r0, r1, r3, &miss);
//  __ bind(&miss);
//  __ DecrementCounter(&Counters::keyed_load_string_length, 1, r1, r3);
//
//  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);
//
  return GetCode(CALLBACKS, name);
}


// TODO(1224671): implement the fast case.
Object* KeyedLoadStubCompiler::CompileLoadFunctionPrototype(String* name) {
  UNIMPLEMENTED();
  __ break_(0x1398);
//  // ----------- S t a t e -------------
//  //  -- lr    : return address
//  //  -- sp[0] : key
//  //  -- sp[4] : receiver
//  // -----------------------------------
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  return GetCode(CALLBACKS, name);
}


Object* KeyedStoreStubCompiler::CompileStoreField(JSObject* object,
                                                  int index,
                                                  Map* transition,
                                                  String* name) {
  UNIMPLEMENTED();
  __ break_(0x1415);
  // ----------- S t a t e -------------
  //  -- a0    : value
  //  -- a2    : name
  //  -- ra    : return address
  //  -- [sp]  : receiver
  // -----------------------------------
  Label miss;

  __ IncrementCounter(&Counters::keyed_store_field, 1, a1, a3);

  // Check that the name has not changed.
//  __ cmp(r2, Operand(Handle<String>(name)));
//  __ b(ne, &miss);
  __ bcond(ne, &miss, a2, Operand(Handle<String>(name)));
  __ nop(); // NOP_ADDED

  // Load receiver from the stack.
//  __ ldr(r3, MemOperand(sp));
  __ lw(a3, MemOperand(sp));
  // a1 is used as scratch register, a3 and a2 might be clobbered.
  GenerateStoreField(masm(),
                     Builtins::StoreIC_ExtendStorage,
                     object,
                     index,
                     transition,
                     a3, a2, a1,
                     &miss);
  __ bind(&miss);

  __ DecrementCounter(&Counters::keyed_store_field, 1, a1, a3);
//  __ mov(r2, Operand(Handle<String>(name)));  // restore name register.
  __ li(a2, Operand(Handle<String>(name)));  // restore name register.
  Handle<Code> ic(Builtins::builtin(Builtins::KeyedStoreIC_Miss));
  __ Jump(ic, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode(transition == NULL ? FIELD : MAP_TRANSITION, name);
}


Object* ConstructStubCompiler::CompileConstructStub(
    SharedFunctionInfo* shared) {
#ifdef DEBUG
//  printf("ConstructStubCompiler::CompileConstructStub\n");
#endif
  // ----------- S t a t e -------------
  //  -- a0    : argc
  //  -- a1    : constructor
  //  -- ra    : return address
  //  -- [sp]  : last argument
  // -----------------------------------
  Label generic_stub_call;

  // Use t7 for holding undefined which is used in several places below.
  __ LoadRoot(t7, Heap::kUndefinedValueRootIndex);

#ifdef ENABLE_DEBUGGER_SUPPORT
  // Check to see whether there are any break points in the function code. If
  // there are jump to the generic constructor stub which calls the actual
  // code for the function thereby hitting the break points.
//  __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
//  __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kDebugInfoOffset));
//  __ cmp(r2, r7);
//  __ b(ne, &generic_stub_call);
  __ lw(t5, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  __ lw(a2, FieldMemOperand(t5, SharedFunctionInfo::kDebugInfoOffset));
  __ bcond(ne, &generic_stub_call, a2, Operand(t7));
  __ nop(); // NOP_ADDED
#endif

  // Load the initial map and verify that it is in fact a map.
  // a1: constructor function
  // t7: undefined
//  __ ldr(r2, FieldMemOperand(r1, JSFunction::kPrototypeOrInitialMapOffset));
//  __ tst(r2, Operand(kSmiTagMask));
//  __ b(eq, &generic_stub_call);
//  __ CompareObjectType(r2, r3, r4, MAP_TYPE);
//  __ b(ne, &generic_stub_call);
  __ lw(a2, FieldMemOperand(a1, JSFunction::kPrototypeOrInitialMapOffset));
  __ andi(t0, a2, Operand(kSmiTagMask));
  __ bcond(eq, &generic_stub_call, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  __ GetObjectType(a2, a3, t0);
  __ bcond(ne, &generic_stub_call, t0, Operand(MAP_TYPE));
  __ nop(); // NOP_ADDED

#ifdef DEBUG
  // Cannot construct functions this way.
  // a0: argc
  // a1: constructor function
  // a2: initial map
  // t7: undefined
//  __ CompareInstanceType(r2, r3, JS_FUNCTION_TYPE);
//  __ Check(ne, "Function constructed by construct stub.");
  __ lbu(a3, FieldMemOperand(a2, Map::kInstanceTypeOffset));
  __ Check(ne, "Function constructed by construct stub.", a3, Operand(JS_FUNCTION_TYPE));
#endif

  // Now allocate the JSObject in new space.
  // a0: argc
  // a1: constructor function
  // a2: initial map
  // t7: undefined
//  __ ldrb(r3, FieldMemOperand(r2, Map::kInstanceSizeOffset));
  __ lbu(a3, FieldMemOperand(a2, Map::kInstanceSizeOffset));
  __ AllocateInNewSpace(a3,
                        t4,
                        t5,
                        t6,
                        &generic_stub_call,
                        NO_ALLOCATION_FLAGS);

  // Allocated the JSObject, now initialize the fields. Map is set to initial
  // map and properties and elements are set to empty fixed array.
  // a0: argc
  // a1: constructor function
  // a2: initial map
  // a3: object size (in words)
  // t4: JSObject (not tagged)
  // r7: undefined
//  __ LoadRoot(r6, Heap::kEmptyFixedArrayRootIndex);
//  __ mov(r5, r4);
//  ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
//  __ str(r2, MemOperand(r5, kPointerSize, PostIndex));
//  ASSERT_EQ(1 * kPointerSize, JSObject::kPropertiesOffset);
//  __ str(r6, MemOperand(r5, kPointerSize, PostIndex));
//  ASSERT_EQ(2 * kPointerSize, JSObject::kElementsOffset);
//  __ str(r6, MemOperand(r5, kPointerSize, PostIndex));
  __ LoadRoot(t6, Heap::kEmptyFixedArrayRootIndex);
  __ mov(t5, t4);
  __ sw(a2, MemOperand(t5, JSObject::kMapOffset));
  __ sw(t6, MemOperand(t5, JSObject::kPropertiesOffset));
  __ sw(t6, MemOperand(t5, JSObject::kElementsOffset));
  ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
  ASSERT_EQ(1 * kPointerSize, JSObject::kPropertiesOffset);
  ASSERT_EQ(2 * kPointerSize, JSObject::kElementsOffset);

  __ addiu(t5, t5, Operand(3 * kPointerSize));

  // Calculate the location of the first argument. The stack contains only the
  // argc arguments.
//  __ add(r1, sp, Operand(r0, LSL, kPointerSizeLog2));
  __ sll(a1, a0, kPointerSizeLog2);
  __ addu(a1, a1, Operand(sp));

  // We need to add the 4 args slots size because they need to be setup when we
  // call the real time function the first time this kind of object is
  // initialized (cf Builtins::Generate_JSConstructStubGeneric).
  // TOCHECK: This need is maybe just because I first implemented it with args
  // slots. Try to do it without: we should not need this as the real time
  // function called has the stack setup just before it is called.
  __ addiu(a1, a1, StandardFrameConstants::kRArgsSlotsSize);

  // Fill all the in-object properties with undefined.
  // a0: argc
  // a1: first argument
  // a3: object size (in words)
  // t4: JSObject (not tagged)
  // t5: First in-object property of JSObject (not tagged)
  // t7: undefined
  // Fill the initialized properties with a constant value or a passed argument
  // depending on the this.x = ...; assignment in the function.
  for (int i = 0; i < shared->this_property_assignments_count(); i++) {
    if (shared->IsThisPropertyAssignmentArgument(i)) {
      Label not_passed, next;
      // Check if the argument assigned to the property is actually passed.
      int arg_number = shared->GetThisPropertyAssignmentArgument(i);
//      __ cmp(r0, Operand(arg_number));
//      __ b(le, &not_passed);
      __ bcond(less_equal, &not_passed, a0, Operand(arg_number));
      __ nop(); // NOP_ADDED
      // Argument passed - find it on the stack.
//      __ ldr(r2, MemOperand(r1, (arg_number + 1) * -kPointerSize));
//      __ str(r2, MemOperand(r5, kPointerSize, PostIndex));
//      __ b(&next);
//      __ bind(&not_passed);
      __ lw(a2, MemOperand(a1, (arg_number + 1) * -kPointerSize));
      __ sw(a2, MemOperand(t5));
      __ addiu(t5, t5, Operand(kPointerSize));
      __ b(&next);
      __ nop(); // NOP_ADDED
      __ bind(&not_passed);
      // Set the property to undefined.
//      __ str(r7, MemOperand(r5, kPointerSize, PostIndex));
      __ sw(t7, MemOperand(t5));
      __ addiu(t5, t5, Operand(kPointerSize));
      __ bind(&next);
    } else {
      // Set the property to the constant value.
      Handle<Object> constant(shared->GetThisPropertyAssignmentConstant(i));
//      __ mov(r2, Operand(constant));
//      __ str(r2, MemOperand(r5, kPointerSize, PostIndex));
      __ li(a2, Operand(constant));
      __ sw(a2, MemOperand(t5));
      __ addiu(t5, t5, Operand(kPointerSize));
    }
  }

  // Fill the unused in-object property fields with undefined.
  for (int i = shared->this_property_assignments_count();
       i < shared->CalculateInObjectProperties();
       i++) {
//      __ str(r7, MemOperand(r5, kPointerSize, PostIndex));
      __ sw(t7, MemOperand(t5));
      __ addiu(t5, t5, Operand(kPointerSize));
  }

  // a0: argc
  // t4: JSObject (not tagged)
  // Move argc to a1 and the JSObject to return to v0 and tag it.
//  __ mov(r1, r0);
//  __ mov(r0, r4);
//  __ orr(r0, r0, Operand(kHeapObjectTag));
  __ mov(a1, a0);
  __ mov(v0, t4);
  __ or_(v0, v0, Operand(kHeapObjectTag));

  // v0: JSObject
  // a1: argc
  // Remove caller arguments and receiver from the stack and return.
//  __ add(sp, sp, Operand(r1, LSL, kPointerSizeLog2));
//  __ add(sp, sp, Operand(kPointerSize));
//  __ IncrementCounter(&Counters::constructed_objects, 1, r1, r2);
//  __ IncrementCounter(&Counters::constructed_objects_stub, 1, r1, r2);
//  __ Jump(lr);
  __ sll(t0, a1, kPointerSizeLog2);
  __ add(sp, sp, Operand(t0));
  __ addi(sp, sp, Operand(kPointerSize));
  __ IncrementCounter(&Counters::constructed_objects, 1, a1, a2);
  __ IncrementCounter(&Counters::constructed_objects_stub, 1, a1, a2);
  __ Jump(ra);
  __ nop(); // NOP_ADDED

  // Jump to the generic stub in case the specialized code cannot handle the
  // construction.
  __ bind(&generic_stub_call);
  Code* code = Builtins::builtin(Builtins::JSConstructStubGeneric);
  Handle<Code> generic_construct_stub(code);
  __ Jump(generic_construct_stub, RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED

  // Return the generated code.
  return GetCode();
}


#undef __

} }  // namespace v8::internal
