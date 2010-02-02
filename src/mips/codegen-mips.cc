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
#include "parser.h"
#include "register-allocator-inl.h"
#include "runtime.h"
#include "scopes.h"
#include "compiler.h"



namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm_)

static void EmitIdenticalObjectComparison(MacroAssembler* masm,
                                          Label* slow,
                                          Condition cc);
static void EmitSmiNonsmiComparison(MacroAssembler* masm,
                                    Label* rhs_not_nan,
                                    Label* slow,
                                    bool strict);
static void EmitTwoNonNanDoubleComparison(MacroAssembler* masm, Condition cc);
static void EmitStrictTwoHeapObjectCompare(MacroAssembler* masm);
static void MultiplyByKnownInt(MacroAssembler* masm,
                               Register source,
                               Register destination,
                               int known_int);
static bool IsEasyToMultiplyBy(int x);



// -------------------------------------------------------------------------
// Platform-specific DeferredCode functions.


void DeferredCode::SaveRegisters() {
  for (int i = 0; i < RegisterAllocator::kNumRegisters; i++) {
    int action = registers_[i];
    if (action == kPush) {
      __ push(RegisterAllocator::ToRegister(i));
    } else if (action != kIgnore && (action & kSyncedFlag) == 0) {
      __ sw(RegisterAllocator::ToRegister(i), MemOperand(fp, action));
    }
  }
}


void DeferredCode::RestoreRegisters() {
  // Restore registers in reverse order due to the stack.
  for (int i = RegisterAllocator::kNumRegisters - 1; i >= 0; i--) {
    int action = registers_[i];
    if (action == kPush) {
      __ pop(RegisterAllocator::ToRegister(i));
    } else if (action != kIgnore) {
      action &= ~kSyncedFlag;
      __ lw(RegisterAllocator::ToRegister(i), MemOperand(fp, action));
    }
  }
}


// -------------------------------------------------------------------------
// CodeGenState implementation.

CodeGenState::CodeGenState(CodeGenerator* owner)
    : owner_(owner),
      true_target_(NULL),
      false_target_(NULL),
      previous_(NULL) {
  owner_->set_state(this);
}


CodeGenState::CodeGenState(CodeGenerator* owner,
                           JumpTarget* true_target,
                           JumpTarget* false_target)
    : owner_(owner),
      true_target_(true_target),
      false_target_(false_target),
      previous_(owner->state()) {
  owner_->set_state(this);
}


CodeGenState::~CodeGenState() {
  ASSERT(owner_->state() == this);
  owner_->set_state(previous_);
}


// -------------------------------------------------------------------------
// CodeGenerator implementation

CodeGenerator::CodeGenerator(int buffer_size, Handle<Script> script,
                             bool is_eval)
    : is_eval_(is_eval),
      script_(script),
      deferred_(8),
      masm_(new MacroAssembler(NULL, buffer_size)),
      scope_(NULL),
      frame_(NULL),
      allocator_(NULL),
      cc_reg_(cc_always),
      state_(NULL),
      function_return_is_shadowed_(false) {
}


// Calling conventions:
// s8_fp: caller's frame pointer
// sp: stack pointer
// a1: called JS function
// cp: callee's context

void CodeGenerator::GenCode(FunctionLiteral* fun) {
  // Record the position for debugging purposes.
  CodeForFunctionPosition(fun);

  ZoneList<Statement*>* body = fun->body();

  // Initialize state.
  ASSERT(scope_ == NULL);
  scope_ = fun->scope();
  ASSERT(allocator_ == NULL);
  RegisterAllocator register_allocator(this);
  allocator_ = &register_allocator;
  ASSERT(frame_ == NULL);
  frame_ = new VirtualFrame();
  cc_reg_ = cc_always;
  {
    CodeGenState state(this);


    // Entry:
    // Stack: receiver, arguments
    // ra: return address
    // fp: caller's frame pointer
    // sp: stack pointer
    // a1: called JS function
    // cp: callee's context
    allocator_->Initialize();
    frame_->Enter();

#ifdef DEBUG
    if (strlen(FLAG_stop_at) > 0 &&
        fun->name()->IsEqualTo(CStrVector(FLAG_stop_at))) {
      frame_->SpillAll();
      __ stop("stop-at");
    }
#endif

    // Allocate space for locals and initialize them.  This also checks
    // for stack overflow.
    frame_->AllocateStackSlots();
    // Initialize the function return target after the locals are set
    // up, because it needs the expected frame height from the frame.
    function_return_.set_direction(JumpTarget::BIDIRECTIONAL);
    function_return_is_shadowed_ = false;

    VirtualFrame::SpilledScope spilled_scope;
    if (scope_->num_heap_slots() > 0) {
#ifdef DEBUG
//      printf("%s - %d - %s: if (scope_->num_heap_slots() > 0)\n", __FILE__, __LINE__, __func__);
#endif

      // Allocate local context.
      // Get outer context and create a new context based on it.
      __ lw(a0, frame_->Function());
      frame_->EmitPush(a0);
      frame_->CallRuntime(Runtime::kNewContext, 1);  // v0 holds the result
      __ nop(); // NOP_ADDED

#ifdef DEBUG
      JumpTarget verified_true;
      verified_true.Branch(eq, no_hint, v0, Operand(cp));
      __ nop(); // NOP_ADDED
      __ stop("NewContext: v0 is expected to be the same as cp");
      verified_true.Bind();
#endif
      // Update context local.
      __ sw(cp, frame_->Context());
    }

    // TODO(1241774): Improve this code:
    // 1) only needed if we have a context
    // 2) no need to recompute context ptr every single time
    // 3) don't copy parameter operand code from SlotOperand!
    {
      Comment cmnt2(masm_, "[ copy context parameters into .context");

      // Note that iteration order is relevant here! If we have the same
      // parameter twice (e.g., function (x, y, x)), and that parameter
      // needs to be copied into the context, it must be the last argument
      // passed to the parameter that needs to be copied. This is a rare
      // case so we don't check for it, instead we rely on the copying
      // order: such a parameter is copied repeatedly into the same
      // context location and thus the last value is what is seen inside
      // the function.
      for (int i = 0; i < scope_->num_parameters(); i++) {
        Variable* par = scope_->parameter(i);
        Slot* slot = par->slot();
        if (slot != NULL && slot->type() == Slot::CONTEXT) {
          ASSERT(!scope_->is_global_scope());  // no parameters in global scope
          __ lw(a1, frame_->ParameterAt(i));
          // Loads r2 with context; used below in RecordWrite.
          __ sw(a1, SlotOperand(slot, a2));
          // Load the offset into r3.
          int slot_offset =
              FixedArray::kHeaderSize + slot->index() * kPointerSize;
          __ li(a3, Operand(slot_offset));
          __ RecordWrite(a2, a3, a1);
        }
      }
    }

    // Store the arguments object.  This must happen after context
    // initialization because the arguments object may be stored in the
    // context.
    if (scope_->arguments() != NULL) {
#ifdef DEBUG
//      printf("%s - %d - %s: if (scope_->arguments() != NULL) ", __FILE__, __LINE__, __func__);
#endif

      ASSERT(scope_->arguments_shadow() != NULL);
      Comment cmnt(masm_, "[ allocate arguments object");
      { Reference shadow_ref(this, scope_->arguments_shadow());
        { Reference arguments_ref(this, scope_->arguments());
          ArgumentsAccessStub stub(ArgumentsAccessStub::NEW_OBJECT);
          __ lw(a2, frame_->Function());
          // The receiver is below the arguments (and args slots), the return address,
          // and the frame pointer on the stack.
          const int kReceiverDisplacement = 2 + 4 + scope_->num_parameters();
          __ addiu(a1, fp, Operand(kReceiverDisplacement * kPointerSize));
          __ li(a0, Operand(Smi::FromInt(scope_->num_parameters())));
          frame_->Adjust(3);
          __ multi_push_reversed(a0.bit() | a1.bit() | a2.bit());
          frame_->CallStub(&stub, 3);
          __ nop(); // NOP_ADDED
          frame_->EmitPush(v0);
          arguments_ref.SetValue(NOT_CONST_INIT);
        }
        shadow_ref.SetValue(NOT_CONST_INIT);
      }
      frame_->Drop();  // Value is no longer needed.
    }

    // Generate code to 'execute' declarations and initialize functions
    // (source elements). In case of an illegal redeclaration we need to
    // handle that instead of processing the declarations.
    if (scope_->HasIllegalRedeclaration()) {
      Comment cmnt(masm_, "[ illegal redeclarations");
      scope_->VisitIllegalRedeclaration(this);
    } else {
      Comment cmnt(masm_, "[ declarations");
      ProcessDeclarations(scope_->declarations());
      // Bail out if a stack-overflow exception occurred when processing
      // declarations.
      if (HasStackOverflow()) return;
    }

    if (FLAG_trace) {
      frame_->CallRuntime(Runtime::kTraceEnter, 0);
      __ nop(); // NOP_ADDED
      // Ignore the return value.
    }

    // Compile the body of the function in a vanilla state. Don't
    // bother compiling all the code if the scope has an illegal
    // redeclaration.
    if (!scope_->HasIllegalRedeclaration()) {
      Comment cmnt(masm_, "[ function body");
#ifdef DEBUG
      bool is_builtin = Bootstrapper::IsActive();
      bool should_trace =
          is_builtin ? FLAG_trace_builtin_calls : FLAG_trace_calls;
      if (should_trace) {
        frame_->CallRuntime(Runtime::kDebugTrace, 0);
        __ nop(); // NOP_ADDED
        // Ignore the return value.
      }
#endif
#ifdef DEBUG
//      printf("VisitStatementsAndSpill(body)\n");
#endif
      VisitStatementsAndSpill(body);
    }
  }

  // Generate the return sequence if necessary.
  if (has_valid_frame() || function_return_.is_linked()) {
    if (!function_return_.is_linked()) {
      CodeForReturnPosition(fun);
    }
    // exit
    // v0: result
    // sp: stack pointer
    // fp: frame pointer
    // cp: callee's context
    __ LoadRoot(v0, Heap::kUndefinedValueRootIndex);

    function_return_.Bind();
    if (FLAG_trace) {
      // Push the return value on the stack as the parameter.
      // Runtime::TraceExit returns the parameter as it is.
      frame_->EmitPush(v0);
      frame_->CallRuntime(Runtime::kTraceExit, 1);
      __ nop(); // NOP_ADDED
    }

    // Add a label for checking the size of the code used for returning.
    Label check_exit_codesize;
    masm_->bind(&check_exit_codesize);

    // Tear down the frame which will restore the caller's frame pointer and
    // the link register.
    frame_->Exit();

    // Here we use masm_-> instead of the __ macro to avoid the code coverage
    // tool from instrumenting as we rely on the code size here.
    masm_->addiu(sp, sp, Operand((scope_->num_parameters() + 1)*kPointerSize));
//                                  + StandardFrameConstants::kRegularArgsSlotsSize));
    masm_->Jump(ra);

    // Check that the size of the code used for returning matches what is
    // expected by the debugger.
    ASSERT_EQ(kJSReturnSequenceLength,
              masm_->InstructionsGeneratedSince(&check_exit_codesize));
    __ nop();
  }

  // Code generation state must be reset.
  ASSERT(!has_cc());
  ASSERT(state_ == NULL);
  ASSERT(!function_return_is_shadowed_);
  function_return_.Unuse();
  DeleteFrame();

  // Process any deferred code using the register allocator.
  if (!HasStackOverflow()) {
    ProcessDeferred();
  }

  allocator_ = NULL;
  scope_ = NULL;
}


MemOperand CodeGenerator::SlotOperand(Slot* slot, Register tmp) {
  // Currently, this assertion will fail if we try to assign to
  // a constant variable that is constant because it is read-only
  // (such as the variable referring to a named function expression).
  // We need to implement assignments to read-only variables.
  // Ideally, we should do this during AST generation (by converting
  // such assignments into expression statements); however, in general
  // we may not be able to make the decision until past AST generation,
  // that is when the entire program is known.
  ASSERT(slot != NULL);
  int index = slot->index();
  switch (slot->type()) {
    case Slot::PARAMETER:
      return frame_->ParameterAt(index);

    case Slot::LOCAL:
      return frame_->LocalAt(index);

    case Slot::CONTEXT: {
#ifdef DEBUG
//    printf("case Slot::CONTEXT: \n");
#endif

//      // Follow the context chain if necessary.
      ASSERT(!tmp.is(cp));  // do not overwrite context register
      Register context = cp;
      int chain_length = scope()->ContextChainLength(slot->var()->scope());
      for (int i = 0; i < chain_length; i++) {
        // Load the closure.
        // (All contexts, even 'with' contexts, have a closure,
        // and it is the same for all contexts inside a function.
        // There is no need to go to the function context first.)
//        __ ldr(tmp, ContextOperand(context, Context::CLOSURE_INDEX));
        __ lw(tmp, ContextOperand(context, Context::CLOSURE_INDEX));
        // Load the function context (which is the incoming, outer context).
//        __ ldr(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
        __ lw(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
        context = tmp;
      }
      // We may have a 'with' context now. Get the function context.
      // (In fact this mov may never be the needed, since the scope analysis
      // may not permit a direct context access in this case and thus we are
      // always at a function context. However it is safe to dereference be-
      // cause the function context of a function context is itself. Before
      // deleting this mov we should try to create a counter-example first,
      // though...)
//      __ ldr(tmp, ContextOperand(context, Context::FCONTEXT_INDEX));
      __ lw(tmp, ContextOperand(context, Context::FCONTEXT_INDEX));
      return ContextOperand(tmp, index);
    }

    default:
      UNREACHABLE();
      return MemOperand(no_reg, 0);
  }
}


MemOperand CodeGenerator::ContextSlotOperandCheckExtensions(
    Slot* slot,
    Register tmp,
    Register tmp2,
    JumpTarget* slow) {
  ASSERT(slot->type() == Slot::CONTEXT);
  Register context = cp;

  UNIMPLEMENTED();
  __ break_(0x00666);

  for (Scope* s = scope(); s != slot->var()->scope(); s = s->outer_scope()) {
    if (s->num_heap_slots() > 0) {
      if (s->calls_eval()) {
        // Check that extension is NULL.
//        __ ldr(tmp2, ContextOperand(context, Context::EXTENSION_INDEX));
//        __ tst(tmp2, tmp2);
        __ lw(tmp2, ContextOperand(context, Context::EXTENSION_INDEX));
        slow->Branch(ne, no_hint, tmp2, Operand(zero_reg));
        __ nop(); // NOP_ADDED
      }
//      __ ldr(tmp, ContextOperand(context, Context::CLOSURE_INDEX));
//      __ ldr(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
      __ lw(tmp, ContextOperand(context, Context::CLOSURE_INDEX));
      __ lw(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
      context = tmp;
    }
  }
  // Check that last extension is NULL.
//  __ ldr(tmp2, ContextOperand(context, Context::EXTENSION_INDEX));
//  __ tst(tmp2, tmp2);
  __ lw(tmp2, ContextOperand(context, Context::EXTENSION_INDEX));
  slow->Branch(ne, no_hint, tmp2, Operand(zero_reg));
//  __ ldr(tmp, ContextOperand(context, Context::FCONTEXT_INDEX));
  __ lw(tmp, ContextOperand(context, Context::FCONTEXT_INDEX));
  return ContextOperand(tmp, slot->index());
}


// Loads a value on TOS. If it is a boolean value, the result may have been
// (partially) translated into branches, or it may have set the condition
// code register. If force_cc is set, the value is forced to set the
// condition code register and no value is pushed. If the condition code
// register was set, has_cc() is true and cc_reg_ contains the condition to
// test for 'true'.
void CodeGenerator::LoadCondition(Expression* x,
                                  JumpTarget* true_target,
                                  JumpTarget* false_target,
                                  bool force_cc) {
  ASSERT(!has_cc());
  int original_height = frame_->height();

  { CodeGenState new_state(this, true_target, false_target);
    Visit(x);

    // If we hit a stack overflow, we may not have actually visited
    // the expression.  In that case, we ensure that we have a
    // valid-looking frame state because we will continue to generate
    // code as we unwind the C++ stack.
    //
    // It's possible to have both a stack overflow and a valid frame
    // state (eg, a subexpression overflowed, visiting it returned
    // with a dummied frame state, and visiting this expression
    // returned with a normal-looking state).
    if (HasStackOverflow() &&
        has_valid_frame() &&
        !has_cc() &&
        frame_->height() == original_height) {
      true_target->Jump();
      __ nop(); // NOP_ADDED
    }
  }
  if (force_cc && frame_ != NULL && !has_cc()) {
    // Convert the TOS value to a boolean in the condition code register.
    ToBoolean(true_target, false_target);
  }
  ASSERT(!force_cc || !has_valid_frame() || has_cc());
  ASSERT(!has_valid_frame() ||
         (has_cc() && frame_->height() == original_height) ||
         (!has_cc() && frame_->height() == original_height + 1));
}


void CodeGenerator::Load(Expression* x) {
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  JumpTarget true_target;
  JumpTarget false_target;
  LoadCondition(x, &true_target, &false_target, false);

  if (has_cc()) {
    // Convert cc_reg_ into a boolean value.
    JumpTarget loaded;
    JumpTarget materialize_true;
    materialize_true.Branch(cc_reg_);
    __ LoadRoot(a0, Heap::kFalseValueRootIndex);
    frame_->EmitPush(a0);
    loaded.Jump();
    __ nop(); // NOP_ADDED
    materialize_true.Bind();
    __ LoadRoot(a0, Heap::kTrueValueRootIndex);
    frame_->EmitPush(a0);
    loaded.Bind();
    cc_reg_ = cc_always;
  }

  if (true_target.is_linked() || false_target.is_linked()) {
    // We have at least one condition value that has been "translated"
    // into a branch, thus it needs to be loaded explicitly.
    JumpTarget loaded;
    if (frame_ != NULL) {
      loaded.Jump();  // Don't lose the current TOS.
      __ nop(); // NOP_ADDED
    }
    bool both = true_target.is_linked() && false_target.is_linked();
    // Load "true" if necessary.
    if (true_target.is_linked()) {
      true_target.Bind();
      __ LoadRoot(a0, Heap::kTrueValueRootIndex);
      frame_->EmitPush(a0);
    }
    // If both "true" and "false" need to be loaded jump across the code for
    // "false".
    if (both) {
      loaded.Jump();
      __ nop(); // NOP_ADDED
    }
    // Load "false" if necessary.
    if (false_target.is_linked()) {
      false_target.Bind();
      __ LoadRoot(a0, Heap::kFalseValueRootIndex);
      frame_->EmitPush(a0);
    }
    // A value is loaded on all paths reaching this point.
    loaded.Bind();
  }
  ASSERT(has_valid_frame());
  ASSERT(!has_cc());
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::LoadGlobal() {
  VirtualFrame::SpilledScope spilled_scope;
  __ lw(a0, GlobalObject());
  frame_->EmitPush(a0);
}


void CodeGenerator::LoadGlobalReceiver(Register scratch) {
  VirtualFrame::SpilledScope spilled_scope;
  __ lw(scratch, ContextOperand(cp, Context::GLOBAL_INDEX));
  __ lw(scratch,
         FieldMemOperand(scratch, GlobalObject::kGlobalReceiverOffset));
  frame_->EmitPush(scratch);
}


// TODO(1241834): Get rid of this function in favor of just using Load, now
// that we have the INSIDE_TYPEOF typeof state. => Need to handle global
// variables w/o reference errors elsewhere.
void CodeGenerator::LoadTypeofExpression(Expression* x) {
//  VirtualFrame::SpilledScope spilled_scope;
  Variable* variable = x->AsVariableProxy()->AsVariable();
  if (variable != NULL && !variable->is_this() && variable->is_global()) {
    // NOTE: This is somewhat nasty. We force the compiler to load
    // the variable as if through '<global>.<variable>' to make sure we
    // do not get reference errors.
    Slot global(variable, Slot::CONTEXT, Context::GLOBAL_INDEX);
    Literal key(variable->name());
    // TODO(1241834): Fetch the position from the variable instead of using
    // no position.
    Property property(&global, &key, RelocInfo::kNoPosition);
    LoadAndSpill(&property);
  } else {
    LoadAndSpill(x);
  }
}


Reference::Reference(CodeGenerator* cgen, Expression* expression)
    : cgen_(cgen), expression_(expression), type_(ILLEGAL) {
  cgen->LoadReference(this);
}


Reference::~Reference() {
  cgen_->UnloadReference(this);
}


void CodeGenerator::LoadReference(Reference* ref) {
#ifdef DEBUG
//  printf("CodeGenerator::LoadReference\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ LoadReference");
  Expression* e = ref->expression();
  Property* property = e->AsProperty();
  Variable* var = e->AsVariableProxy()->AsVariable();

  if (property != NULL) {
    // The expression is either a property or a variable proxy that rewrites
    // to a property.
    LoadAndSpill(property->obj());
    // We use a named reference if the key is a literal symbol, unless it is
    // a string that can be legally parsed as an integer.  This is because
    // otherwise we will not get into the slow case code that handles [] on
    // String objects.
    Literal* literal = property->key()->AsLiteral();
    uint32_t dummy;
    if (literal != NULL &&
        literal->handle()->IsSymbol() &&
        !String::cast(*(literal->handle()))->AsArrayIndex(&dummy)) {
      ref->set_type(Reference::NAMED);
    } else {
      LoadAndSpill(property->key());
      ref->set_type(Reference::KEYED);
    }
  } else if (var != NULL) {
    // The expression is a variable proxy that does not rewrite to a
    // property.  Global variables are treated as named property references.
    if (var->is_global()) {
      LoadGlobal();
      ref->set_type(Reference::NAMED);
    } else {
      ASSERT(var->slot() != NULL);
      ref->set_type(Reference::SLOT);
    }
  } else {
    // Anything else is a runtime error.
    LoadAndSpill(e);
    frame_->CallRuntime(Runtime::kThrowReferenceError, 1);
    __ nop();   // NOP_ADDED
  }
}


void CodeGenerator::UnloadReference(Reference* ref) {
  VirtualFrame::SpilledScope spilled_scope;
  // Pop a reference from the stack while preserving TOS.
  Comment cmnt(masm_, "[ UnloadReference");
  int size = ref->size();
  if (size > 0) {
    frame_->EmitPop(a0);
    frame_->Drop(size);
    frame_->EmitPush(a0);
  }
}


// ECMA-262, section 9.2, page 30: ToBoolean(). Convert the given
// register to a boolean in the condition code register. The code
// may jump to 'false_target' in case the register converts to 'false'.
void CodeGenerator::ToBoolean(JumpTarget* true_target,
                              JumpTarget* false_target) {
  VirtualFrame::SpilledScope spilled_scope;
  // Note: The generated code snippet does not change stack variables.
  //       Only the condition code should be set.
//  frame_->EmitPop(r0);
  frame_->EmitPop(t0);

  // Fast case checks

  // Check if the value is 'false'.
  __ LoadRoot(t1, Heap::kFalseValueRootIndex);
  false_target->Branch(eq, no_hint, t0, Operand(t1));
  __ nop();  // NOP_ADDED

  // Check if the value is 'true'.
  __ LoadRoot(t2, Heap::kTrueValueRootIndex);
  true_target->Branch(eq, no_hint, t0, Operand(t2));
  __ nop();  // NOP_ADDED

  // Check if the value is 'undefined'.
  __ LoadRoot(t3, Heap::kUndefinedValueRootIndex);
  false_target->Branch(eq, no_hint, t0, Operand(t3));
  __ nop();  // NOP_ADDED

  // Check if the value is a smi.
//  __ cmp(r0, Operand(Smi::FromInt(0)));
  false_target->Branch(eq, no_hint, t0, Operand(Smi::FromInt(0)));
  __ nop();  // NOP_ADDED
  __ andi(t4, t0, Operand(kSmiTagMask));
  true_target->Branch(eq, no_hint, t4, Operand(zero_reg));
  __ nop();  // NOP_ADDED

  // Slow case: call the runtime.
  frame_->EmitPush(t0);
  frame_->CallRuntime(Runtime::kToBool, 1);
  __ nop();  // NOP_ADDED
  // Convert the result (v0) to a condition code.
//  __ cmp(r0, ip);
  __ LoadRoot(s6, Heap::kFalseValueRootIndex);
  __ mov(s5, v0);

  cc_reg_ = ne;
}


void CodeGenerator::GenericBinaryOperation(Token::Value op,
                                           OverwriteMode overwrite_mode,
                                           int constant_rhs) {
#ifdef DEBUG
//  printf("CodeGenerator::GenericBinaryOperation\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  // sp[0] : y
  // sp[1] : x
  // result : v0

  // Stub is entered with a call: 'return address' is in lr.
  switch (op) {
    case Token::ADD:  // fall through.
    case Token::SUB:  // fall through.
    case Token::MUL:
    case Token::DIV:
    case Token::MOD:
    case Token::BIT_OR:
    case Token::BIT_AND:
    case Token::BIT_XOR:
    case Token::SHL:
    case Token::SHR:
    case Token::SAR: {
      frame_->EmitPop(a0);  // a0 : y
      frame_->EmitPop(a1);  // a1 : x
      GenericBinaryOpStub stub(op, overwrite_mode, constant_rhs);
      frame_->CallStub(&stub, 0);
      __ nop(); // NOP_ADDED
      break;
    }

    case Token::COMMA:
      frame_->EmitPop(v0);
      // simply discard left value
      frame_->Drop();
      break;
//
    default:
      // Other cases should have been handled before this point.
      UNREACHABLE();
      break;
  }
}


class DeferredInlineSmiOperation: public DeferredCode {
 public:
  DeferredInlineSmiOperation(Token::Value op,
                             int value,
                             bool reversed,
                             OverwriteMode overwrite_mode)
      : op_(op),
        value_(value),
        reversed_(reversed),
        overwrite_mode_(overwrite_mode) {
    set_comment("[ DeferredInlinedSmiOperation");
  }

  virtual void Generate();

 private:
  Token::Value op_;
  int value_;
  bool reversed_;
  OverwriteMode overwrite_mode_;
};


void DeferredInlineSmiOperation::Generate() {
  // In CodeGenerator::SmiOperation we used a1 instead of a0, and we left the
  // register untouched.
  // We just need to load value_ and switch if necessary
  switch (op_) {
    case Token::ADD:
    case Token::SUB:
    case Token::MUL:
    case Token::MOD:
    case Token::BIT_OR:
    case Token::BIT_XOR:
    case Token::BIT_AND: {
      if (reversed_) {
        __ mov(a0, a1);
        __ li(a1, Operand(Smi::FromInt(value_)));
      } else {
        __ li(a0, Operand(Smi::FromInt(value_)));
      }
      break;
    }
    case Token::SHL:
    case Token::SHR:
    case Token::SAR: {
      if (!reversed_) {
        __ li(a0, Operand(Smi::FromInt(value_)));
      } else {
        UNREACHABLE();  // Should have been handled in SmiOperation.
      }
      break;
    }

    default:
      // Other cases should have been handled before this point.
      UNREACHABLE();
      break;
  }

  GenericBinaryOpStub stub(op_, overwrite_mode_, value_);
  __ CallStub(&stub);
  __ nop(); // NOP_ADDED
}


static bool PopCountLessThanEqual2(unsigned int x) {
  x &= x - 1;
  return (x & (x - 1)) == 0;
}


// Returns the index of the lowest bit set.
static int BitPosition(unsigned x) {
  int bit_posn = 0;
  while ((x & 0xf) == 0) {
    bit_posn += 4;
    x >>= 4;
  }
  while ((x & 1) == 0) {
    bit_posn++;
    x >>= 1;
  }
  return bit_posn;
}


void CodeGenerator::SmiOperation(Token::Value op,
                                 Handle<Object> value,
                                 bool reversed,
                                 OverwriteMode mode) {
#ifdef DEBUG
//  printf("CodeGenerator::SmiOperation\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  // NOTE: This is an attempt to inline (a bit) more of the code for
  // some possible smi operations (like + and -) when (at least) one
  // of the operands is a literal smi. With this optimization, the
  // performance of the system is increased by ~15%, and the generated
  // code size is increased by ~1% (measured on a combination of
  // different benchmarks).

  // We care about keeping a1 unchanged, as it spares the need to reverse the
  // optimistic operation if we need to jump to the deferred code.

  // sp[0] : operand

  // TODO(MIPS.1): Implement overflow check

//  __ break_(0x04008);
  int int_value = Smi::cast(*value)->value();

  JumpTarget exit;
  // We use a1 instead of a0 because in most cases we will need the value in a1
  // if we jump to the deferred code.
  frame_->EmitPop(a1);

  bool something_to_inline = true;
  switch (op) {
    // TODO(MIPS.1): Implement overflow cases in CodeGenerator::SmiOperation
    case Token::ADD: {
      DeferredCode* deferred =
          new DeferredInlineSmiOperation(op, int_value, reversed, mode);

      __ addiu(v0, a1, Operand(value));
//      deferred->Branch(vs);
      __ andi(t0, v0, Operand(kSmiTagMask));
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      deferred->BindExit();
      break;
    }

    case Token::SUB: {
      DeferredCode* deferred =
          new DeferredInlineSmiOperation(op, int_value, reversed, mode);

      if (reversed) {
        __ li(t0, Operand(value));
        __ sub(v0, t0, Operand(a1));
      } else {
        __ li(t0, Operand(value));
        __ sub(v0, a1, Operand(t0));
      }
//      deferred->Branch(vs);
      __ andi(t0, v0, Operand(kSmiTagMask));
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      deferred->BindExit();
      break;
    }


    case Token::BIT_OR:
    case Token::BIT_XOR:
    case Token::BIT_AND: {
      DeferredCode* deferred =
        new DeferredInlineSmiOperation(op, int_value, reversed, mode);
      __ andi(t0, a1, Operand(kSmiTagMask));
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      deferred->BindExit();
      switch (op) {
        case Token::BIT_OR:  __ or_(v0, a1, Operand(value)); break;
        case Token::BIT_XOR: __ xor_(v0, a1, Operand(value)); break;
        case Token::BIT_AND: __ and_(v0, a1, Operand(value)); break;
        default: UNREACHABLE();
      }
      deferred->BindExit();
      break;
    }

    case Token::SHL:
    case Token::SHR:
    case Token::SAR: {
      if (reversed) {
        something_to_inline = false;
        break;
      }
      int shift_value = int_value & 0x1f;  // least significant 5 bits
      DeferredCode* deferred =
        new DeferredInlineSmiOperation(op, shift_value, false, mode);
      __ andi(t0, a1, Operand(kSmiTagMask));
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      __ sra(a2, a1, kSmiTagSize);  // Remove tag
      switch (op) {
        case Token::SHL: {
          if (shift_value != 0) {
            __ sll(v0, a2, shift_value);
          }
          // Check that the result fits in a Smi.
          __ addiu(t3, v0, Operand(0x40000000));
          __ andi(t3, t3, Operand(0x80000000));
          deferred->Branch(ne, t3, Operand(zero_reg));
          __ nop(); // NOP_ADDED
          break;
        }
        case Token::SHR: {
          // LSR by immediate 0 means shifting 32 bits.
          if (shift_value != 0) {
            __ srl(v0, a2, shift_value);
          }
          // check that the *unsigned* result fits in a smi
          // neither of the two high-order bits can be set:
          // - 0x80000000: high bit would be lost when smi tagging
          // - 0x40000000: this number would convert to negative when
          // smi tagging these two cases can only happen with shifts
          // by 0 or 1 when handed a valid smi
          // Check that the result fits in a Smi.
          __ andi(t3, v0, Operand(0xc0000000));
          deferred->Branch(ne, t3, Operand(zero_reg));
          break;
        }
        case Token::SAR: {
          if (shift_value != 0) {
            // ASR by immediate 0 means shifting 32 bits.
            __ sra(v0, a2, shift_value);
          }
          break;
        }
        default: UNREACHABLE();
      }
      __ sll(v0, v0, kSmiTagSize);  // Tag result
      deferred->BindExit();
      break;
    }

    case Token::MOD: {
      if (reversed || int_value < 2 || !IsPowerOf2(int_value)) {
        something_to_inline = false;
        break;
      }
      DeferredCode* deferred =
        new DeferredInlineSmiOperation(op, int_value, reversed, mode);
      unsigned mask = (0x80000000u | kSmiTagMask);
      __ andi(t0, a1, Operand(mask));
      // Go to deferred code on non-Smis and negative.
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      mask = (int_value << kSmiTagSize) - 1;
      __ and_(v0, a1, Operand(mask));
      deferred->BindExit();
      break;
    }

    case Token::MUL: {
      if (!IsEasyToMultiplyBy(int_value)) {
        something_to_inline = false;
        break;
      }
      DeferredCode* deferred =
        new DeferredInlineSmiOperation(op, int_value, reversed, mode);
      unsigned max_smi_that_wont_overflow = Smi::kMaxValue / int_value;
      max_smi_that_wont_overflow <<= kSmiTagSize;
      unsigned mask = 0x80000000u;
      while ((mask & max_smi_that_wont_overflow) == 0) {
        mask |= mask >> 1;
      }
      mask |= kSmiTagMask;
      // This does a single mask that checks for a too high value in a
      // conservative way and for a non-Smi.  It also filters out negative
      // numbers, unfortunately, but since this code is inline we prefer
      // brevity to comprehensiveness.
      __ andi(t0, a1, Operand(mask));
      deferred->Branch(ne, t0, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      MultiplyByKnownInt(masm_, a1, v0, int_value);
      deferred->BindExit();
      break;
    }

    default:
      something_to_inline = false;
      break;
  }

  if (!something_to_inline) {
    if (!reversed) {
      __ li(a1, Operand(value));
      frame_->EmitMultiPush(a0.bit() | a1.bit());
      GenericBinaryOperation(op, mode, int_value);
    } else {
      __ li(a1, Operand(value));
      frame_->EmitMultiPushReversed(a1.bit() | a0.bit());
      GenericBinaryOperation(op, mode, kUnknownIntValue);
    }
  }

  exit.Bind();
}


// On MIPS we load registers s5 and s6 with the values which should be compared.
// With the CodeGenerator::cc_reg_ condition, functions will be able to
// evaluate correctly the condition. (eg CodeGenerator::Branch)
void CodeGenerator::Comparison(Condition cc,
                               Expression* left,
                               Expression* right,
                               bool strict) {
  __ nop();
  if (left != NULL) LoadAndSpill(left);
  if (right != NULL) LoadAndSpill(right);

  VirtualFrame::SpilledScope spilled_scope;
  // sp[0] : y
  // sp[1] : x
  // result : cc register

  // Strict only makes sense for equality comparisons.
  ASSERT(!strict || cc == eq);

  JumpTarget exit;
  JumpTarget smi;
  // Implement '>' and '<=' by reversal to obtain ECMA-262 conversion order.
  if (cc == greater || cc == less_equal) {
    cc = ReverseCondition(cc);
    frame_->EmitPop(a1);
    frame_->EmitPop(a0);
  } else {
    frame_->EmitPop(a0);
    frame_->EmitPop(a1);
  }
  __ or_(t2, a0, Operand(a1));
  __ andi(t3, t2, Operand(kSmiTagMask));
  smi.Branch(eq, no_hint, t3, Operand(zero_reg));

  // Perform non-smi comparison by stub.
  // CompareStub takes arguments in a0 and a1, returns <0, >0 or 0 in r0.
  // We call with 0 args because there are 0 on the stack.
  CompareStub stub(cc, strict);
  frame_->CallStub(&stub, 0);
  __ nop(); // NOP_ADDED
  __ mov(s5, v0);
  __ li(s6, Operand(0));
  exit.Jump();
  __ nop(); // NOP_ADDED

  // Do smi comparisons by pointer comparison.
  smi.Bind();
  __ mov(s5, a1);
  __ mov(s6, a0);

  exit.Bind();
  cc_reg_ = cc;
}


class CallFunctionStub: public CodeStub {
 public:
  CallFunctionStub(int argc, InLoopFlag in_loop)
      : argc_(argc), in_loop_(in_loop) {}

  void Generate(MacroAssembler* masm);

 private:
  int argc_;
  InLoopFlag in_loop_;

#if defined(DEBUG)
  void Print() { PrintF("CallFunctionStub (argc %d)\n", argc_); }
#endif  // defined(DEBUG)

  Major MajorKey() { return CallFunction; }
  int MinorKey() { return argc_; }
  InLoopFlag InLoop() { return in_loop_; }
};


// Call the function on the stack with the given arguments.
void CodeGenerator::CallWithArguments(ZoneList<Expression*>* args,
                                         int position) {
#ifdef DEBUG
//  printf("Using CodeGenerator::CallWithArguments. There may be issues with stack alignment.\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  // Push the arguments ("left-to-right") on the stack.
  int arg_count = args->length();
  for (int i = 0; i < arg_count; i++) {
    LoadAndSpill(args->at(i));
  }

  // Record the position for debugging purposes.
  CodeForSourcePosition(position);

  // Use the shared code stub to call the function.
  InLoopFlag in_loop = loop_nesting() > 0 ? IN_LOOP : NOT_IN_LOOP;
  CallFunctionStub call_function(arg_count, in_loop);
  frame_->CallStub(&call_function, arg_count + 1);
  __ nop(); // NOP_ADDED
//  __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize); // (branch delay)

  // We need to manually restore context and pop function from the stack after
  // this call.
}


void CodeGenerator::Branch(bool if_true, JumpTarget* target) {
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(has_cc());
  Condition cc = if_true ? cc_reg_ : NegateCondition(cc_reg_);
  // cf CodeGenerator::Comparison comments.
  target->Branch(cc, no_hint, s5, Operand(s6));
  cc_reg_ = cc_always;
}


void CodeGenerator::CheckStack() {
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ check stack");

  __ LoadRoot(ip, Heap::kStackLimitRootIndex);
  StackCheckStub stub;
  // Call the stub if lower.
  __ jalcond(Operand(reinterpret_cast<intptr_t>(stub.GetCode().location()),
                                                      RelocInfo::CODE_TARGET),
              Uless, sp, Operand(ip)
            );
  __ nop();
}


void CodeGenerator::VisitStatements(ZoneList<Statement*>* statements) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitStatements\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  for (int i = 0; frame_ != NULL && i < statements->length(); i++) {
    VisitAndSpill(statements->at(i));
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitBlock(Block* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitBlock\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Block");
  CodeForStatementPosition(node);
  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);
  VisitStatementsAndSpill(node->statements());
  if (node->break_target()->is_linked()) {
    node->break_target()->Bind();
  }
  node->break_target()->Unuse();
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
#ifdef DEBUG
//  printf("CodeGenerator::DeclareGlobals\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  frame_->EmitPush(cp);
  __ li(t0, Operand(pairs));
  frame_->EmitPush(t0);
  __ li(t0, Operand(Smi::FromInt(is_eval() ? 1 : 0)));
  frame_->EmitPush(t0);
  frame_->CallRuntime(Runtime::kDeclareGlobals, 3);
  __ nop(); // NOP_ADDED
  // The result is discarded.
}


void CodeGenerator::VisitDeclaration(Declaration* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitDeclaration\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Declaration");
  Variable* var = node->proxy()->var();
  ASSERT(var != NULL);  // must have been resolved
  Slot* slot = var->slot();

  // If it was not possible to allocate the variable at compile time,
  // we need to "declare" it at runtime to make sure it actually
  // exists in the local context.
  if (slot != NULL && slot->type() == Slot::LOOKUP) {
    // Variables with a "LOOKUP" slot were introduced as non-locals
    // during variable resolution and must have mode DYNAMIC.
    ASSERT(var->is_dynamic());
    // For now, just do a runtime call.
    frame_->EmitPush(cp);
//    __ mov(r0, Operand(var->name()));
//    frame_->EmitPush(r0);
    __ li(t0, Operand(var->name()));
    frame_->EmitPush(t0);
    // Declaration nodes are always declared in only two modes.
    ASSERT(node->mode() == Variable::VAR || node->mode() == Variable::CONST);
    PropertyAttributes attr = node->mode() == Variable::VAR ? NONE : READ_ONLY;
//    __ mov(r0, Operand(Smi::FromInt(attr)));
//    frame_->EmitPush(r0);
    __ li(t0, Operand(Smi::FromInt(attr)));
    frame_->EmitPush(t0);
    // Push initial value, if any.
    // Note: For variables we must not push an initial value (such as
    // 'undefined') because we may have a (legal) redeclaration and we
    // must not destroy the current value.
    if (node->mode() == Variable::CONST) {
//      __ LoadRoot(r0, Heap::kTheHoleValueRootIndex);
//      frame_->EmitPush(r0);
      __ LoadRoot(t0, Heap::kTheHoleValueRootIndex);
      frame_->EmitPush(t0);
    } else if (node->fun() != NULL) {
      LoadAndSpill(node->fun());
    } else {
//      __ mov(r0, Operand(0));  // no initial value!
//      frame_->EmitPush(r0);
      __ li(t0, Operand(0));  // no initial value!
      frame_->EmitPush(t0);
    }
    frame_->CallRuntime(Runtime::kDeclareContextSlot, 4);
    // Ignore the return value (declarations are statements).
    ASSERT(frame_->height() == original_height);
    return;
  }

  ASSERT(!var->is_global());

  // If we have a function or a constant, we need to initialize the variable.
  Expression* val = NULL;
  if (node->mode() == Variable::CONST) {
    val = new Literal(Factory::the_hole_value());
  } else {
    val = node->fun();  // NULL if we don't have a function
  }

  if (val != NULL) {
    {
      // Set initial value.
      Reference target(this, node->proxy());
      LoadAndSpill(val);
      target.SetValue(NOT_CONST_INIT);
      // The reference is removed from the stack (preserving TOS) when
      // it goes out of scope.
    }
    // Get rid of the assigned value (declarations are statements).
    frame_->Drop();
  }
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitExpressionStatement(ExpressionStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitExpressionStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ExpressionStatement");
  CodeForStatementPosition(node);
  Expression* expression = node->expression();
  expression->MarkAsStatement();
  LoadAndSpill(expression);
  frame_->Drop();
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitEmptyStatement(EmptyStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitEmptyStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "// EmptyStatement");
  CodeForStatementPosition(node);
  // nothing to do
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitIfStatement(IfStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitIfStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ IfStatement");
  // Generate different code depending on which parts of the if statement
  // are present or not.
  bool has_then_stm = node->HasThenStatement();
  bool has_else_stm = node->HasElseStatement();

  CodeForStatementPosition(node);

  JumpTarget exit;
  if (has_then_stm && has_else_stm) {
    Comment cmnt(masm_, "[ IfThenElse");
    JumpTarget then;
    JumpTarget else_;
    // if (cond)
    LoadConditionAndSpill(node->condition(),
                          &then, &else_, true);
    if (frame_ != NULL) {
      Branch(false, &else_);
    }
    // then
    if (frame_ != NULL || then.is_linked()) {
      then.Bind();
      VisitAndSpill(node->then_statement());
    }
    if (frame_ != NULL) {
      exit.Jump();
      __ nop(); // NOP_ADDED
    }
    // else
    if (else_.is_linked()) {
      else_.Bind();
      VisitAndSpill(node->else_statement());
    }

  } else if (has_then_stm) {
    Comment cmnt(masm_, "[ IfThen");
    ASSERT(!has_else_stm);
    JumpTarget then;
    // if (cond)
    LoadConditionAndSpill(node->condition(),
                          &then, &exit, true);
    if (frame_ != NULL) {
      Branch(false, &exit);
    }
    // then
    if (frame_ != NULL || then.is_linked()) {
      then.Bind();
      VisitAndSpill(node->then_statement());
    }

  } else if (has_else_stm) {
    Comment cmnt(masm_, "[ IfElse");
    ASSERT(!has_then_stm);
    JumpTarget else_;
    // if (!cond)
    LoadConditionAndSpill(node->condition(),
                          &exit, &else_, true);
    if (frame_ != NULL) {
      Branch(true, &exit);
    }
    // else
    if (frame_ != NULL || else_.is_linked()) {
      else_.Bind();
      VisitAndSpill(node->else_statement());
    }

  } else {
    Comment cmnt(masm_, "[ If");
    ASSERT(!has_then_stm && !has_else_stm);
    // if (cond)
    LoadConditionAndSpill(node->condition(),
                          &exit, &exit, false);
    if (frame_ != NULL) {
      if (has_cc()) {
        cc_reg_ = cc_always;
      } else {
        frame_->Drop();
      }
    }
  }

  // end
  if (exit.is_linked()) {
    exit.Bind();
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitContinueStatement(ContinueStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitContinueStatement\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ContinueStatement");
  CodeForStatementPosition(node);
  node->target()->continue_target()->Jump();
  __ nop(); // NOP_ADDED
}


void CodeGenerator::VisitBreakStatement(BreakStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitBreakStatement\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ BreakStatement");
  CodeForStatementPosition(node);
  node->target()->break_target()->Jump();
  __ nop(); // NOP_ADDED
}


void CodeGenerator::VisitReturnStatement(ReturnStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitReturnStatement\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ReturnStatement");

  CodeForStatementPosition(node);
  LoadAndSpill(node->expression());
  if (function_return_is_shadowed_) {
    frame_->EmitPop(v0);
    function_return_.Jump();
    __ nop();   // NOP_ADDED
  } else {
    // Pop the result from the frame and prepare the frame for
    // returning thus making it easier to merge.
    frame_->EmitPop(v0);
//    __ break_(0x00009);
    frame_->PrepareForReturn();

    function_return_.Jump();
    __ nop();   // NOP_ADDED
  }
}


void CodeGenerator::VisitWithEnterStatement(WithEnterStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitWithEnterStatement\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ WithEnterStatement");
  CodeForStatementPosition(node);
  LoadAndSpill(node->expression());
  if (node->is_catch_block()) {
    frame_->CallRuntime(Runtime::kPushCatchContext, 1);
    __ nop(); // NOP_ADDED
  } else {
    frame_->CallRuntime(Runtime::kPushContext, 1);
    __ nop(); // NOP_ADDED
  }
#ifdef DEBUG
  JumpTarget verified_true;
//  __ cmp(r0, Operand(cp));
  verified_true.Branch(eq, no_hint, v0, Operand(cp));
  __ nop(); // NOP_ADDED
  __ stop("PushContext: v0 is expected to be the same as cp");
  verified_true.Bind();
  __ nop(); // NOP_ADDED
#endif
  // Update context local.
//  __ str(cp, frame_->Context());
  __ sw(cp, frame_->Context());
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitWithExitStatement(WithExitStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitWithExitStatement\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ WithExitStatement");
  CodeForStatementPosition(node);
  // Pop context.
//  __ ldr(cp, ContextOperand(cp, Context::PREVIOUS_INDEX));
  __ lw(cp, ContextOperand(cp, Context::PREVIOUS_INDEX));
  // Update context local.
//  __ str(cp, frame_->Context());
  __ sw(cp, frame_->Context());
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitSwitchStatement(SwitchStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitSwitchStatement\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ SwitchStatement");
  CodeForStatementPosition(node);
  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);

  LoadAndSpill(node->tag());

  JumpTarget next_test;
  JumpTarget fall_through;
  JumpTarget default_entry;
  JumpTarget default_exit(JumpTarget::BIDIRECTIONAL);
  ZoneList<CaseClause*>* cases = node->cases();
  int length = cases->length();
  CaseClause* default_clause = NULL;

  for (int i = 0; i < length; i++) {
    CaseClause* clause = cases->at(i);
    if (clause->is_default()) {
      // Remember the default clause and compile it at the end.
      default_clause = clause;
      continue;
    }
//
    Comment cmnt(masm_, "[ Case clause");
    // Compile the test.
    next_test.Bind();
    next_test.Unuse();
    // Duplicate TOS.
//    __ ldr(r0, frame_->Top());
//    frame_->EmitPush(r0);
    __ lw(a0, frame_->Top());
    frame_->EmitPush(a0);
    Comparison(eq, NULL, clause->label(), true);
    Branch(false, &next_test);
    __ nop(); // NOP_ADDED

    // Before entering the body from the test, remove the switch value from
    // the stack.
    frame_->Drop();

    // Label the body so that fall through is enabled.
    if (i > 0 && cases->at(i - 1)->is_default()) {
      default_exit.Bind();
    } else {
      fall_through.Bind();
      fall_through.Unuse();
    }
    VisitStatementsAndSpill(clause->statements());

    // If control flow can fall through from the body, jump to the next body
    // or the end of the statement.
    if (frame_ != NULL) {
      if (i < length - 1 && cases->at(i + 1)->is_default()) {
        default_entry.Jump();
        __ nop(); // NOP_ADDED
      } else {
        fall_through.Jump();
        __ nop(); // NOP_ADDED
      }
    }
  }

  // The final "test" removes the switch value.
  next_test.Bind();
  frame_->Drop();

  // If there is a default clause, compile it.
  if (default_clause != NULL) {
    Comment cmnt(masm_, "[ Default clause");
    default_entry.Bind();
    VisitStatementsAndSpill(default_clause->statements());
    // If control flow can fall out of the default and there is a case after
    // it, jup to that case's body.
    if (frame_ != NULL && default_exit.is_bound()) {
      default_exit.Jump();
      __ nop(); // NOP_ADDED
    }
  }

  if (fall_through.is_linked()) {
    fall_through.Bind();
  }

  if (node->break_target()->is_linked()) {
    node->break_target()->Bind();
  }
  node->break_target()->Unuse();
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitDoWhileStatement(DoWhileStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitDoWhileStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ DoWhileStatement");
  CodeForStatementPosition(node);
  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);
  JumpTarget body(JumpTarget::BIDIRECTIONAL);

  // Label the top of the loop for the backward CFG edge.  If the test
  // is always true we can use the continue target, and if the test is
  // always false there is no need.
  ConditionAnalysis info = AnalyzeCondition(node->cond());
  switch (info) {
    case ALWAYS_TRUE:
      node->continue_target()->set_direction(JumpTarget::BIDIRECTIONAL);
      node->continue_target()->Bind();
      break;
    case ALWAYS_FALSE:
      node->continue_target()->set_direction(JumpTarget::FORWARD_ONLY);
      break;
    case DONT_KNOW:
      node->continue_target()->set_direction(JumpTarget::FORWARD_ONLY);
      body.Bind();
      break;
  }

  CheckStack();  // TODO(1222600): ignore if body contains calls.
  VisitAndSpill(node->body());

      // Compile the test.
  switch (info) {
    case ALWAYS_TRUE:
      // If control can fall off the end of the body, jump back to the
      // top.
      if (has_valid_frame()) {
        node->continue_target()->Jump();
        __ nop(); // NOP_ADDED
      }
      break;
    case ALWAYS_FALSE:
      // If we have a continue in the body, we only have to bind its
      // jump target.
      if (node->continue_target()->is_linked()) {
        node->continue_target()->Bind();
      }
      break;
    case DONT_KNOW:
      // We have to compile the test expression if it can be reached by
      // control flow falling out of the body or via continue.
      if (node->continue_target()->is_linked()) {
        node->continue_target()->Bind();
      }
      if (has_valid_frame()) {
        LoadConditionAndSpill(node->cond(), &body, node->break_target(), true);
        if (has_valid_frame()) {
          // A invalid frame here indicates that control did not
          // fall out of the test expression.
          Branch(true, &body);
        }
      }
      break;
  }

  if (node->break_target()->is_linked()) {
    node->break_target()->Bind();
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitWhileStatement(WhileStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitWhileStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ WhileStatement");
  CodeForStatementPosition(node);

  // If the test is never true and has no side effects there is no need
  // to compile the test or body.
  ConditionAnalysis info = AnalyzeCondition(node->cond());
  if (info == ALWAYS_FALSE) return;

  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);

  // Label the top of the loop with the continue target for the backward
  // CFG edge.
  node->continue_target()->set_direction(JumpTarget::BIDIRECTIONAL);
  node->continue_target()->Bind();


  if (info == DONT_KNOW) {
    JumpTarget body;
    LoadConditionAndSpill(node->cond(), &body, node->break_target(), true);
    if (has_valid_frame()) {
      // A NULL frame indicates that control did not fall out of the
      // test expression.
      Branch(false, node->break_target());
    }
    if (has_valid_frame() || body.is_linked()) {
      body.Bind();
    }
  }

  if (has_valid_frame()) {
    CheckStack();  // TODO(1222600): ignore if body contains calls.
    VisitAndSpill(node->body());

    // If control flow can fall out of the body, jump back to the top.
    if (has_valid_frame()) {
      node->continue_target()->Jump();
      __ nop(); // NOP_ADDED
    }
  }
  if (node->break_target()->is_linked()) {
    node->break_target()->Bind();
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitForStatement(ForStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitForStatement\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ForStatement");
  CodeForStatementPosition(node);
  if (node->init() != NULL) {
    VisitAndSpill(node->init());
  }

  // If the test is never true there is no need to compile the test or
  // body.
  ConditionAnalysis info = AnalyzeCondition(node->cond());
  if (info == ALWAYS_FALSE) return;

  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);

  // If there is no update statement, label the top of the loop with the
  // continue target, otherwise with the loop target.
  JumpTarget loop(JumpTarget::BIDIRECTIONAL);
  if (node->next() == NULL) {
    node->continue_target()->set_direction(JumpTarget::BIDIRECTIONAL);
    node->continue_target()->Bind();
  } else {
    node->continue_target()->set_direction(JumpTarget::FORWARD_ONLY);
    loop.Bind();
  }

  // If the test is always true, there is no need to compile it.
  if (info == DONT_KNOW) {
    JumpTarget body;
    LoadConditionAndSpill(node->cond(), &body, node->break_target(), true);
    if (has_valid_frame()) {
      Branch(false, node->break_target());
      __ nop(); // NOP_ADDED
    }
    if (has_valid_frame() || body.is_linked()) {
      body.Bind();
    }
  }

  if (has_valid_frame()) {
    CheckStack();  // TODO(1222600): ignore if body contains calls.
    VisitAndSpill(node->body());

    if (node->next() == NULL) {
      // If there is no update statement and control flow can fall out
      // of the loop, jump directly to the continue label.
      if (has_valid_frame()) {
        node->continue_target()->Jump();
        __ nop(); // NOP_ADDED
      }
    } else {
      // If there is an update statement and control flow can reach it
      // via falling out of the body of the loop or continuing, we
      // compile the update statement.
      if (node->continue_target()->is_linked()) {
        node->continue_target()->Bind();
      }
      if (has_valid_frame()) {
        // Record source position of the statement as this code which is
        // after the code for the body actually belongs to the loop
        // statement and not the body.
        CodeForStatementPosition(node);
        VisitAndSpill(node->next());
        loop.Jump();
        __ nop(); // NOP_ADDED
      }
    }
  }
  if (node->break_target()->is_linked()) {
    node->break_target()->Bind();
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitForInStatement(ForInStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitForInStatement\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ForInStatement");
  CodeForStatementPosition(node);

  JumpTarget primitive;
  JumpTarget jsobject;
  JumpTarget fixed_array;
  JumpTarget entry(JumpTarget::BIDIRECTIONAL);
  JumpTarget end_del_check;
  JumpTarget exit;

  // Get the object to enumerate over (converted to JSObject).
  LoadAndSpill(node->enumerable());

  // Both SpiderMonkey and kjs ignore null and undefined in contrast
  // to the specification.  12.6.4 mandates a call to ToObject.
//  frame_->EmitPop(r0);
//  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//  __ cmp(r0, ip);
//  exit.Branch(eq);
//  __ LoadRoot(ip, Heap::kNullValueRootIndex);
//  __ cmp(r0, ip);
//  exit.Branch(eq);
  frame_->EmitPop(a0);
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
  exit.Branch(eq, no_hint, a0, Operand(ip));
  __ nop(); // NOP_ADDED
  __ LoadRoot(ip, Heap::kNullValueRootIndex);
  exit.Branch(eq, no_hint, a0, Operand(ip));
  __ nop(); // NOP_ADDED

  // Stack layout in body:
  // [iteration counter (Smi)]
  // [length of array]
  // [FixedArray]
  // [Map or 0]
  // [Object]

  // Check if enumerable is already a JSObject
//  __ tst(r0, Operand(kSmiTagMask));
//  primitive.Branch(eq);
//  __ CompareObjectType(r0, r1, r1, FIRST_JS_OBJECT_TYPE);
//  jsobject.Branch(hs);
  __ andi(t0, a0, Operand(kSmiTagMask));
  primitive.Branch(eq, no_hint, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  __ GetObjectType(a0, a1, a1);
  jsobject.Branch(Ugreater_equal, no_hint, a1, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED

  primitive.Bind();
#ifdef NO_NATIVES
  __ break_(0x1973);
#else
  frame_->EmitPush(a0);
  Result arg_count(a0);
//  __ mov(r0, Operand(0));
  __ li(a0, Operand(0));
  frame_->InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS, &arg_count, 1);
  __ nop(); // NOP_ADDED
#endif

  jsobject.Bind();
  // Get the set of properties (as a FixedArray or Map).
  frame_->EmitPush(a0);  // duplicate the object being enumerated
  frame_->EmitPush(a0);
  frame_->CallRuntime(Runtime::kGetPropertyNamesFast, 1);
  __ nop(); // NOP_ADDED

  // If we got a Map, we can do a fast modification check.
  // Otherwise, we got a FixedArray, and we have to do a slow check.
//  __ mov(r2, Operand(r0));
//  __ ldr(r1, FieldMemOperand(r2, HeapObject::kMapOffset));
//  __ LoadRoot(ip, Heap::kMetaMapRootIndex);
//  __ cmp(r1, ip);
//  fixed_array.Branch(ne);
  __ mov(a2, v0);
  __ lw(a1, FieldMemOperand(a2, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kMetaMapRootIndex);
  fixed_array.Branch(ne, no_hint, a1, Operand(ip));
  __ nop(); // NOP_ADDED

  // Get enum cache
//  __ mov(r1, Operand(r0));
//  __ ldr(r1, FieldMemOperand(r1, Map::kInstanceDescriptorsOffset));
//  __ ldr(r1, FieldMemOperand(r1, DescriptorArray::kEnumerationIndexOffset));
//  __ ldr(r2,
//         FieldMemOperand(r1, DescriptorArray::kEnumCacheBridgeCacheOffset));
  __ mov(a1, v0);
  __ lw(a1, FieldMemOperand(a1, Map::kInstanceDescriptorsOffset));
  __ lw(a1, FieldMemOperand(a1, DescriptorArray::kEnumerationIndexOffset));
  __ lw(a2, FieldMemOperand(a1, DescriptorArray::kEnumCacheBridgeCacheOffset));

//  frame_->EmitPush(r0);  // map
//  frame_->EmitPush(r2);  // enum cache bridge cache
//  __ ldr(r0, FieldMemOperand(r2, FixedArray::kLengthOffset));
//  __ mov(r0, Operand(r0, LSL, kSmiTagSize));
//  frame_->EmitPush(r0);
//  __ mov(r0, Operand(Smi::FromInt(0)));
//  frame_->EmitPush(r0);
//  entry.Jump();
  frame_->EmitPush(v0);  // map
  frame_->EmitPush(a2);  // enum cache bridge cache
  __ lw(a0, FieldMemOperand(a2, FixedArray::kLengthOffset));
  __ sll(a0, a0, kSmiTagSize);
  frame_->EmitPush(a0);
  __ li(a0, Operand(Smi::FromInt(0)));
  frame_->EmitPush(a0);
  entry.Jump();
  __ nop(); // NOP_ADDED

  fixed_array.Bind();
//  __ mov(r1, Operand(Smi::FromInt(0)));
//  frame_->EmitPush(r1);  // insert 0 in place of Map
//  frame_->EmitPush(r0);
  __ li(a1, Operand(Smi::FromInt(0)));
  frame_->EmitPush(a1);  // insert 0 in place of Map
  frame_->EmitPush(v0);

  // Push the length of the array and the initial index onto the stack.
//  __ ldr(r0, FieldMemOperand(r0, FixedArray::kLengthOffset));
//  __ mov(r0, Operand(r0, LSL, kSmiTagSize));
//  frame_->EmitPush(r0);
//  __ mov(r0, Operand(Smi::FromInt(0)));  // init index
//  frame_->EmitPush(r0);
  __ lw(a0, FieldMemOperand(v0, FixedArray::kLengthOffset));
  __ sll(a0, a0, kSmiTagSize);
  frame_->EmitPush(a0);
  __ li(a0, Operand(Smi::FromInt(0)));  // init index
  frame_->EmitPush(a0);

  // Condition.
  entry.Bind();
  // sp[0] : index
  // sp[1] : array/enum cache length
  // sp[2] : array or enum cache
  // sp[3] : 0 or map
  // sp[4] : enumerable
  // Grab the current frame's height for the break and continue
  // targets only after all the state is pushed on the frame.
  node->break_target()->set_direction(JumpTarget::FORWARD_ONLY);
  node->continue_target()->set_direction(JumpTarget::FORWARD_ONLY);

//  __ ldr(r0, frame_->ElementAt(0));  // load the current count
//  __ ldr(r1, frame_->ElementAt(1));  // load the length
//  __ cmp(r0, Operand(r1));  // compare to the array length
//  node->break_target()->Branch(hs);
  __ lw(a0, frame_->ElementAt(0));  // load the current count
  __ lw(a1, frame_->ElementAt(1));  // load the length
  node->break_target()->Branch(Ugreater_equal, no_hint, a0, Operand(a1));
  __ nop(); // NOP_ADDED

//  __ ldr(r0, frame_->ElementAt(0));
  __ lw(a0, frame_->ElementAt(0));

  // Get the i'th entry of the array.
//  __ ldr(r2, frame_->ElementAt(2));
//  __ add(r2, r2, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
//  __ ldr(r3, MemOperand(r2, r0, LSL, kPointerSizeLog2 - kSmiTagSize));
  __ lw(a2, frame_->ElementAt(2));
  __ addu(a2, a2, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ sll(t0, a0, kPointerSizeLog2 - kSmiTagSize);
  __ addu(t2, t0, a2);
  __ lw(a3, MemOperand(t2));

  // Get Map or 0.
//  __ ldr(r2, frame_->ElementAt(3));
  __ lw(a2, frame_->ElementAt(3));
  // Check if this (still) matches the map of the enumerable.
  // If not, we have to filter the key.
//  __ ldr(r1, frame_->ElementAt(4));
//  __ ldr(r1, FieldMemOperand(r1, HeapObject::kMapOffset));
//  __ cmp(r1, Operand(r2));
//  end_del_check.Branch(eq);
  __ lw(a1, frame_->ElementAt(4));
  __ lw(a1, FieldMemOperand(a1, HeapObject::kMapOffset));
  end_del_check.Branch(eq, no_hint, a1, Operand(a2));
  __ nop(); // NOP_ADDED

  // Convert the entry to a string (or null if it isn't a property anymore).
//  __ ldr(r0, frame_->ElementAt(4));  // push enumerable
//  frame_->EmitPush(r0);
//  frame_->EmitPush(r3);  // push entry
//  Result arg_count_reg(r0);
//  __ mov(r0, Operand(1));
//  frame_->InvokeBuiltin(Builtins::FILTER_KEY, CALL_JS, &arg_count_reg, 2);
//  __ mov(r3, Operand(r0));
  __ lw(a0, frame_->ElementAt(4));  // push enumerable
  frame_->EmitPush(a0);
  frame_->EmitPush(a3);  // push entry
  Result arg_count_reg(a0);
  __ li(a0, Operand(1));
  frame_->InvokeBuiltin(Builtins::FILTER_KEY, CALL_JS, &arg_count_reg, 2);
  __ nop(); // NOP_ADDED
  __ mov(a3, a0);

  // If the property has been removed while iterating, we just skip it.
  __ LoadRoot(ip, Heap::kNullValueRootIndex);
//  __ cmp(r3, ip);
  node->continue_target()->Branch(eq, no_hint, a3, Operand(ip));
  __ nop(); // NOP_ADDED

  end_del_check.Bind();
  // Store the entry in the 'each' expression and take another spin in the
  // loop.  r3: i'th entry of the enum cache (or string there of)
//  frame_->EmitPush(r3);  // push entry
  frame_->EmitPush(a3);  // push entry
  { Reference each(this, node->each());
    if (!each.is_illegal()) {
      if (each.size() > 0) {
//        __ ldr(r0, frame_->ElementAt(each.size()));
//        frame_->EmitPush(r0);
        __ lw(a0, frame_->ElementAt(each.size()));
        frame_->EmitPush(a0);
      }
      // If the reference was to a slot we rely on the convenient property
      // that it doesn't matter whether a value (eg, r3 pushed above) is
      // right on top of or right underneath a zero-sized reference.
      each.SetValue(NOT_CONST_INIT);
      if (each.size() > 0) {
        // It's safe to pop the value lying on top of the reference before
        // unloading the reference itself (which preserves the top of stack,
        // ie, now the topmost value of the non-zero sized reference), since
        // we will discard the top of stack after unloading the reference
        // anyway.
//        frame_->EmitPop(r0);
        frame_->EmitPop(a0);
      }
    }
  }
  // Discard the i'th entry pushed above or else the remainder of the
  // reference, whichever is currently on top of the stack.
  frame_->Drop();

  // Body.
  CheckStack();  // TODO(1222600): ignore if body contains calls.
  VisitAndSpill(node->body());

  // Next.  Reestablish a spilled frame in case we are coming here via
  // a continue in the body.
  node->continue_target()->Bind();
  frame_->SpillAll();
//  frame_->EmitPop(r0);
//  __ add(r0, r0, Operand(Smi::FromInt(1)));
//  frame_->EmitPush(r0);
  frame_->EmitPop(a0);
  __ add(a0, a0, Operand(Smi::FromInt(1)));
  frame_->EmitPush(a0);
  entry.Jump();
  __ nop(); // NOP_ADDED

  // Cleanup.  No need to spill because VirtualFrame::Drop is safe for
  // any frame.
  node->break_target()->Bind();
  frame_->Drop(5);

  // Exit.
  exit.Bind();
  node->continue_target()->Unuse();
  node->break_target()->Unuse();
  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::VisitTryCatchStatement(TryCatchStatement* node) {
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ TryCatchStatement");
  CodeForStatementPosition(node);

  JumpTarget try_block;
  JumpTarget exit;


  try_block.Call();
  __ nop();
  // --- Catch block ---
//  frame_->EmitPush(r0);
  frame_->EmitPush(v0);

  // Store the caught exception in the catch variable.
  { Reference ref(this, node->catch_var());
    ASSERT(ref.is_slot());
    // Here we make use of the convenient property that it doesn't matter
    // whether a value is immediately on top of or underneath a zero-sized
    // reference.
    ref.SetValue(NOT_CONST_INIT);
  }

  // Remove the exception from the stack.
  frame_->Drop();

  VisitStatementsAndSpill(node->catch_block()->statements());
  if (frame_ != NULL) {
    exit.Jump();
    __ nop();
  }


  // --- Try block ---
  try_block.Bind();

  frame_->PushTryHandler(TRY_CATCH_HANDLER);
  int handler_height = frame_->height();

  // Shadow the labels for all escapes from the try block, including
  // returns. During shadowing, the original label is hidden as the
  // LabelShadow and operations on the original actually affect the
  // shadowing label.
  //
  // We should probably try to unify the escaping labels and the return
  // label.
  int nof_escapes = node->escaping_targets()->length();
  List<ShadowTarget*> shadows(1 + nof_escapes);

  // Add the shadow target for the function return.
  static const int kReturnShadowIndex = 0;
  shadows.Add(new ShadowTarget(&function_return_));
  bool function_return_was_shadowed = function_return_is_shadowed_;
  function_return_is_shadowed_ = true;
  ASSERT(shadows[kReturnShadowIndex]->other_target() == &function_return_);

  // Add the remaining shadow targets.
  for (int i = 0; i < nof_escapes; i++) {
    shadows.Add(new ShadowTarget(node->escaping_targets()->at(i)));
  }

  // Generate code for the statements in the try block.
  VisitStatementsAndSpill(node->try_block()->statements());

  // Stop the introduced shadowing and count the number of required unlinks.
  // After shadowing stops, the original labels are unshadowed and the
  // LabelShadows represent the formerly shadowing labels.
  bool has_unlinks = false;
  for (int i = 0; i < shadows.length(); i++) {
    shadows[i]->StopShadowing();
    has_unlinks = has_unlinks || shadows[i]->is_linked();
  }
  function_return_is_shadowed_ = function_return_was_shadowed;

  // Get an external reference to the handler address.
  ExternalReference handler_address(Top::k_handler_address);

  // If we can fall off the end of the try block, unlink from try chain.
  if (has_valid_frame()) {
    // The next handler address is on top of the frame.  Unlink from
    // the handler list and drop the rest of this handler from the
    // frame.
    ASSERT(StackHandlerConstants::kNextOffset == 0);
//    frame_->EmitPop(r1);
//    __ mov(r3, Operand(handler_address));
//    __ str(r1, MemOperand(r3));
    frame_->EmitPop(a1);
    __ li(a3, Operand(handler_address));
    __ sw(a1, MemOperand(a3));
    frame_->Drop(StackHandlerConstants::kSize / kPointerSize - 1);
    if (has_unlinks) {
      exit.Jump();
      __ nop();
    }
  }

  // Generate unlink code for the (formerly) shadowing labels that have been
  // jumped to.  Deallocate each shadow target.
  for (int i = 0; i < shadows.length(); i++) {
    if (shadows[i]->is_linked()) {
      // Unlink from try chain;
      shadows[i]->Bind();
      // Because we can be jumping here (to spilled code) from unspilled
      // code, we need to reestablish a spilled frame at this block.
      frame_->SpillAll();

      // Reload sp from the top handler, because some statements that we
      // break from (eg, for...in) may have left stuff on the stack.
//      __ mov(r3, Operand(handler_address));
//      __ ldr(sp, MemOperand(r3));
      __ li(a3, Operand(handler_address));
      __ lw(sp, MemOperand(a3));
      frame_->Forget(frame_->height() - handler_height);

      ASSERT(StackHandlerConstants::kNextOffset == 0);
//      frame_->EmitPop(r1);
//      __ str(r1, MemOperand(r3));
      frame_->EmitPop(a1);
      __ sw(a1, MemOperand(a3));
      frame_->Drop(StackHandlerConstants::kSize / kPointerSize - 1);

      if (!function_return_is_shadowed_ && i == kReturnShadowIndex) {
        frame_->PrepareForReturn();
      }
      shadows[i]->other_target()->Jump();
    }
  }

  exit.Bind();
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitTryFinallyStatement(TryFinallyStatement* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitTryFinallyStatement\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ TryFinallyStatement");
  CodeForStatementPosition(node);

  // State: Used to keep track of reason for entering the finally
  // block. Should probably be extended to hold information for
  // break/continue from within the try block.
  enum { FALLING, THROWING, JUMPING };

  JumpTarget try_block;
  JumpTarget finally_block;

  try_block.Call();
  __ nop(); // NOP_ADDED

  frame_->EmitPush(a0);  // save exception object on the stack
  // In case of thrown exceptions, this is where we continue.
//  __ mov(r2, Operand(Smi::FromInt(THROWING)));
  __ li(a2, Operand(Smi::FromInt(THROWING)));
  finally_block.Jump();
  __ nop(); // NOP_ADDED

  // --- Try block ---
  try_block.Bind();

  frame_->PushTryHandler(TRY_FINALLY_HANDLER);
  int handler_height = frame_->height();

  // Shadow the labels for all escapes from the try block, including
  // returns.  Shadowing hides the original label as the LabelShadow and
  // operations on the original actually affect the shadowing label.

  // We should probably try to unify the escaping labels and the return
  // label.
  int nof_escapes = node->escaping_targets()->length();
  List<ShadowTarget*> shadows(1 + nof_escapes);

  // Add the shadow target for the function return.
  static const int kReturnShadowIndex = 0;
  shadows.Add(new ShadowTarget(&function_return_));
  bool function_return_was_shadowed = function_return_is_shadowed_;
  function_return_is_shadowed_ = true;
  ASSERT(shadows[kReturnShadowIndex]->other_target() == &function_return_);

  // Add the remaining shadow targets.
  for (int i = 0; i < nof_escapes; i++) {
    shadows.Add(new ShadowTarget(node->escaping_targets()->at(i)));
  }

  // Generate code for the statements in the try block.
  VisitStatementsAndSpill(node->try_block()->statements());

  // Stop the introduced shadowing and count the number of required unlinks.
  // After shadowing stops, the original labels are unshadowed and the
  // LabelShadows represent the formerly shadowing labels.
  int nof_unlinks = 0;
  for (int i = 0; i < shadows.length(); i++) {
    shadows[i]->StopShadowing();
    if (shadows[i]->is_linked()) nof_unlinks++;
  }
  function_return_is_shadowed_ = function_return_was_shadowed;

  // Get an external reference to the handler address.
  ExternalReference handler_address(Top::k_handler_address);

  // If we can fall off the end of the try block, unlink from the try
  // chain and set the state on the frame to FALLING.
  if (has_valid_frame()) {
    // The next handler address is on top of the frame.
    ASSERT(StackHandlerConstants::kNextOffset == 0);
//    frame_->EmitPop(r1);
//    __ mov(r3, Operand(handler_address));
//    __ str(r1, MemOperand(r3));
//    frame_->Drop(StackHandlerConstants::kSize / kPointerSize - 1);
    frame_->EmitPop(a1);
    __ li(a3, Operand(handler_address));
    __ sw(a1, MemOperand(a3));
    frame_->Drop(StackHandlerConstants::kSize / kPointerSize - 1);

    // Fake a top of stack value (unneeded when FALLING) and set the
    // state in r2, then jump around the unlink blocks if any.
//    __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
//    frame_->EmitPush(r0);
//    __ mov(r2, Operand(Smi::FromInt(FALLING)));
    __ LoadRoot(a0, Heap::kUndefinedValueRootIndex);
    frame_->EmitPush(a0);
    __ li(a2, Operand(Smi::FromInt(FALLING)));
    if (nof_unlinks > 0) {
      finally_block.Jump();
      __ nop(); // NOP_ADDED
    }
  }

  // Generate code to unlink and set the state for the (formerly)
  // shadowing targets that have been jumped to.
  for (int i = 0; i < shadows.length(); i++) {
    if (shadows[i]->is_linked()) {
      // If we have come from the shadowed return, the return value is
      // in (a non-refcounted reference to) r0.  We must preserve it
      // until it is pushed.
      //
      // Because we can be jumping here (to spilled code) from
      // unspilled code, we need to reestablish a spilled frame at
      // this block.
      shadows[i]->Bind();
      frame_->SpillAll();

      // Reload sp from the top handler, because some statements that
      // we break from (eg, for...in) may have left stuff on the
      // stack.
//      __ mov(r3, Operand(handler_address));
//      __ ldr(sp, MemOperand(r3));
      __ li(a3, Operand(handler_address));
      __ lw(sp, MemOperand(a3));
      frame_->Forget(frame_->height() - handler_height);

      // Unlink this handler and drop it from the frame.  The next
      // handler address is currently on top of the frame.
      ASSERT(StackHandlerConstants::kNextOffset == 0);
      frame_->EmitPop(a1);
      __ sw(a1, MemOperand(a3));
      frame_->Drop(StackHandlerConstants::kSize / kPointerSize - 1);

      if (i == kReturnShadowIndex) {
        // If this label shadowed the function return, materialize the
        // return value on the stack.
        frame_->EmitPush(v0);
      } else {
        // Fake TOS for targets that shadowed breaks and continues.
        __ LoadRoot(a0, Heap::kUndefinedValueRootIndex);
        frame_->EmitPush(a0);
      }
//      __ mov(r2, Operand(Smi::FromInt(JUMPING + i)));
      if (--nof_unlinks > 0) {
        // If this is not the last unlink block, jump around the next.
        finally_block.Jump();
        __ nop(); // NOP_ADDED
      }
    }
  }

//  // --- Finally block ---
  finally_block.Bind();

  // Push the state on the stack.
//  frame_->EmitPush(r2);
  frame_->EmitPush(a2);

  // We keep two elements on the stack - the (possibly faked) result
  // and the state - while evaluating the finally block.
  //
  // Generate code for the statements in the finally block.
  VisitStatementsAndSpill(node->finally_block()->statements());

  if (has_valid_frame()) {
    // Restore state and return value or faked TOS.
//    frame_->EmitPop(r2);
//    frame_->EmitPop(r0);
    frame_->EmitPop(a2);
    frame_->EmitPop(a0);
  }

  // Generate code to jump to the right destination for all used
  // formerly shadowing targets.  Deallocate each shadow target.
  for (int i = 0; i < shadows.length(); i++) {
    if (has_valid_frame() && shadows[i]->is_bound()) {
      JumpTarget* original = shadows[i]->other_target();
//      __ cmp(r2, Operand(Smi::FromInt(JUMPING + i)));
      if (!function_return_is_shadowed_ && i == kReturnShadowIndex) {
        JumpTarget skip;
        skip.Branch(ne, no_hint, a2, Operand(Smi::FromInt(JUMPING + i)));
        __ nop(); // NOP_ADDED
        frame_->PrepareForReturn();
        original->Jump();
        __ nop(); // NOP_ADDED
        skip.Bind();
      } else {
        original->Branch(eq, no_hint, a2, Operand(Smi::FromInt(JUMPING + i)));
        __ nop(); // NOP_ADDED
      }
    }
  }

  if (has_valid_frame()) {
    // Check if we need to rethrow the exception.
    JumpTarget exit;
//    __ cmp(r2, Operand(Smi::FromInt(THROWING)));
    exit.Branch(ne, no_hint, a2, Operand(Smi::FromInt(THROWING)));
    __ nop(); // NOP_ADDED

    // Rethrow exception.
    frame_->EmitPush(a0);
    frame_->CallRuntime(Runtime::kReThrow, 1);

    // Done.
    exit.Bind();
  }
  ASSERT(!has_valid_frame() || frame_->height() == original_height);
}


void CodeGenerator::VisitDebuggerStatement(DebuggerStatement* node) {
  UNIMPLEMENTED();
  __ break_(0x00666);
  __ nop();
//#ifdef DEBUG
//  int original_height = frame_->height();
//#endif
//  VirtualFrame::SpilledScope spilled_scope;
//  Comment cmnt(masm_, "[ DebuggerStatament");
//  CodeForStatementPosition(node);
//#ifdef ENABLE_DEBUGGER_SUPPORT
//  frame_->CallRuntime(Runtime::kDebugBreak, 0);
//#endif
//  // Ignore the return value.
//  ASSERT(frame_->height() == original_height);
}


void CodeGenerator::InstantiateBoilerplate(Handle<JSFunction> boilerplate) {
#ifdef DEBUG
//  printf("CodeGenerator::InstantiateBoilerplate\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(boilerplate->IsBoilerplate());

  // Create a new closure.
  frame_->EmitPush(cp);
//  __ mov(r0, Operand(boilerplate));
//  frame_->EmitPush(r0);
  __ li(a0, Operand(boilerplate));
  frame_->EmitPush(a0);

  frame_->CallRuntime(Runtime::kNewClosure, 2);
  __ nop(); // NOP_ADDED
//  frame_->EmitPush(r0);
  frame_->EmitPush(v0);
}


void CodeGenerator::VisitFunctionLiteral(FunctionLiteral* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitFunctionLiteral\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ FunctionLiteral");

  // Build the function boilerplate and instantiate it.
  Handle<JSFunction> boilerplate =
      Compiler::BuildBoilerplate(node, script_, this);
  // Check for stack-overflow exception.
  if (HasStackOverflow()) {
    ASSERT(frame_->height() == original_height);
    return;
  }
  InstantiateBoilerplate(boilerplate);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitFunctionBoilerplateLiteral(
    FunctionBoilerplateLiteral* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitFunctionBoilerplateLiteral\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ FunctionBoilerplateLiteral");
  InstantiateBoilerplate(node->boilerplate());
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitConditional(Conditional* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitConditional\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Conditional");
  JumpTarget then;
  JumpTarget else_;
  LoadConditionAndSpill(node->condition(), &then, &else_, true);
  if (has_valid_frame()) {
    Branch(false, &else_);
    __ nop(); // NOP_ADDED
  }
  if (has_valid_frame() || then.is_linked()) {
    then.Bind();
    LoadAndSpill(node->then_expression());
  }
  if (else_.is_linked()) {
    JumpTarget exit;
    if (has_valid_frame()) {
      exit.Jump();
      __ nop(); // NOP_ADDED
    }
    else_.Bind();
    LoadAndSpill(node->else_expression());
    if (exit.is_linked()) exit.Bind();
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::LoadFromSlot(Slot* slot, TypeofState typeof_state) {
#ifdef DEBUG
//  printf("CodeGenerator::LoadFromSlot\n");
#endif

  VirtualFrame::SpilledScope spilled_scope;
  if (slot->type() == Slot::LOOKUP) {
    ASSERT(slot->var()->is_dynamic());

    JumpTarget slow;
    JumpTarget done;

    // Generate fast-case code for variables that might be shadowed by
    // eval-introduced variables.  Eval is used a lot without
    // introducing variables.  In those cases, we do not want to
    // perform a runtime call for all variables in the scope
    // containing the eval.
    if (slot->var()->mode() == Variable::DYNAMIC_GLOBAL) {
//      LoadFromGlobalSlotCheckExtensions(slot, typeof_state, r1, r2, &slow);
      LoadFromGlobalSlotCheckExtensions(slot, typeof_state, a1, a2, &slow);
      // If there was no control flow to slow, we can exit early.
      if (!slow.is_linked()) {
//        frame_->EmitPush(r0);
        frame_->EmitPush(a0);
        return;
      }

      done.Jump();
      __ nop(); // NOP_ADDED

    } else if (slot->var()->mode() == Variable::DYNAMIC_LOCAL) {
      __ break_(0x55555);
      Slot* potential_slot = slot->var()->local_if_not_shadowed()->slot();
      // Only generate the fast case for locals that rewrite to slots.
      // This rules out argument loads.
      if (potential_slot != NULL) {
        __ lw(a0,
               ContextSlotOperandCheckExtensions(potential_slot,
                                                 a1,
                                                 a2,
                                                 &slow));
        if (potential_slot->var()->mode() == Variable::CONST) {
          __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//          __ cmp(r0, ip);
          __ LoadRoot(a0, Heap::kUndefinedValueRootIndex, eq, a0, Operand(ip));
        }
        // There is always control flow to slow from
        // ContextSlotOperandCheckExtensions so we have to jump around
        // it.
        done.Jump();
        __ nop();   // NOP_ADDED
      }
    }

    slow.Bind();
    frame_->EmitPush(cp);
//    __ mov(r0, Operand(slot->var()->name()));
//    frame_->EmitPush(r0);
    __ li(a0, Operand(slot->var()->name()));
    frame_->EmitPush(a0);

    if (typeof_state == INSIDE_TYPEOF) {
      frame_->CallRuntime(Runtime::kLoadContextSlotNoReferenceError, 2);
    } else {
      frame_->CallRuntime(Runtime::kLoadContextSlot, 2);
    }
    __ nop();

    done.Bind();
    frame_->EmitPush(v0);

  } else {
    // Note: We would like to keep the assert below, but it fires because of
    // some nasty code in LoadTypeofExpression() which should be removed...
    // ASSERT(!slot->var()->is_dynamic());

    // Special handling for locals allocated in registers.
//    __ ldr(r0, SlotOperand(slot, r2));
    __ lw(a0, SlotOperand(slot, a2));
    frame_->EmitPush(a0);
    if (slot->var()->mode() == Variable::CONST) {
      // Const slots may contain 'the hole' value (the constant hasn't been
      // initialized yet) which needs to be converted into the 'undefined'
      // value.
      Comment cmnt(masm_, "[ Unhole const");
//      frame_->EmitPop(r0);
//      __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//      __ cmp(r0, ip);
//      __ LoadRoot(r0, Heap::kUndefinedValueRootIndex, eq);
//      frame_->EmitPush(r0);
      frame_->EmitPop(a0);
      __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
      __ LoadRoot(a0, Heap::kUndefinedValueRootIndex, eq, a0, Operand(ip));
      frame_->EmitPush(a0);
    }
  }
}


void CodeGenerator::LoadFromGlobalSlotCheckExtensions(Slot* slot,
                                                      TypeofState typeof_state,
                                                      Register tmp,
                                                      Register tmp2,
                                                      JumpTarget* slow) {
  UNIMPLEMENTED();
  __ break_(0x00666);
  __ nop();
  // Check that no extension objects have been created by calls to
//  // eval from the current scope to the global scope.
//  Register context = cp;
//  Scope* s = scope();
//  while (s != NULL) {
//    if (s->num_heap_slots() > 0) {
//      if (s->calls_eval()) {
//        // Check that extension is NULL.
//        __ ldr(tmp2, ContextOperand(context, Context::EXTENSION_INDEX));
//        __ tst(tmp2, tmp2);
//        slow->Branch(ne);
//      }
//      // Load next context in chain.
//      __ ldr(tmp, ContextOperand(context, Context::CLOSURE_INDEX));
//      __ ldr(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
//      context = tmp;
//    }
//    // If no outer scope calls eval, we do not need to check more
//    // context extensions.
//    if (!s->outer_scope_calls_eval() || s->is_eval_scope()) break;
//    s = s->outer_scope();
//  }
//
//  if (s->is_eval_scope()) {
//    Label next, fast;
//    if (!context.is(tmp)) {
//      __ mov(tmp, Operand(context));
//    }
//    __ bind(&next);
//    // Terminate at global context.
//    __ ldr(tmp2, FieldMemOperand(tmp, HeapObject::kMapOffset));
//    __ LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
//    __ cmp(tmp2, ip);
//    __ b(eq, &fast);
//    // Check that extension is NULL.
//    __ ldr(tmp2, ContextOperand(tmp, Context::EXTENSION_INDEX));
//    __ tst(tmp2, tmp2);
//    slow->Branch(ne);
//    // Load next context in chain.
//    __ ldr(tmp, ContextOperand(tmp, Context::CLOSURE_INDEX));
//    __ ldr(tmp, FieldMemOperand(tmp, JSFunction::kContextOffset));
//    __ b(&next);
//    __ bind(&fast);
//  }
//
//  // All extension objects were empty and it is safe to use a global
//  // load IC call.
//  Handle<Code> ic(Builtins::builtin(Builtins::LoadIC_Initialize));
//  // Load the global object.
//  LoadGlobal();
//  // Setup the name register.
//  Result name(r2);
//  __ mov(r2, Operand(slot->var()->name()));
//  // Call IC stub.
//  if (typeof_state == INSIDE_TYPEOF) {
//    frame_->CallCodeObject(ic, RelocInfo::CODE_TARGET, &name, 0);
//  } else {
//    frame_->CallCodeObject(ic, RelocInfo::CODE_TARGET_CONTEXT, &name, 0);
//  }
//
//  // Drop the global object. The result is in r0.
//  frame_->Drop();
}


void CodeGenerator::VisitSlot(Slot* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitSlot\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Slot");
  LoadFromSlot(node, typeof_state());
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitVariableProxy(VariableProxy* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitVariableProxy\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ VariableProxy");

  Variable* var = node->var();
  Expression* expr = var->rewrite();
  if (expr != NULL) {
    Visit(expr);
  } else {
    ASSERT(var->is_global());
    Reference ref(this, node);
    ref.GetValueAndSpill();
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitLiteral(Literal* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitLiteral\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Literal");
  __ li(t0, Operand(node->handle()));
  frame_->EmitPush(t0);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitRegExpLiteral(RegExpLiteral* node) {
  UNIMPLEMENTED();
  __ break_(0x00666);
  __ nop();
//#ifdef DEBUG
//  int original_height = frame_->height();
//#endif
//  VirtualFrame::SpilledScope spilled_scope;
//  Comment cmnt(masm_, "[ RexExp Literal");
//
//  // Retrieve the literal array and check the allocated entry.
//
//  // Load the function of this activation.
//  __ ldr(r1, frame_->Function());
//
//  // Load the literals array of the function.
//  __ ldr(r1, FieldMemOperand(r1, JSFunction::kLiteralsOffset));
//
//  // Load the literal at the ast saved index.
//  int literal_offset =
//      FixedArray::kHeaderSize + node->literal_index() * kPointerSize;
//  __ ldr(r2, FieldMemOperand(r1, literal_offset));
//
//  JumpTarget done;
//  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//  __ cmp(r2, ip);
//  done.Branch(ne);
//
//  // If the entry is undefined we call the runtime system to computed
//  // the literal.
//  frame_->EmitPush(r1);  // literal array  (0)
//  __ mov(r0, Operand(Smi::FromInt(node->literal_index())));
//  frame_->EmitPush(r0);  // literal index  (1)
//  __ mov(r0, Operand(node->pattern()));  // RegExp pattern (2)
//  frame_->EmitPush(r0);
//  __ mov(r0, Operand(node->flags()));  // RegExp flags   (3)
//  frame_->EmitPush(r0);
//  frame_->CallRuntime(Runtime::kMaterializeRegExpLiteral, 4);
//  __ mov(r2, Operand(r0));
//
//  done.Bind();
//  // Push the literal.
//  frame_->EmitPush(r2);
//  ASSERT(frame_->height() == original_height + 1);
}


// This deferred code stub will be used for creating the boilerplate
// by calling Runtime_CreateObjectLiteralBoilerplate.
// Each created boilerplate is stored in the JSFunction and they are
// therefore context dependent.
class DeferredObjectLiteral: public DeferredCode {
 public:
  explicit DeferredObjectLiteral(ObjectLiteral* node) : node_(node) {
    set_comment("[ DeferredObjectLiteral");
  }

  virtual void Generate();

 private:
  ObjectLiteral* node_;
};


void DeferredObjectLiteral::Generate() {
  // Argument is passed in r1.

  // If the entry is undefined we call the runtime system to compute
  // the literal.
  // Literal array (0).
//  __ push(r1);
  __ push(a1);
  // Literal index (1).
//  __ mov(r0, Operand(Smi::FromInt(node_->literal_index())));
//  __ push(r0);
  __ li(t0, Operand(Smi::FromInt(node_->literal_index())));
  __ push(t0);
  // Constant properties (2).
//  __ mov(r0, Operand(node_->constant_properties()));
//  __ push(r0);
  __ li(t0, Operand(node_->constant_properties()));
  __ push(t0);
  __ CallRuntime(Runtime::kCreateObjectLiteralBoilerplate, 3);
  __ nop(); // NOP_ADDED
//  __ mov(r2, Operand(r0));
  __ mov(a2, v0);
//  // Result is returned in r2.
}


void CodeGenerator::VisitObjectLiteral(ObjectLiteral* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitObjectLiteral\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ObjectLiteral");

  DeferredObjectLiteral* deferred = new DeferredObjectLiteral(node);

  // Retrieve the literal array and check the allocated entry.

  // Load the function of this activation.
//  __ ldr(r1, frame_->Function());
  __ lw(a1, frame_->Function());

  // Load the literals array of the function.
//  __ ldr(r1, FieldMemOperand(r1, JSFunction::kLiteralsOffset));
  __ lw(a1, FieldMemOperand(a1, JSFunction::kLiteralsOffset));

  // Load the literal at the ast saved index.
  int literal_offset =
      FixedArray::kHeaderSize + node->literal_index() * kPointerSize;
//  __ ldr(r2, FieldMemOperand(r1, literal_offset));
  __ lw(a2, FieldMemOperand(a1, literal_offset));

  // Check whether we need to materialize the object literal boilerplate.
  // If so, jump to the deferred code.
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//  __ cmp(r2, Operand(ip));
//  deferred->Branch(eq);
  deferred->Branch(eq, a2, Operand(ip));
  __ nop(); // NOP_ADDED
  deferred->BindExit();

  // Push the object literal boilerplate.
//  frame_->EmitPush(r2);
  frame_->EmitPush(a2);

  // Clone the boilerplate object.
  Runtime::FunctionId clone_function_id = Runtime::kCloneLiteralBoilerplate;
  if (node->depth() == 1) {
    clone_function_id = Runtime::kCloneShallowLiteralBoilerplate;
  }
  frame_->CallRuntime(clone_function_id, 1);
  __ nop(); // NOP_ADDED
//  frame_->EmitPush(r0);  // save the result
  frame_->EmitPush(v0);  // save the result
  // r0: cloned object literal

  for (int i = 0; i < node->properties()->length(); i++) {
    ObjectLiteral::Property* property = node->properties()->at(i);
    Literal* key = property->key();
    Expression* value = property->value();
    switch (property->kind()) {
      case ObjectLiteral::Property::CONSTANT:
        break;
      case ObjectLiteral::Property::MATERIALIZED_LITERAL:
        if (CompileTimeValue::IsCompileTimeValue(property->value())) break;
        // else fall through
      case ObjectLiteral::Property::COMPUTED:  // fall through
      case ObjectLiteral::Property::PROTOTYPE: {
//        frame_->EmitPush(r0);  // dup the result
        frame_->EmitPush(v0);  // dup the result
        LoadAndSpill(key);
        LoadAndSpill(value);
        frame_->CallRuntime(Runtime::kSetProperty, 3);
        __ nop();   // NOP_ADDED
        // restore r0
//        __ ldr(r0, frame_->Top());
        __ lw(v0, frame_->Top());
        break;
      }
      case ObjectLiteral::Property::SETTER: {
//        frame_->EmitPush(r0);
        frame_->EmitPush(v0);
        LoadAndSpill(key);
//        __ mov(r0, Operand(Smi::FromInt(1)));
        __ li(a0, Operand(Smi::FromInt(1)));
//        frame_->EmitPush(r0);
        frame_->EmitPush(a0);
        LoadAndSpill(value);
        frame_->CallRuntime(Runtime::kDefineAccessor, 4);
        __ nop();   // NOP_ADDED
//        __ ldr(r0, frame_->Top());
        __ lw(v0, frame_->Top());
        break;
      }
      case ObjectLiteral::Property::GETTER: {
//        frame_->EmitPush(r0);
        frame_->EmitPush(v0);
//        LoadAndSpill(key);
//        __ mov(r0, Operand(Smi::FromInt(0)));
        __ li(a0, Operand(Smi::FromInt(0)));
//        frame_->EmitPush(r0);
        frame_->EmitPush(a0);
        LoadAndSpill(value);
        frame_->CallRuntime(Runtime::kDefineAccessor, 4);
        __ nop();   // NOP_ADDED
//        __ ldr(r0, frame_->Top());
        __ lw(v0, frame_->Top());
        break;
      }
    }
  }
  ASSERT(frame_->height() == original_height + 1);
}


// This deferred code stub will be used for creating the boilerplate
// by calling Runtime_CreateArrayLiteralBoilerplate.
// Each created boilerplate is stored in the JSFunction and they are
// therefore context dependent.
class DeferredArrayLiteral: public DeferredCode {
 public:
  explicit DeferredArrayLiteral(ArrayLiteral* node) : node_(node) {
    set_comment("[ DeferredArrayLiteral");
  }

  virtual void Generate();

 private:
  ArrayLiteral* node_;
};


void DeferredArrayLiteral::Generate() {
  // Argument is passed in a1.

  // If the entry is undefined we call the runtime system to compute
  // the literal.
  // Literal array (0).
//  __ push(r1);
  __ push(a1);
  // Literal index (1).
//  __ mov(r0, Operand(Smi::FromInt(node_->literal_index())));
//  __ push(r0);
  __ li(a0, Operand(Smi::FromInt(node_->literal_index())));
  __ push(a0);
  // Constant properties (2).
//  __ mov(r0, Operand(node_->literals()));
//  __ push(r0);
  __ li(a0, Operand(node_->literals()));
  __ push(a0);
  __ CallRuntime(Runtime::kCreateArrayLiteralBoilerplate, 3);
  __ nop(); // NOP_ADDED
//  __ mov(r2, Operand(r0));
  __ mov(a2, v0);
  // Result is returned in a2.
}


void CodeGenerator::VisitArrayLiteral(ArrayLiteral* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitArrayLiteral\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ ArrayLiteral");

  DeferredArrayLiteral* deferred = new DeferredArrayLiteral(node);

  // Retrieve the literal array and check the allocated entry.

  // Load the function of this activation.
//  __ ldr(r1, frame_->Function());
  __ lw(a1, frame_->Function());

  // Load the literals array of the function.
//  __ ldr(r1, FieldMemOperand(r1, JSFunction::kLiteralsOffset));
  __ lw(a1, FieldMemOperand(a1, JSFunction::kLiteralsOffset));

  // Load the literal at the ast saved index.
  int literal_offset =
      FixedArray::kHeaderSize + node->literal_index() * kPointerSize;
//  __ ldr(r2, FieldMemOperand(r1, literal_offset));
  __ lw(a2, FieldMemOperand(a1, literal_offset));

  // Check whether we need to materialize the object literal boilerplate.
  // If so, jump to the deferred code.
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//  __ cmp(r2, Operand(ip));
//  deferred->Branch(eq);
  deferred->Branch(eq, a2, Operand(ip));
  __ nop(); // NOP_ADDED
  deferred->BindExit();

  // Push the object literal boilerplate.
//  frame_->EmitPush(r2);
  frame_->EmitPush(a2);

  // Clone the boilerplate object.
  Runtime::FunctionId clone_function_id = Runtime::kCloneLiteralBoilerplate;
  if (node->depth() == 1) {
    clone_function_id = Runtime::kCloneShallowLiteralBoilerplate;
  }

  frame_->CallRuntime(clone_function_id, 1);
  __ nop(); // NOP_ADDED

  frame_->EmitPush(v0);  // save the result
  // v0: cloned object literal

  // Generate code to set the elements in the array that are not
  // literals.
  for (int i = 0; i < node->values()->length(); i++) {
    Expression* value = node->values()->at(i);

    // If value is a literal the property value is already set in the
    // boilerplate object.
    if (value->AsLiteral() != NULL) continue;
    // If value is a materialized literal the property value is already set
    // in the boilerplate object if it is simple.
    if (CompileTimeValue::IsCompileTimeValue(value)) continue;
//
    // The property must be set by generated code.
    LoadAndSpill(value);
    frame_->EmitPop(a0);

    // Fetch the object literal.
//    __ ldr(r1, frame_->Top());
    __ lw(a1, frame_->Top());
    // Get the elements array.
//    __ ldr(r1, FieldMemOperand(r1, JSObject::kElementsOffset));
    __ lw(a1, FieldMemOperand(a1, JSObject::kElementsOffset));

    // Write to the indexed properties array.
    int offset = i * kPointerSize + FixedArray::kHeaderSize;
//    __ str(r0, FieldMemOperand(r1, offset));
    __ sw(a0, FieldMemOperand(a1, offset));
//
    // Update the write barrier for the array address.
//    __ mov(r3, Operand(offset));
//    __ RecordWrite(r1, r3, r2);
    __ li(a3, Operand(offset));
    __ RecordWrite(a1, a3, a2);
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitCatchExtensionObject(CatchExtensionObject* node) {
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  // Call runtime routine to allocate the catch extension object and
  // assign the exception value to the catch variable.
  Comment cmnt(masm_, "[ CatchExtensionObject");
  LoadAndSpill(node->key());
  LoadAndSpill(node->value());
  frame_->CallRuntime(Runtime::kCreateCatchExtensionObject, 2);
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitAssignment(Assignment* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitAssignment\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Assignment");

  { Reference target(this, node->target());
    if (target.is_illegal()) {
      // Fool the virtual frame into thinking that we left the assignment's
      // value on the frame.
      __ li(a0, Operand(Smi::FromInt(0)));
      frame_->EmitPush(a0);
      ASSERT(frame_->height() == original_height + 1);
      return;
    }

    if (node->op() == Token::ASSIGN ||
        node->op() == Token::INIT_VAR ||
        node->op() == Token::INIT_CONST) {
      LoadAndSpill(node->value());

    } else {
      // +=, *= and similar binary assignments.
      // Get the old value of the lhs.
      target.GetValueAndSpill();
      Literal* literal = node->value()->AsLiteral();
      bool overwrite =
          (node->value()->AsBinaryOperation() != NULL &&
           node->value()->AsBinaryOperation()->ResultOverwriteAllowed());
      if (literal != NULL && literal->handle()->IsSmi()) {
        SmiOperation(node->binary_op(),
                     literal->handle(),
                     false,
                     overwrite ? OVERWRITE_RIGHT : NO_OVERWRITE);
        frame_->EmitPush(v0);

      } else {
        LoadAndSpill(node->value());
        GenericBinaryOperation(node->binary_op(),
                               overwrite ? OVERWRITE_RIGHT : NO_OVERWRITE);
        frame_->EmitPush(v0);
      }
    }

    Variable* var = node->target()->AsVariableProxy()->AsVariable();
    if (var != NULL &&
        (var->mode() == Variable::CONST) &&
        node->op() != Token::INIT_VAR && node->op() != Token::INIT_CONST) {
      // Assignment ignored - leave the value on the stack.

    } else {
      CodeForSourcePosition(node->position());
      if (node->op() == Token::INIT_CONST) {
        // Dynamic constant initializations must use the function context
        // and initialize the actual constant declared. Dynamic variable
        // initializations are simply assignments and use SetValue.
        target.SetValue(CONST_INIT);
      } else {
        target.SetValue(NOT_CONST_INIT);
      }
    }
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitThrow(Throw* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitThrow\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Throw");

  LoadAndSpill(node->exception());
  CodeForSourcePosition(node->position());
  frame_->CallRuntime(Runtime::kThrow, 1);
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitProperty(Property* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitProperty\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Property");

  { Reference property(this, node);
    property.GetValueAndSpill();
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitCall(Call* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitCall\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ Call");

  Expression* function = node->expression();
  ZoneList<Expression*>* args = node->arguments();
  int arg_count = args->length();

  // Standard function call.
  // Check if the function is a variable or a property.
  Variable* var = function->AsVariableProxy()->AsVariable();
  Property* property = function->AsProperty();

  // ------------------------------------------------------------------------
  // Fast-case: Use inline caching.
  // ---
  // According to ECMA-262, section 11.2.3, page 44, the function to call
  // must be resolved after the arguments have been evaluated. The IC code
  // automatically handles this by loading the arguments before the function
  // is resolved in cache misses (this also holds for megamorphic calls).
  // ------------------------------------------------------------------------

  if (var != NULL && var->is_possibly_eval()) {
#ifdef DEBUG
//    printf("(var != NULL && var->is_possibly_eval())\n");
#endif
    // ----------------------------------
    // JavaScript example: 'eval(arg)'  // eval is not known to be shadowed
    // ----------------------------------

    __ break_(0x3378);  // never tested

    // In a call to eval, we first call %ResolvePossiblyDirectEval to
    // resolve the function we need to call and the receiver of the
    // call.  Then we call the resolved function using the given
    // arguments.
    // Prepare stack for call to resolved function.

    __ SetupAlignedCall(t0, arg_count);

    LoadAndSpill(function);
    __ LoadRoot(t2, Heap::kUndefinedValueRootIndex);
    frame_->EmitPush(t2);  // Slot for receiver

    for (int i = 0; i < arg_count; i++) {
      LoadAndSpill(args->at(i));
    }

    // Prepare stack for call to ResolvePossiblyDirectEval.
    __ lw(a1, MemOperand(sp, arg_count * kPointerSize + kPointerSize));
    frame_->EmitPush(a1);
    if (arg_count > 0) {
      __ lw(a1, MemOperand(sp, arg_count * kPointerSize));
      frame_->EmitPush(a1);
    } else {
      frame_->EmitPush(t2);
    }

    // Resolve the call.
    frame_->CallRuntime(Runtime::kResolvePossiblyDirectEval, 2);
    __ nop(); // NOP_ADDED

    // On return we do not use ReturnFromAlignedCall() because we will call the
    // resolved function below. Instead we remove the 2 extra args pushed on the
    // stack.

// TOCHECK: Was here for fixup ?
//    __ addiu(sp, sp, Operand(4));

    // Touch up stack with the right values for the function and the receiver.
    __ lw(a1, FieldMemOperand(a0, FixedArray::kHeaderSize));
    __ sw(a1, MemOperand(sp, (arg_count + 1) * kPointerSize));
    __ lw(a1, FieldMemOperand(a0, FixedArray::kHeaderSize + kPointerSize));
    __ sw(a1, MemOperand(sp, arg_count * kPointerSize));

    // Call the function.
    CodeForSourcePosition(node->position());

    InLoopFlag in_loop = loop_nesting() > 0 ? IN_LOOP : NOT_IN_LOOP;
    CallFunctionStub call_function(arg_count, in_loop);
    frame_->CallStub(&call_function, arg_count + 1);
    __ nop(); // NOP_ADDED

    __ ReturnFromAlignedCall(); 

    __ lw(cp, frame_->Context());
    frame_->EmitPush(v0);

//    __ ldr(cp, frame_->Context());
//    // Remove the function from the stack.
//    frame_->Drop();
//    frame_->EmitPush(r0);

  } else if (var != NULL && !var->is_this() && var->is_global()) {
#ifdef DEBUG
//    printf("(var != NULL && !var->is_this() && var->is_global())\n");
#endif
    // ----------------------------------
    // JavaScript example: 'foo(1, 2, 3)'  // foo is global
    // ----------------------------------


    // We need sp to be 8 bytes aligned when calling the stub.
    __ SetupAlignedCall(t3, arg_count);

    // Push the name of the function and the receiver onto the stack.
    __ li(a0, Operand(var->name()));
    frame_->EmitPush(a0);

    // Pass the global object as the receiver and let the IC stub
    // patch the stack to use the global proxy as 'this' in the
    // invoked function.
    LoadGlobal();

    // Load the arguments.
    for (int i = 0; i < arg_count; i++) {
      LoadAndSpill(args->at(i));
    }


    // Setup the receiver register and call the IC initialization code.
    InLoopFlag in_loop = loop_nesting() > 0 ? IN_LOOP : NOT_IN_LOOP;
    Handle<Code> stub = ComputeCallInitialize(arg_count, in_loop);
    CodeForSourcePosition(node->position());
    frame_->CallCodeObject(stub, RelocInfo::CODE_TARGET_CONTEXT,
                           arg_count + 1);
//    __ addiu(sp, sp, Operand(-StandardFrameConstants::kRArgsSlotsSize));
    __ nop();
    __ ReturnFromAlignedCall();
    __ lw(cp, frame_->Context());
    // Remove the function from the stack.
    frame_->DropFromVFrameOnly();
    frame_->EmitPush(v0);

  } else if (var != NULL && var->slot() != NULL &&
             var->slot()->type() == Slot::LOOKUP) {
#ifdef DEBUG
//    printf("(var != NULL && var->slot() != NULL &&var->slot()->type() == Slot::LOOKUP)\n");
#endif
    // ----------------------------------
    // JavaScript example: 'with (obj) foo(1, 2, 3)'  // foo is in obj
    // ----------------------------------

    // Load the function
//    frame_->EmitPush(cp);
//    __ mov(r0, Operand(var->name()));
//    frame_->EmitPush(r0);
//    frame_->CallRuntime(Runtime::kLoadContextSlot, 2);
//    // r0: slot value; r1: receiver
    frame_->EmitPush(cp);
    __ li(a0, Operand(var->name()));
    frame_->EmitPush(a0);
    frame_->CallRuntime(Runtime::kLoadContextSlot, 2);
    __ nop();
    // r0: slot value; r1: receiver

    // Load the receiver.
//    frame_->EmitPush(r0);  // function
//    frame_->EmitPush(r1);  // receiver
    frame_->EmitPush(a0);  // function
    frame_->EmitPush(a1);  // receiver

    // Call the function.
    CallWithArguments(args, node->position());
    __ nop(); // NOP_ADDED
    frame_->EmitPush(v0);

  } else if (property != NULL) {
#ifdef DEBUG
//    printf("(property != NULL)\n");
#endif
    // Check if the key is a literal string.
    Literal* literal = property->key()->AsLiteral();

    if (literal != NULL && literal->handle()->IsSymbol()) {
      // ------------------------------------------------------------------
      // JavaScript example: 'object.foo(1, 2, 3)' or 'map["key"](1, 2, 3)'
      // ------------------------------------------------------------------

      __ SetupAlignedCall(t2, arg_count);

      // Push the name of the function and the receiver onto the stack.
//      __ mov(r0, Operand(literal->handle()));
//      frame_->EmitPush(r0);
      __ li(a0, Operand(literal->handle()));
      frame_->EmitPush(a0);
      LoadAndSpill(property->obj());

      // Load the arguments.
      for (int i = 0; i < arg_count; i++) {
        LoadAndSpill(args->at(i));
      }

      // Set the receiver register and call the IC initialization code.
      InLoopFlag in_loop = loop_nesting() > 0 ? IN_LOOP : NOT_IN_LOOP;
      Handle<Code> stub = ComputeCallInitialize(arg_count, in_loop);
      CodeForSourcePosition(node->position());
      frame_->CallCodeObject(stub, RelocInfo::CODE_TARGET, arg_count + 1);
//      __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize); // branch delay
      __ nop();
      __ ReturnFromAlignedCall(); 

      __ lw(cp, frame_->Context());

      // Remove the function from the stack.
      frame_->DropFromVFrameOnly();

      frame_->EmitPush(v0);


    } else {
#ifdef DEBUG
//    printf("else\n");
#endif
      // -------------------------------------------
      // JavaScript example: 'array[index](1, 2, 3)'
      // -------------------------------------------

      __ SetupAlignedCall(t3, arg_count);

      // Load the function to call from the property through a reference.
      Reference ref(this, property);
      ref.GetValueAndSpill();  // receiver

      // Pass receiver to called function.
      if (property->is_synthetic()) {
        LoadGlobalReceiver(a0);
      } else {
        __ lw(a0, frame_->ElementAt(ref.size()));
        frame_->EmitPush(a0);
      }

      // Call the function (and allocate args slots).
      CallWithArguments(args, node->position());
      __ ReturnFromAlignedCall(); 
      
      __ lw(cp, frame_->Context());
      frame_->DropFromVFrameOnly(); // discard the TOS

      frame_->EmitPush(v0);
    }
  } else {
#ifdef DEBUG
//    printf("else2\n");
#endif
    // ----------------------------------
    // JavaScript example: 'foo(1, 2, 3)'  // foo is not global
    // ----------------------------------

    __ SetupAlignedCall(t1, arg_count);

    // Load the function.
    LoadAndSpill(function);

    // Pass the global proxy as the receiver.
    LoadGlobalReceiver(a0);

    // Call the function (and allocate args slots).
    CallWithArguments(args, node->position());
    __ ReturnFromAlignedCall(); 

    __ lw(cp, frame_->Context());
    frame_->DropFromVFrameOnly(); // discard the TOS

    frame_->EmitPush(v0);
  }

  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitCallNew(CallNew* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitCallNew\n");
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ CallNew");

  // According to ECMA-262, section 11.2.2, page 44, the function
  // expression in new calls must be evaluated before the
  // arguments. This is different from ordinary calls, where the
  // actual function to call is resolved after the arguments have been
  // evaluated.

  // Setup the stack
  ZoneList<Expression*>* args = node->arguments();
  int arg_count = args->length();

  __ push(s3);                        // Save s3 on the stack
  __ mov(s3, sp);                     // Save sp
  __ li(t0, Operand(~7));             // Load sp mask
  __ and_(sp, sp, Operand(t0));       // Align sp.
//  __ break_(0x3648);

    // We are going to push (arg_count + 2)*4 on the stack. We make sure sp will
    // be 8 bytes aligned after this.
  if( (arg_count % 2) != 0) {
    __ addiu(sp, sp, -4);
  }

  // Compute function to call and use the global object as the
  // receiver. There is no need to use the global proxy here because
  // it will always be replaced with a newly allocated object.
  LoadAndSpill(node->expression());
  LoadGlobal();

  // Push the arguments ("left-to-right") on the stack.
  for (int i = 0; i < arg_count; i++) {
    LoadAndSpill(args->at(i));
  }

  // r0: the number of arguments.
//  Result num_args(r0);
  Result num_args(a0);
//  __ mov(r0, Operand(arg_count));
  __ li(a0, Operand(arg_count));

  // Load the function into r1 as per calling convention.
//  Result function(r1);
//  __ ldr(r1, frame_->ElementAt(arg_count + 1));
  Result function(a1);
  __ lw(a1, frame_->ElementAt(arg_count + 1));

  // Call the construct call builtin that handles allocation and
  // constructor invocation.
  CodeForSourcePosition(node->position());
  Handle<Code> ic(Builtins::builtin(Builtins::JSConstructCall));
  frame_->CallCodeObject(ic,
                         RelocInfo::CONSTRUCT_CALL,
                         &num_args,
                         &function,
                         arg_count + 1,
                         true); // Special handling of args slots
//  __ addiu(sp, sp, Operand(-StandardFrameConstants::kRArgsSlotsSize));
  __ nop();
  __ mov(sp, s3);                     // Restore sp.
  __ teq(fp, zero_reg, 0x122);   // debug help
  __ pop(s3);                         // Restore s3


  // Discard old TOS value and push r0 on the stack (same as Pop(), push(r0)).
//  __ str(r0, frame_->Top());
//  __ sw(v0, frame_->Top());
  __ push(v0);
  ASSERT(frame_->height() == original_height + 1);
//  __ break_(0x04309);
}


void CodeGenerator::GenerateClassOf(ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateClassOf\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 1);
  JumpTarget leave, null, function, non_function_constructor;

//  // Load the object into r0.
  LoadAndSpill(args->at(0));
//  frame_->EmitPop(r0);
  frame_->EmitPop(a0);

  // If the object is a smi, we return null.
//  __ tst(r0, Operand(kSmiTagMask));
//  null.Branch(eq);
  __ andi(t0, a0, Operand(kSmiTagMask));
  null.Branch(eq, no_hint, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Check that the object is a JS object but take special care of JS
  // functions to make sure they have 'Function' as their class.
//  __ CompareObjectType(r0, r0, r1, FIRST_JS_OBJECT_TYPE);
//  null.Branch(lt);
  __ GetObjectType(a0, a0, a1);
  null.Branch(less, no_hint, a1, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED

  // As long as JS_FUNCTION_TYPE is the last instance type and it is
  // right after LAST_JS_OBJECT_TYPE, we can avoid checking for
  // LAST_JS_OBJECT_TYPE.
  ASSERT(LAST_TYPE == JS_FUNCTION_TYPE);
  ASSERT(JS_FUNCTION_TYPE == LAST_JS_OBJECT_TYPE + 1);
//  __ cmp(r1, Operand(JS_FUNCTION_TYPE));
//  function.Branch(eq);
  function.Branch(eq, no_hint, a1, Operand(JS_FUNCTION_TYPE));

  // Check if the constructor in the map is a function.
//  __ ldr(r0, FieldMemOperand(r0, Map::kConstructorOffset));
//  __ CompareObjectType(r0, r1, r1, JS_FUNCTION_TYPE);
//  non_function_constructor.Branch(ne);
  __ lw(a0, FieldMemOperand(a0, Map::kConstructorOffset));
  __ GetObjectType(a0, a1, a1);
  non_function_constructor.Branch(ne, no_hint, a1, Operand(JS_FUNCTION_TYPE));

  // The r0 register now contains the constructor function. Grab the
  // instance class name from there.
//  __ ldr(r0, FieldMemOperand(r0, JSFunction::kSharedFunctionInfoOffset));
//  __ ldr(r0, FieldMemOperand(r0, SharedFunctionInfo::kInstanceClassNameOffset));
//  frame_->EmitPush(r0);
  __ lw(a0, FieldMemOperand(a0, JSFunction::kSharedFunctionInfoOffset));
  __ lw(v0, FieldMemOperand(a0, SharedFunctionInfo::kInstanceClassNameOffset));
  frame_->EmitPush(v0);
  leave.Jump();
  __ nop(); // NOP_ADDED

  // Functions have class 'Function'.
  function.Bind();
//  __ mov(r0, Operand(Factory::function_class_symbol()));
//  frame_->EmitPush(r0);
  __ li(v0, Operand(Factory::function_class_symbol()));
  frame_->EmitPush(v0);
  leave.Jump();
  __ nop(); // NOP_ADDED

  // Objects with a non-function constructor have class 'Object'.
  non_function_constructor.Bind();
//  __ mov(r0, Operand(Factory::Object_symbol()));
//  frame_->EmitPush(r0);
  __ li(v0, Operand(Factory::Object_symbol()));
  frame_->EmitPush(v0);
  leave.Jump();
  __ nop(); // NOP_ADDED

  // Non-JS objects have class null.
  null.Bind();
//  __ LoadRoot(r0, Heap::kNullValueRootIndex);
//  frame_->EmitPush(r0);
  __ LoadRoot(v0, Heap::kNullValueRootIndex);
  frame_->EmitPush(v0);

  // All done.
  leave.Bind();
}


void CodeGenerator::GenerateValueOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
  __ break_(0x00666);
//  VirtualFrame::SpilledScope spilled_scope;
//  ASSERT(args->length() == 1);
//  JumpTarget leave;
//  LoadAndSpill(args->at(0));
//  frame_->EmitPop(r0);  // r0 contains object.
//  // if (object->IsSmi()) return the object.
//  __ tst(r0, Operand(kSmiTagMask));
//  leave.Branch(eq);
//  // It is a heap object - get map. If (!object->IsJSValue()) return the object.
//  __ CompareObjectType(r0, r1, r1, JS_VALUE_TYPE);
//  leave.Branch(ne);
//  // Load the value.
//  __ ldr(r0, FieldMemOperand(r0, JSValue::kValueOffset));
//  leave.Bind();
//  frame_->EmitPush(r0);
}


void CodeGenerator::GenerateSetValueOf(ZoneList<Expression*>* args) {
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 2);
  JumpTarget leave;
  LoadAndSpill(args->at(0));  // Load the object.
  LoadAndSpill(args->at(1));  // Load the value.
  frame_->EmitPop(a0);  // r0 contains value
  frame_->EmitPop(a1);  // r1 contains object
  // if (object->IsSmi()) return object.
//  __ tst(r1, Operand(kSmiTagMask));
  __ andi(t1, a1, Operand(kSmiTagMask));
  leave.Branch(eq, no_hint, t1, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  // It is a heap object - get map. If (!object->IsJSValue()) return the object.
//  __ CompareObjectType(r1, r2, r2, JS_VALUE_TYPE);
  __ GetObjectType(a1, a2, a2);
  leave.Branch(ne, no_hint, a2, Operand(JS_VALUE_TYPE));
  __ nop(); // NOP_ADDED
  // Store the value.
//  __ str(r0, FieldMemOperand(r1, JSValue::kValueOffset));
  __ sw(v0, FieldMemOperand(a1, JSValue::kValueOffset));
  // Update the write barrier.
//  __ mov(r2, Operand(JSValue::kValueOffset - kHeapObjectTag));
//  __ RecordWrite(r1, r2, r3);
  __ li(a2, Operand(JSValue::kValueOffset - kHeapObjectTag));
  __ RecordWrite(a1, a2, a3);
  // Leave.
  leave.Bind();
  frame_->EmitPush(v0);
}


void CodeGenerator::GenerateIsSmi(ZoneList<Expression*>* args) {
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 1);
  LoadAndSpill(args->at(0));
  frame_->EmitPop(t0);
//  __ tst(r0, Operand(kSmiTagMask));
  __ andi(s5, t0, Operand(kSmiTagMask));
  __ mov(s6, zero_reg);
  cc_reg_ = eq;
}


void CodeGenerator::GenerateLog(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
  __ break_(0x00666);
//  VirtualFrame::SpilledScope spilled_scope;
//  // See comment in CodeGenerator::GenerateLog in codegen-ia32.cc.
//  ASSERT_EQ(args->length(), 3);
//#ifdef ENABLE_LOGGING_AND_PROFILING
//  if (ShouldGenerateLog(args->at(0))) {
//    LoadAndSpill(args->at(1));
//    LoadAndSpill(args->at(2));
//    __ CallRuntime(Runtime::kLog, 2);
//  }
//#endif
//  __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
//  frame_->EmitPush(r0);
}


void CodeGenerator::GenerateIsNonNegativeSmi(ZoneList<Expression*>* args) {
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 1);
  LoadAndSpill(args->at(0));
//  frame_->EmitPop(r0);
//  __ tst(r0, Operand(kSmiTagMask | 0x80000000u));
  frame_->EmitPop(a0);
  __ andi(s5, a0, Operand(kSmiTagMask | 0x80000000u));
  __ li(s6, Operand(0));
  cc_reg_ = eq;
}


// This should generate code that performs a charCodeAt() call or returns
// undefined in order to trigger the slow case, Runtime_StringCharCodeAt.
// It is not yet implemented on ARM, so it always goes to the slow case.
void CodeGenerator::GenerateFastCharCodeAt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
  __ break_(0x00666);
//  VirtualFrame::SpilledScope spilled_scope;
//  ASSERT(args->length() == 2);
//  __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
//  frame_->EmitPush(r0);
}


void CodeGenerator::GenerateIsArray(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
  __ break_(0x00666);
//  VirtualFrame::SpilledScope spilled_scope;
//  ASSERT(args->length() == 1);
//  LoadAndSpill(args->at(0));
//  JumpTarget answer;
//  // We need the CC bits to come out as not_equal in the case where the
//  // object is a smi.  This can't be done with the usual test opcode so
//  // we use XOR to get the right CC bits.
//  frame_->EmitPop(r0);
//  __ and_(r1, r0, Operand(kSmiTagMask));
//  __ eor(r1, r1, Operand(kSmiTagMask), SetCC);
//  answer.Branch(ne);
//  // It is a heap object - get the map. Check if the object is a JS array.
//  __ CompareObjectType(r0, r1, r1, JS_ARRAY_TYPE);
//  answer.Bind();
//  cc_reg_ = eq;
}


void CodeGenerator::GenerateIsConstructCall(ZoneList<Expression*>* args) {
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 0);

  // Get the frame pointer for the calling frame.
//  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ lw(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));

  // Skip the arguments adaptor frame if it exists.
  Label check_frame_marker;
//  __ ldr(r1, MemOperand(r2, StandardFrameConstants::kContextOffset));
//  __ cmp(r1, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
//  __ b(ne, &check_frame_marker);
//  __ ldr(r2, MemOperand(r2, StandardFrameConstants::kCallerFPOffset));
  __ lw(a1, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ bcond(ne, &check_frame_marker,
              a1, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ nop(); // NOP_ADDED
  __ lw(a2, MemOperand(a2, StandardFrameConstants::kCallerFPOffset));

  // Check the marker in the calling frame.
  __ bind(&check_frame_marker);
//  __ ldr(r1, MemOperand(r2, StandardFrameConstants::kMarkerOffset));
//  __ cmp(r1, Operand(Smi::FromInt(StackFrame::CONSTRUCT)));
  __ lw(s5, MemOperand(a2, StandardFrameConstants::kMarkerOffset));
  __ li(s6, Operand(Smi::FromInt(StackFrame::CONSTRUCT)));
  cc_reg_ = eq;
}


void CodeGenerator::GenerateArgumentsLength(ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateArgumentsLength\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 0);

  // Seed the result with the formal parameters count, which will be used
  // in case no arguments adaptor frame is found below the current frame.
//  __ mov(r0, Operand(Smi::FromInt(scope_->num_parameters())));
  __ li(a0, Operand(Smi::FromInt(scope_->num_parameters())));
//
  // Call the shared stub to get to the arguments.length.
  ArgumentsAccessStub stub(ArgumentsAccessStub::READ_LENGTH);
  frame_->CallStub(&stub, 0);
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
}


void CodeGenerator::GenerateArgumentsAccess(ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateArgumentsAccess\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 1);

  // Satisfy contract with ArgumentsAccessStub:
  // Load the key into r1 and the formal parameters count into r0.
  LoadAndSpill(args->at(0));
//  frame_->EmitPop(r1);
//  __ mov(r0, Operand(Smi::FromInt(scope_->num_parameters())));
  frame_->EmitPop(a1);
  __ li(a0, Operand(Smi::FromInt(scope_->num_parameters())));

  // Call the shared stub to get to arguments[key].
  ArgumentsAccessStub stub(ArgumentsAccessStub::READ_ELEMENT);
  frame_->CallStub(&stub, 0);
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
}


void CodeGenerator::GenerateRandomPositiveSmi(ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateRandomPositiveSmi\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 0);
  __ Call(ExternalReference::random_positive_smi_function().address(),
          RelocInfo::RUNTIME_ENTRY);
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
}


void CodeGenerator::GenerateFastMathOp(MathOp op, ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateFastMathOp\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  LoadAndSpill(args->at(0));
  switch (op) {
    case SIN:
      frame_->CallRuntime(Runtime::kMath_sin, 1);
      break;
    case COS:
      frame_->CallRuntime(Runtime::kMath_cos, 1);
      break;
  }
  __ nop(); // NOP_ADDED
  frame_->EmitPush(v0);
}


void CodeGenerator::GenerateObjectEquals(ZoneList<Expression*>* args) {
#ifdef DEBUG
//  printf("CodeGenerator::GenerateObjectEquals\n");
#endif
  VirtualFrame::SpilledScope spilled_scope;
  ASSERT(args->length() == 2);

  // Load the two objects into registers and perform the comparison.
  LoadAndSpill(args->at(0));
  LoadAndSpill(args->at(1));
//  frame_->EmitPop(r0);
//  frame_->EmitPop(r1);
  frame_->EmitPop(a0);
  frame_->EmitPop(a1);
//  __ cmp(r0, Operand(r1));
  __ mov(s5, a0);
  __ mov(s6, a1);
  cc_reg_ = eq;
}


void CodeGenerator::VisitCallRuntime(CallRuntime* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitCallRuntime\n");    // Debug printf
#endif

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  if (CheckForInlineRuntimeCall(node)) {
    ASSERT((has_cc() && frame_->height() == original_height) ||
           (!has_cc() && frame_->height() == original_height + 1));
    return;
  }

  ZoneList<Expression*>* args = node->arguments();
  Comment cmnt(masm_, "[ CallRuntime");
  Runtime::Function* function = node->function();

  int arg_count = args->length();

  if (function == NULL) {
    // Prepare stack for calling JS runtime function.
    __ SetupAlignedCall(t0, arg_count);
//    __ mov(r0, Operand(node->name()));
    __ li(t0, Operand(node->name()));
    frame_->EmitPush(t0);
    // Push the builtins object found in the current global object.
//    __ ldr(r1, GlobalObject());
//    __ ldr(r0, FieldMemOperand(r1, GlobalObject::kBuiltinsOffset));
    __ lw(t1, GlobalObject());
    __ lw(t0, FieldMemOperand(t1, GlobalObject::kBuiltinsOffset));
    frame_->EmitPush(t0);
  }

  // Push the arguments ("left-to-right").
  for (int i = 0; i < arg_count; i++) {
    LoadAndSpill(args->at(i));
  }

  if (function == NULL) {
    // Call the JS runtime function.
    InLoopFlag in_loop = loop_nesting() > 0 ? IN_LOOP : NOT_IN_LOOP;
    Handle<Code> stub = ComputeCallInitialize(arg_count, in_loop);
    frame_->CallCodeObject(stub, RelocInfo::CODE_TARGET, arg_count + 1);
//    __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
    __ nop();

    __ ReturnFromAlignedCall(); 
    __ lw(cp, frame_->Context());
    frame_->DropFromVFrameOnly();
    frame_->EmitPush(v0);
  } else {
    // Call the C runtime function.
    frame_->CallRuntime(function, arg_count);
    __ nop();   // NOP_ADDED
    frame_->EmitPush(v0);
  }
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitUnaryOperation(UnaryOperation* node) {
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ UnaryOperation");

  Token::Value op = node->op();

  if (op == Token::NOT) {
    LoadConditionAndSpill(node->expression(),
                          false_target(),
                          true_target(),
                          true);
    // LoadCondition may (and usually does) leave a test and branch to
    // be emitted by the caller.  In that case, negate the condition.
    if (has_cc()) cc_reg_ = NegateCondition(cc_reg_);

  } else if (op == Token::DELETE) {
    Property* property = node->expression()->AsProperty();
    Variable* variable = node->expression()->AsVariableProxy()->AsVariable();
    if (property != NULL) {
      LoadAndSpill(property->obj());
      LoadAndSpill(property->key());
      Result arg_count(a0);
      __ li(a0, Operand(1));  // not counting receiver
      frame_->InvokeBuiltin(Builtins::DELETE, CALL_JS, &arg_count, 2);
      __ nop(); // NOP_ADDED

    } else if (variable != NULL) {
      Slot* slot = variable->slot();
      if (variable->is_global()) {
        LoadGlobal();
        __ li(a0, Operand(variable->name()));
        frame_->EmitPush(a0);
        Result arg_count(a0);
        __ li(a0, Operand(1));  // not counting receiver
        frame_->InvokeBuiltin(Builtins::DELETE, CALL_JS, &arg_count, 2);
        __ nop(); // NOP_ADDED

      } else if (slot != NULL && slot->type() == Slot::LOOKUP) {
        // lookup the context holding the named variable
        frame_->EmitPush(cp);
        __ li(a0, Operand(variable->name()));
        frame_->EmitPush(a0);
        frame_->CallRuntime(Runtime::kLookupContext, 2);
        // v0: context
        frame_->EmitPush(v0);
        __ li(a0, Operand(variable->name()));
        frame_->EmitPush(a0);
        Result arg_count(a0);
        __ li(a0, Operand(1));  // not counting receiver
        frame_->InvokeBuiltin(Builtins::DELETE, CALL_JS, &arg_count, 2);
        __ nop(); // NOP_ADDED

      } else {
        // Default: Result of deleting non-global, not dynamically
        // introduced variables is false.
        __ LoadRoot(v0, Heap::kFalseValueRootIndex);
      }

    } else {
      // Default: Result of deleting expressions is true.
      LoadAndSpill(node->expression());  // may have side-effects
      frame_->Drop();
      __ LoadRoot(v0, Heap::kTrueValueRootIndex);
    }
    frame_->EmitPush(v0);

  } else if (op == Token::TYPEOF) {
    // Special case for loading the typeof expression; see comment on
    // LoadTypeofExpression().
    LoadTypeofExpression(node->expression());
    frame_->CallRuntime(Runtime::kTypeof, 1);
    frame_->EmitPush(v0);  // r0 has result

  } else {
    LoadAndSpill(node->expression());
    frame_->EmitPop(a0);
    switch (op) {
      case Token::NOT:
      case Token::DELETE:
      case Token::TYPEOF:
        UNREACHABLE();  // handled above
        break;

      case Token::SUB: {
        bool overwrite =
            (node->expression()->AsBinaryOperation() != NULL &&
             node->expression()->AsBinaryOperation()->ResultOverwriteAllowed());
        UnarySubStub stub(overwrite);
        frame_->CallStub(&stub, 0);
        __ nop();   // NOP_ADDED
        break;
      }

      case Token::BIT_NOT: {
        // smi check
        JumpTarget smi_label;
        JumpTarget continue_label;
//        __ tst(r0, Operand(kSmiTagMask));
        __ andi(t0, a0, Operand(kSmiTagMask));
        smi_label.Branch(eq, no_hint, t0, Operand(zero_reg));
        __ nop();   // NOP_ADDED

        frame_->EmitPush(a0);
        Result arg_count(a0);
        __ li(a0, Operand(0));  // not counting receiver
        frame_->InvokeBuiltin(Builtins::BIT_NOT, CALL_JS, &arg_count, 1);
        __ nop();   // NOP_ADDED

        continue_label.Jump();
        __ nop();   // NOP_ADDED
        smi_label.Bind();
//        __ mvn(r0, Operand(r0));
//        __ bic(r0, r0, Operand(kSmiTagMask));  // bit-clear inverted smi-tag
        __ or_(a0, a0, Operand(kSmiTagMask));
        __ movn(a0, a0);
        continue_label.Bind();
        break;
      }

      case Token::VOID:
        // since the stack top is cached in r0, popping and then
        // pushing a value can be done by just writing to r0.
        __ LoadRoot(a0, Heap::kUndefinedValueRootIndex);
        break;

      case Token::ADD: {
        // Smi check.
        JumpTarget continue_label;
//        __ tst(r0, Operand(kSmiTagMask));
        __ andi(t0, a0, Operand(kSmiTagMask));
        continue_label.Branch(eq, no_hint, t0, Operand(zero_reg));
        __ nop();   // NOP_ADDED
        frame_->EmitPush(a0);
        Result arg_count(a0);
        __ li(a0, Operand(0));  // not counting receiver
        frame_->InvokeBuiltin(Builtins::TO_NUMBER, CALL_JS, &arg_count, 1);
        __ nop();   // NOP_ADDED
        continue_label.Bind();
        break;
      }
      default:
        UNREACHABLE();
    }
    frame_->EmitPush(v0);  // r0 has result
  }
  ASSERT(!has_valid_frame() ||
         (has_cc() && frame_->height() == original_height) ||
         (!has_cc() && frame_->height() == original_height + 1));
}


void CodeGenerator::VisitCountOperation(CountOperation* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitCountOperation\n");
#endif


  // TODO(MIPS.1): Implement overflow checks

#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ CountOperation");

  bool is_postfix = node->is_postfix();
  bool is_increment = node->op() == Token::INC;

  Variable* var = node->expression()->AsVariableProxy()->AsVariable();
  bool is_const = (var != NULL && var->mode() == Variable::CONST);

  // Postfix: Make room for the result.
  if (is_postfix) {
    __ li(v0, Operand(0));
    frame_->EmitPush(v0);
    __ push(v0);
  }

  { Reference target(this, node->expression());
    if (target.is_illegal()) {
      // Spoof the virtual frame to have the expected height (one higher
      // than on entry).
      if (!is_postfix) {
        __ li(v0, Operand(0));
        frame_->EmitPush(v0);
      }
      ASSERT(frame_->height() == original_height + 1);
      return;
    }
    target.GetValueAndSpill();
    frame_->EmitPop(a0);

    JumpTarget slow;
    JumpTarget exit;

    // Check for smi operand.
    __ andi(t0, a0, Operand(kSmiTagMask));
    slow.Branch(ne, no_hint, t0, Operand(zero_reg));
    __ nop();   // NOP_ADDED

    // Postfix: Store the old value as the result.
    if (is_postfix) {
      __ sw(a0, frame_->ElementAt(target.size()));
    }

    // Perform optimistic increment/decrement.
    if (is_increment) {
      __ add(v0, a0, Operand(Smi::FromInt(1)));
    } else {
      __ add(v0, a0, Operand((Smi::FromInt(-1))));
    }

    // If the increment/decrement didn't overflow, we're done.
    // TODO(MIPS.1): Since we don't check for overflow we should always jump.
    exit.Branch(eq, no_hint, zero_reg, Operand(zero_reg));
    __ nop();   // NOP_ADDED


//    // Revert optimistic increment/decrement.
//    if (is_increment) {
//      __ sub(r0, r0, Operand(r1));
//    } else {
//      __ add(r0, r0, Operand(r1));
//    }
//
//    // Slow case: Convert to number.
    slow.Bind();
    __ break_(0x09001); // We should not come here yet.
//    {
//      // Convert the operand to a number.
//      frame_->EmitPush(r0);
//      Result arg_count(r0);
//      __ mov(r0, Operand(0));
//      frame_->InvokeBuiltin(Builtins::TO_NUMBER, CALL_JS, &arg_count, 1);
//    }
//    if (is_postfix) {
//      // Postfix: store to result (on the stack).
//      __ str(r0, frame_->ElementAt(target.size()));
//    }
//
//    // Compute the new value.
//    __ mov(r1, Operand(Smi::FromInt(1)));
//    frame_->EmitPush(r0);
//    frame_->EmitPush(r1);
//    if (is_increment) {
//      frame_->CallRuntime(Runtime::kNumberAdd, 2);
//    } else {
//      frame_->CallRuntime(Runtime::kNumberSub, 2);
//    }

    // Store the new value in the target if not const.
    exit.Bind();
    frame_->EmitPush(v0);
    if (!is_const) target.SetValue(NOT_CONST_INIT);
  }

  // Postfix: Discard the new value and use the old.
  if (is_postfix) frame_->EmitPop(v0);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitBinaryOperation(BinaryOperation* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitBinaryOperation\n");
#endif

//  __ break_(0x00332);
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ BinaryOperation");
  Token::Value op = node->op();

  // According to ECMA-262 section 11.11, page 58, the binary logical
  // operators must yield the result of one of the two expressions
  // before any ToBoolean() conversions. This means that the value
  // produced by a && or || operator is not necessarily a boolean.

  // NOTE: If the left hand side produces a materialized value (not in
  // the CC register), we force the right hand side to do the
  // same. This is necessary because we may have to branch to the exit
  // after evaluating the left hand side (due to the shortcut
  // semantics), but the compiler must (statically) know if the result
  // of compiling the binary operation is materialized or not.

  if (op == Token::AND) {
    JumpTarget is_true;
    LoadConditionAndSpill(node->left(),
                          &is_true,
                          false_target(),
                          false);
    if (has_valid_frame() && !has_cc()) {
      // The left-hand side result is on top of the virtual frame.
      JumpTarget pop_and_continue;
      JumpTarget exit;

//      __ ldr(r0, frame_->Top());  // Duplicate the stack top.
//      frame_->EmitPush(r0);
      __ lw(t0, frame_->Top());  // Duplicate the stack top.
      frame_->EmitPush(t0);
      // Avoid popping the result if it converts to 'false' using the
      // standard ToBoolean() conversion as described in ECMA-262,
      // section 9.2, page 30.
      ToBoolean(&pop_and_continue, &exit);
      Branch(false, &exit);

      // Pop the result of evaluating the first part.
      pop_and_continue.Bind();
//      frame_->EmitPop(r0);
      frame_->EmitPop(t0);
//
      // Evaluate right side expression.
      is_true.Bind();
      LoadAndSpill(node->right());

      // Exit (always with a materialized value).
      exit.Bind();
    } else if (has_cc() || is_true.is_linked()) {
      // The left-hand side is either (a) partially compiled to
      // control flow with a final branch left to emit or (b) fully
      // compiled to control flow and possibly true.
      if (has_cc()) {
        Branch(false, false_target());
      }
      is_true.Bind();
      LoadConditionAndSpill(node->right(),
                            true_target(),
                            false_target(),
                            false);
    } else {
      // Nothing to do.
      ASSERT(!has_valid_frame() && !has_cc() && !is_true.is_linked());
    }

  } else if (op == Token::OR) {
    JumpTarget is_false;
    LoadConditionAndSpill(node->left(),
                          true_target(),
                          &is_false,
                          false);
    if (has_valid_frame() && !has_cc()) {
      // The left-hand side result is on top of the virtual frame.
      JumpTarget pop_and_continue;
      JumpTarget exit;

//      __ ldr(r0, frame_->Top());
//      frame_->EmitPush(r0);
      __ lw(a0, frame_->Top());
      frame_->EmitPush(a0);
      // Avoid popping the result if it converts to 'true' using the
      // standard ToBoolean() conversion as described in ECMA-262,
      // section 9.2, page 30.
      ToBoolean(&exit, &pop_and_continue);
      Branch(true, &exit);
      __ nop(); // NOP_ADDED

      // Pop the result of evaluating the first part.
      pop_and_continue.Bind();
      frame_->EmitPop(v0);

      // Evaluate right side expression.
      is_false.Bind();
      LoadAndSpill(node->right());

      // Exit (always with a materialized value).
      exit.Bind();
    } else if (has_cc() || is_false.is_linked()) {
      // The left-hand side is either (a) partially compiled to
      // control flow with a final branch left to emit or (b) fully
      // compiled to control flow and possibly false.
      if (has_cc()) {
        Branch(true, true_target());
        __ nop(); // NOP_ADDED
      }
      is_false.Bind();
      LoadConditionAndSpill(node->right(),
                            true_target(),
                            false_target(),
                            false);
    } else {
      // Nothing to do.
      ASSERT(!has_valid_frame() && !has_cc() && !is_false.is_linked());
    }

  } else {
    // Optimize for the case where (at least) one of the expressions
    // is a literal small integer.
    Literal* lliteral = node->left()->AsLiteral();
    Literal* rliteral = node->right()->AsLiteral();
    // NOTE: The code below assumes that the slow cases (calls to runtime)
    // never return a constant/immutable object.
    bool overwrite_left =
        (node->left()->AsBinaryOperation() != NULL &&
         node->left()->AsBinaryOperation()->ResultOverwriteAllowed());
    bool overwrite_right =
        (node->right()->AsBinaryOperation() != NULL &&
         node->right()->AsBinaryOperation()->ResultOverwriteAllowed());

    if (rliteral != NULL && rliteral->handle()->IsSmi()) {
      LoadAndSpill(node->left());
      SmiOperation(node->op(),
                   rliteral->handle(),
                   false,
                   overwrite_right ? OVERWRITE_RIGHT : NO_OVERWRITE);

    } else if (lliteral != NULL && lliteral->handle()->IsSmi()) {
      LoadAndSpill(node->right());
      SmiOperation(node->op(),
                   lliteral->handle(),
                   true,
                   overwrite_left ? OVERWRITE_LEFT : NO_OVERWRITE);

    } else {
      OverwriteMode overwrite_mode = NO_OVERWRITE;
      if (overwrite_left) {
        overwrite_mode = OVERWRITE_LEFT;
      } else if (overwrite_right) {
        overwrite_mode = OVERWRITE_RIGHT;
      }
      LoadAndSpill(node->left());
      LoadAndSpill(node->right());
      GenericBinaryOperation(node->op(), overwrite_mode);
    }
    __ nop();
    frame_->EmitPush(v0);
  }
  ASSERT(!has_valid_frame() ||
         (has_cc() && frame_->height() == original_height) ||
         (!has_cc() && frame_->height() == original_height + 1));
}


void CodeGenerator::VisitThisFunction(ThisFunction* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitThisFunction\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
//  __ ldr(r0, frame_->Function());
//  frame_->EmitPush(r0);
  __ lw(t0, frame_->Function());
  frame_->EmitPush(t0);
  ASSERT(frame_->height() == original_height + 1);
}


void CodeGenerator::VisitCompareOperation(CompareOperation* node) {
#ifdef DEBUG
//  printf("CodeGenerator::VisitCompareOperation\n");
#endif
#ifdef DEBUG
  int original_height = frame_->height();
#endif
  VirtualFrame::SpilledScope spilled_scope;
  Comment cmnt(masm_, "[ CompareOperation");

  // Get the expressions from the node.
  Expression* left = node->left();
  Expression* right = node->right();
  Token::Value op = node->op();

  // To make null checks efficient, we check if either left or right is the
  // literal 'null'. If so, we optimize the code by inlining a null check
  // instead of calling the (very) general runtime routine for checking
  // equality.
  if (op == Token::EQ || op == Token::EQ_STRICT) {
    bool left_is_null =
        left->AsLiteral() != NULL && left->AsLiteral()->IsNull();
    bool right_is_null =
        right->AsLiteral() != NULL && right->AsLiteral()->IsNull();
    // The 'null' value can only be equal to 'null' or 'undefined'.
    if (left_is_null || right_is_null) {
      LoadAndSpill(left_is_null ? right : left);
//      frame_->EmitPop(r0);
//      __ LoadRoot(ip, Heap::kNullValueRootIndex);
//      __ cmp(r0, ip);
      frame_->EmitPop(t0);
      __ LoadRoot(t1, Heap::kNullValueRootIndex);
//
//      // The 'null' value is only equal to 'undefined' if using non-strict
//      // comparisons.
      if (op != Token::EQ_STRICT) {
        true_target()->Branch(eq, no_hint, t0, Operand(t1));
        __ nop();  // NOP_ADDED

//        __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//        __ cmp(r0, Operand(ip));
//        true_target()->Branch(eq);
        __ LoadRoot(t1, Heap::kUndefinedValueRootIndex);
        true_target()->Branch(eq, no_hint, t0, Operand(t1));
        __ nop();  // NOP_ADDED

//        __ tst(r0, Operand(kSmiTagMask));
//        false_target()->Branch(eq);
        __ andi(t2, t0, Operand(kSmiTagMask));
        false_target()->Branch(eq, no_hint, t2, Operand(zero_reg));
        __ nop();  // NOP_ADDED

        // It can be an undetectable object.
//        __ ldr(r0, FieldMemOperand(r0, HeapObject::kMapOffset));
//        __ ldrb(r0, FieldMemOperand(r0, Map::kBitFieldOffset));
//        __ and_(r0, r0, Operand(1 << Map::kIsUndetectable));
//        __ cmp(r0, Operand(1 << Map::kIsUndetectable));
        __ lw(t0, FieldMemOperand(t0, HeapObject::kMapOffset));
        __ lbu(t0, FieldMemOperand(t0, Map::kBitFieldOffset));
        __ and_(t0, t0, Operand(1 << Map::kIsUndetectable));
//        __ cmp(r0, Operand(1 << Map::kIsUndetectable));
        __ mov(s5, t0);
        __ li(s6, Operand(1 << Map::kIsUndetectable));
      }

      cc_reg_ = eq;
      ASSERT(has_cc() && frame_->height() == original_height);
      return;
    }
  }

  // To make typeof testing for natives implemented in JavaScript really
  // efficient, we generate special code for expressions of the form:
  // 'typeof <expression> == <string>'.
  UnaryOperation* operation = left->AsUnaryOperation();
  if ((op == Token::EQ || op == Token::EQ_STRICT) &&
      (operation != NULL && operation->op() == Token::TYPEOF) &&
      (right->AsLiteral() != NULL &&
       right->AsLiteral()->handle()->IsString())) {
    Handle<String> check(String::cast(*right->AsLiteral()->handle()));

    // Load the operand, move it to register r1->t1.
    LoadTypeofExpression(operation->expression());
    frame_->EmitPop(t1);


    if (check->Equals(Heap::number_symbol())) {
//      __ tst(r1, Operand(kSmiTagMask));
//      true_target()->Branch(eq);
//      __ ldr(r1, FieldMemOperand(r1, HeapObject::kMapOffset));
//      __ LoadRoot(ip, Heap::kHeapNumberMapRootIndex);
//      __ cmp(r1, ip);
//      cc_reg_ = eq;

      __ andi(t2, t1, Operand(kSmiTagMask));
      true_target()->Branch(eq, no_hint, t2, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      __ lw(t1, FieldMemOperand(t1, HeapObject::kMapOffset));
      __ LoadRoot(ip, Heap::kHeapNumberMapRootIndex);
//      __ cmp(r1, ip);
      __ mov(s5, t1);
      __ mov(s6, ip);
      cc_reg_ = eq;

    } else if (check->Equals(Heap::string_symbol())) {
//      __ tst(r1, Operand(kSmiTagMask));
//      false_target()->Branch(eq);
      __ andi(t2, t1, Operand(kSmiTagMask));
      false_target()->Branch(eq, no_hint, t2, Operand(zero_reg));
      __ nop(); // NOP_ADDED

//      __ ldr(r1, FieldMemOperand(r1, HeapObject::kMapOffset));
      __ lw(t1, FieldMemOperand(t1, HeapObject::kMapOffset));

      // It can be an undetectable string object.
//      __ ldrb(r2, FieldMemOperand(r1, Map::kBitFieldOffset));
//      __ and_(r2, r2, Operand(1 << Map::kIsUndetectable));
//      __ cmp(r2, Operand(1 << Map::kIsUndetectable));
//      false_target()->Branch(eq);
      __ lbu(t2, FieldMemOperand(t1, Map::kBitFieldOffset));
      __ and_(t2, t2, Operand(1 << Map::kIsUndetectable));
//      __ cmp(r2, Operand(1 << Map::kIsUndetectable));
      false_target()->Branch(eq, no_hint, t2, Operand(1 << Map::kIsUndetectable));
      __ nop(); // NOP_ADDED

//      __ ldrb(r2, FieldMemOperand(r1, Map::kInstanceTypeOffset));
//      __ cmp(r2, Operand(FIRST_NONSTRING_TYPE));
//      cc_reg_ = lt;
      __ lbu(t2, FieldMemOperand(t1, Map::kInstanceTypeOffset));
      __ mov(s5, t2);
      __ li(s6, Operand(FIRST_NONSTRING_TYPE));
      cc_reg_ = less;

    } else if (check->Equals(Heap::boolean_symbol())) {
//      __ LoadRoot(ip, Heap::kTrueValueRootIndex);
//      __ cmp(r1, ip);
//      true_target()->Branch(eq);
//      __ LoadRoot(ip, Heap::kFalseValueRootIndex);
//      __ cmp(r1, ip);
//      cc_reg_ = eq;
      __ LoadRoot(ip, Heap::kTrueValueRootIndex);
//      __ cmp(r1, ip);
      true_target()->Branch(eq, no_hint, t1, Operand(ip));
      __ nop(); // NOP_ADDED
      __ LoadRoot(ip, Heap::kFalseValueRootIndex);
//      __ cmp(r1, ip);
      __ mov(s5, t1);
      __ mov(s6, ip);
      cc_reg_ = eq;

    } else if (check->Equals(Heap::undefined_symbol())) {
      __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
//      __ cmp(r1, ip);
      true_target()->Branch(eq, no_hint, t1, Operand(ip));
      __ nop(); // NOP_ADDED

//      __ tst(r1, Operand(kSmiTagMask));
      __ andi(t3, t1, Operand(kSmiTagMask));
      false_target()->Branch(eq, no_hint, t3, Operand(zero_reg));
      __ nop(); // NOP_ADDED

      // It can be an undetectable object.
//      __ ldr(r1, FieldMemOperand(r1, HeapObject::kMapOffset));
//      __ ldrb(r2, FieldMemOperand(r1, Map::kBitFieldOffset));
//      __ and_(r2, r2, Operand(1 << Map::kIsUndetectable));
//      __ cmp(r2, Operand(1 << Map::kIsUndetectable));
      __ lw(t1, FieldMemOperand(t1, HeapObject::kMapOffset));
      __ lbu(t2, FieldMemOperand(t1, Map::kBitFieldOffset));
      // Setup s5 and s6 to values to be compared.
      __ and_(s5, t2, Operand(1 << Map::kIsUndetectable));
      __ li(s6, Operand(1 << Map::kIsUndetectable));

      cc_reg_ = eq;

    } else if (check->Equals(Heap::function_symbol())) {
//      __ tst(r1, Operand(kSmiTagMask));
//      false_target()->Branch(eq);
//      __ CompareObjectType(r1, r1, r1, JS_FUNCTION_TYPE);
//      cc_reg_ = eq;
      __ andi(t2, t1, Operand(kSmiTagMask));
      false_target()->Branch(eq, no_hint, t2, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      __ GetObjectType(t1, t1, t1);
      __ mov(s5, t1);
      __ li(s6, Operand(JS_FUNCTION_TYPE));
      cc_reg_ = eq;

    } else if (check->Equals(Heap::object_symbol())) {
//      __ tst(r1, Operand(kSmiTagMask));
//      false_target()->Branch(eq);
      __ andi(t2, t1, Operand(kSmiTagMask));
      false_target()->Branch(eq, no_hint, t2, Operand(zero_reg));
      __ nop(); // NOP_ADDED

//      __ ldr(r2, FieldMemOperand(r1, HeapObject::kMapOffset));
//      __ LoadRoot(ip, Heap::kNullValueRootIndex);
//      __ cmp(r1, ip);
//      true_target()->Branch(eq);
      __ lw(t2, FieldMemOperand(t1, HeapObject::kMapOffset));
      __ LoadRoot(ip, Heap::kNullValueRootIndex);
//      __ cmp(r1, ip);
      true_target()->Branch(eq, no_hint, t1, Operand(ip));
      __ nop(); // NOP_ADDED

      // It can be an undetectable object.
//      __ ldrb(r1, FieldMemOperand(r2, Map::kBitFieldOffset));
//      __ and_(r1, r1, Operand(1 << Map::kIsUndetectable));
//      __ cmp(r1, Operand(1 << Map::kIsUndetectable));
//      false_target()->Branch(eq);
      __ lbu(t1, FieldMemOperand(t2, Map::kBitFieldOffset));
      __ and_(t1, t1, Operand(1 << Map::kIsUndetectable));
//      __ cmp(r1, Operand(1 << Map::kIsUndetectable));
      false_target()->Branch(eq, no_hint, t1, Operand(1 << Map::kIsUndetectable));
      __ nop(); // NOP_ADDED


//      __ ldrb(r2, FieldMemOperand(r2, Map::kInstanceTypeOffset));
//      __ cmp(r2, Operand(FIRST_JS_OBJECT_TYPE));
//      false_target()->Branch(lt);
//      __ cmp(r2, Operand(LAST_JS_OBJECT_TYPE));
//      cc_reg_ = le;
      __ lbu(t2, FieldMemOperand(t2, Map::kInstanceTypeOffset));
//      __ cmp(r2, Operand(FIRST_JS_OBJECT_TYPE));
      false_target()->Branch(less, no_hint, t2, Operand(FIRST_JS_OBJECT_TYPE));
      __ nop(); // NOP_ADDED
//      __ cmp(r2, Operand(LAST_JS_OBJECT_TYPE));
      __ mov(s5, t2);
      __ li(s6, Operand(LAST_JS_OBJECT_TYPE));
      cc_reg_ = less_equal;

    } else {
      // Uncommon case: typeof testing against a string literal that is
      // never returned from the typeof operator.
      false_target()->Jump();
      __ nop(); // NOP_ADDED
    }
    ASSERT(!has_valid_frame() ||
           (has_cc() && frame_->height() == original_height));
    return;
  }

  switch (op) {
    case Token::EQ:
      Comparison(eq, left, right, false);
      break;

    case Token::LT:
      Comparison(less, left, right);
      break;

    case Token::GT:
      Comparison(greater, left, right);
      break;

    case Token::LTE:
      Comparison(less_equal, left, right);
      break;

    case Token::GTE:
      Comparison(greater_equal, left, right);
      break;

    case Token::EQ_STRICT:
      Comparison(eq, left, right, true);
      break;

    case Token::IN: {
      LoadAndSpill(left);
      LoadAndSpill(right);
      Result arg_count(a0);
//      __ mov(r0, Operand(1));  // not counting receiver
      __ li(a0, Operand(1));  // not counting receiver
      frame_->InvokeBuiltin(Builtins::IN, CALL_JS, &arg_count, 2);
      __ nop(); // NOP_ADDED
      frame_->EmitPush(v0);
      break;
    }

    case Token::INSTANCEOF: {
      LoadAndSpill(left);
      LoadAndSpill(right);
      InstanceofStub stub;
      frame_->CallStub(&stub, 2);
      __ nop(); // NOP_ADDED
      // At this point if instanceof succeeded then r0 == 0.
//      __ tst(r0, Operand(r0));
      __ mov(s5, v0);
      __ mov(s6, zero_reg);
      cc_reg_ = eq;
      break;
    }

    default:
      UNREACHABLE();
  }
  ASSERT((has_cc() && frame_->height() == original_height) ||
         (!has_cc() && frame_->height() == original_height + 1));
}


#ifdef DEBUG
bool CodeGenerator::HasValidEntryRegisters() { return true; }
#endif


#undef __
#define __ ACCESS_MASM(masm)


Handle<String> Reference::GetName() {
  ASSERT(type_ == NAMED);
  Property* property = expression_->AsProperty();
  if (property == NULL) {
    // Global variable reference treated as a named property reference.
    VariableProxy* proxy = expression_->AsVariableProxy();
    ASSERT(proxy->AsVariable() != NULL);
    ASSERT(proxy->AsVariable()->is_global());
    return proxy->name();
  } else {
    Literal* raw_name = property->key()->AsLiteral();
    ASSERT(raw_name != NULL);
    return Handle<String>(String::cast(*raw_name->handle()));
  }
}


void Reference::GetValue() {
#ifdef DEBUG
//  printf("Reference::GetValue\n");
#endif
  ASSERT(cgen_->HasValidEntryRegisters());
  ASSERT(!is_illegal());
  ASSERT(!cgen_->has_cc());
  MacroAssembler* masm = cgen_->masm();
  Property* property = expression_->AsProperty();
  if (property != NULL) {
    cgen_->CodeForSourcePosition(property->position());
  }

  switch (type_) {
    case SLOT: {
      Comment cmnt(masm, "[ Load from Slot");
      Slot* slot = expression_->AsVariableProxy()->AsVariable()->slot();
      ASSERT(slot != NULL);
      cgen_->LoadFromSlot(slot, NOT_INSIDE_TYPEOF);
      break;
    }
//
    case NAMED: {
      // TODO(1241834): Make sure that this it is safe to ignore the
      // distinction between expressions in a typeof and not in a typeof. If
      // there is a chance that reference errors can be thrown below, we
      // must distinguish between the two kinds of loads (typeof expression
      // loads must not throw a reference error).
      VirtualFrame* frame = cgen_->frame();
      Comment cmnt(masm, "[ Load from named Property");
      Handle<String> name(GetName());
      Variable* var = expression_->AsVariableProxy()->AsVariable();
      Handle<Code> ic(Builtins::builtin(Builtins::LoadIC_Initialize));
      // Setup the name register.
      Result name_reg(a2);
      __ li(a2, Operand(name));
      ASSERT(var == NULL || var->is_global());
      RelocInfo::Mode rmode = (var == NULL)
                            ? RelocInfo::CODE_TARGET
                            : RelocInfo::CODE_TARGET_CONTEXT;
      frame->CallCodeObject(ic, rmode, &name_reg, 0);
      __ nop(); // NOP_ADDED
      frame->EmitPush(v0);
      break;
    }

    case KEYED: {
      // TODO(1241834): Make sure that this it is safe to ignore the
      // distinction between expressions in a typeof and not in a typeof.

      // TODO(181): Implement inlined version of array indexing once
      // loop nesting is properly tracked on ARM.
      VirtualFrame* frame = cgen_->frame();
      Comment cmnt(masm, "[ Load from keyed Property");
      ASSERT(property != NULL);
      Handle<Code> ic(Builtins::builtin(Builtins::KeyedLoadIC_Initialize));
      Variable* var = expression_->AsVariableProxy()->AsVariable();
      ASSERT(var == NULL || var->is_global());
      RelocInfo::Mode rmode = (var == NULL)
                            ? RelocInfo::CODE_TARGET
                            : RelocInfo::CODE_TARGET_CONTEXT;
      frame->CallCodeObject(ic, rmode, 0);
      __ nop(); // NOP_ADDED
      frame->EmitPush(v0);
      break;
    }

    default:
      UNREACHABLE();
  }
}


void Reference::SetValue(InitState init_state) {
#ifdef DEBUG
//  printf("Reference::SetValue\n");
#endif

  ASSERT(!is_illegal());
  ASSERT(!cgen_->has_cc());
  MacroAssembler* masm = cgen_->masm();
  VirtualFrame* frame = cgen_->frame();
  Property* property = expression_->AsProperty();
  if (property != NULL) {
    cgen_->CodeForSourcePosition(property->position());
  }

  switch (type_) {
    case SLOT: {
      Comment cmnt(masm, "[ Store to Slot");
      Slot* slot = expression_->AsVariableProxy()->AsVariable()->slot();
      ASSERT(slot != NULL);
      if (slot->type() == Slot::LOOKUP) {
        ASSERT(slot->var()->is_dynamic());

        // For now, just do a runtime call.
        frame->EmitPush(cp);
//        __ mov(r0, Operand(slot->var()->name()));
        __ li(a0, Operand(slot->var()->name()));
        frame->EmitPush(a0);

        if (init_state == CONST_INIT) {
          // Same as the case for a normal store, but ignores attribute
          // (e.g. READ_ONLY) of context slot so that we can initialize
          // const properties (introduced via eval("const foo = (some
          // expr);")). Also, uses the current function context instead of
          // the top context.
          //
          // Note that we must declare the foo upon entry of eval(), via a
          // context slot declaration, but we cannot initialize it at the
          // same time, because the const declaration may be at the end of
          // the eval code (sigh...) and the const variable may have been
          // used before (where its value is 'undefined'). Thus, we can only
          // do the initialization when we actually encounter the expression
          // and when the expression operands are defined and valid, and
          // thus we need the split into 2 operations: declaration of the
          // context slot followed by initialization.
          frame->CallRuntime(Runtime::kInitializeConstContextSlot, 3);
        } else {
          frame->CallRuntime(Runtime::kStoreContextSlot, 3);
        }
        __ nop();   // NOP_ADDED
        // Storing a variable must keep the (new) value on the expression
        // stack. This is necessary for compiling assignment expressions.
//        frame->EmitPush(r0);
        frame->EmitPush(v0);

      } else {
        ASSERT(!slot->var()->is_dynamic());

        JumpTarget exit;
        if (init_state == CONST_INIT) {
          ASSERT(slot->var()->mode() == Variable::CONST);
          // Only the first const initialization must be executed (the slot
          // still contains 'the hole' value). When the assignment is
          // executed, the code is identical to a normal store (see below).
          Comment cmnt(masm, "[ Init const");
//          __ ldr(r2, cgen_->SlotOperand(slot, r2));
//          __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
//          __ cmp(r2, ip);
//          exit.Branch(ne);
          __ lw(a2, cgen_->SlotOperand(slot, a2));
          exit.Branch(ne,no_hint, a2, Operand(Heap::kTheHoleValueRootIndex));
          __ nop(); // NOP_ADDED
        }

        // We must execute the store.  Storing a variable must keep the
        // (new) value on the stack. This is necessary for compiling
        // assignment expressions.
        //
        // Note: We will reach here even with slot->var()->mode() ==
        // Variable::CONST because of const declarations which will
        // initialize consts to 'the hole' value and by doing so, end up
        // calling this code.  r2 may be loaded with context; used below in
        // RecordWrite.
//        frame->EmitPop(r0);
//        __ str(r0, cgen_->SlotOperand(slot, r2));
//        frame->EmitPush(r0);
        frame->EmitPop(a0);
        __ sw(a0, cgen_->SlotOperand(slot, a2));
        frame->EmitPush(a0);
        if (slot->type() == Slot::CONTEXT) {
          // Skip write barrier if the written value is a smi.
//          __ tst(r0, Operand(kSmiTagMask));
//          exit.Branch(eq);
          __ andi(ip, a0, Operand(kSmiTagMask));
          exit.Branch(eq, no_hint, ip, Operand(zero_reg));
          // r2->a2 is loaded with context when calling SlotOperand above.
          int offset = FixedArray::kHeaderSize + slot->index() * kPointerSize;
//          __ mov(r3, Operand(offset));
//          __ RecordWrite(r2, r3, r1);
          __ li(a3, Operand(offset));
          __ RecordWrite(a2, a3, a1);
        }
        // If we definitely did not jump over the assignment, we do not need
        // to bind the exit label.  Doing so can defeat peephole
        // optimization.
        if (init_state == CONST_INIT || slot->type() == Slot::CONTEXT) {
          exit.Bind();
        }
      }
      break;
    }
//
    case NAMED: {
      Comment cmnt(masm, "[ Store to named Property");
      // Call the appropriate IC code.
      Handle<Code> ic(Builtins::builtin(Builtins::StoreIC_Initialize));
      Handle<String> name(GetName());

      Result value(a0);
      frame->EmitPop(a0);

      // Setup the name register.
      Result property_name(a2);
      __ li(a2, Operand(name));
      frame->CallCodeObject(ic,
                            RelocInfo::CODE_TARGET,
                            &value,
                            &property_name,
                            0);
      __ nop(); // NOP_ADDED
      frame->EmitPush(v0);
//      __ break_(0x08109);
      break;
    }

    case KEYED: {
      Comment cmnt(masm, "[ Store to keyed Property");
      Property* property = expression_->AsProperty();
      ASSERT(property != NULL);
      cgen_->CodeForSourcePosition(property->position());

      // Call IC code.
      Handle<Code> ic(Builtins::builtin(Builtins::KeyedStoreIC_Initialize));
      // TODO(1222589): Make the IC grab the values from the stack.
      Result value(a0); // We use a0 because it will be used as an argument.
      frame->EmitPop(a0);  // value
      frame->CallCodeObject(ic, RelocInfo::CODE_TARGET, &value, 0);
      __ nop(); // NOP_ADDED
      frame->EmitPush(v0);
      break;
    }

    default:
      UNREACHABLE();
  }
}


// Takes a Smi and converts to an IEEE 64 bit floating point value in two
// registers.  The format is 1 sign bit, 11 exponent bits (biased 1023) and
// 52 fraction bits (20 in the first word, 32 in the second).  Zeros is a
// scratch register.  Destroys the source register.  No GC occurs during this
// stub so you don't have to set up the frame.
// We do not need this as we have a FPU.
class ConvertToDoubleStub : public CodeStub {
 public:
  ConvertToDoubleStub(Register result_reg_1,
                      Register result_reg_2,
                      Register source_reg,
                      Register scratch_reg)
      : result1_(result_reg_1),
        result2_(result_reg_2),
        source_(source_reg),
        zeros_(scratch_reg) { }

 private:
  Register result1_;
  Register result2_;
  Register source_;
  Register zeros_;

  // Minor key encoding in 16 bits.
  class ModeBits: public BitField<OverwriteMode, 0, 2> {};
  class OpBits: public BitField<Token::Value, 2, 14> {};

  Major MajorKey() { return ConvertToDouble; }
  int MinorKey() {
    // Encode the parameters in a unique 16 bit value.
    return  result1_.code() +
           (result2_.code() << 4) +
           (source_.code() << 8) +
           (zeros_.code() << 12);
  }

  void Generate(MacroAssembler* masm);

  const char* GetName() { return "ConvertToDoubleStub"; }

#ifdef DEBUG
  void Print() { PrintF("ConvertToDoubleStub\n"); }
#endif
};


void ConvertToDoubleStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
  __ break_(0x00666);
}


// This stub can convert a signed int32 to a heap number (double).  It does
// not work for int32s that are in Smi range!  No GC occurs during this stub
// so you don't have to set up the frame.
class WriteInt32ToHeapNumberStub : public CodeStub {
 public:
  WriteInt32ToHeapNumberStub(Register the_int,
                             Register the_heap_number,
                             Register scratch)
      : the_int_(the_int),
        the_heap_number_(the_heap_number),
        scratch_(scratch) { }

 private:
  Register the_int_;
  Register the_heap_number_;
  Register scratch_;

  // Minor key encoding in 16 bits.
  class ModeBits: public BitField<OverwriteMode, 0, 2> {};
  class OpBits: public BitField<Token::Value, 2, 14> {};

  Major MajorKey() { return WriteInt32ToHeapNumber; }
  int MinorKey() {
    // Encode the parameters in a unique 16 bit value.
    return  the_int_.code() +
           (the_heap_number_.code() << 4) +
           (scratch_.code() << 8);
  }

  void Generate(MacroAssembler* masm);

  const char* GetName() { return "WriteInt32ToHeapNumberStub"; }

#ifdef DEBUG
  void Print() { PrintF("WriteInt32ToHeapNumberStub\n"); }
#endif
};


// See comment for class.
void WriteInt32ToHeapNumberStub::Generate(MacroAssembler *masm) {
  UNIMPLEMENTED();
  __ break_(0x00666);
}


// Handle the case where the lhs and rhs are the same object.
// Equality is almost reflexive (everything but NaN), so this is a test
// for "identity and not NaN".
static void EmitIdenticalObjectComparison(MacroAssembler* masm,
                                          Label* slow,
                                          Condition cc) {
  Label not_identical;
//  __ cmp(r0, Operand(r1));
//  __ b(ne, &not_identical);
  __ bcond(ne, &not_identical, a0, Operand(a1));
  __ nop(); // NOP_ADDED

//  Register exp_mask_reg = r5;
//  __ mov(exp_mask_reg, Operand(HeapNumber::kExponentMask));
  Register exp_mask_reg = t5;
  __ li(exp_mask_reg, Operand(HeapNumber::kExponentMask));

  // Test for NaN. Sadly, we can't just compare to Factory::nan_value(),
  // so we do the second best thing - test it ourselves.
  Label heap_number, return_equal;
  // They are both equal and they are not both Smis so both of them are not
  // Smis.  If it's not a heap number, then return equal.
  if (cc == less || cc == greater) {
//    __ CompareObjectType(r0, r4, r4, FIRST_JS_OBJECT_TYPE);
//    __ b(ge, slow);
    __ GetObjectType(a0, t4, t4);
    __ bcond(greater, slow, t4, Operand(FIRST_JS_OBJECT_TYPE));
    __ nop();   // NOP_ADDED
  } else {
//    __ CompareObjectType(r0, r4, r4, HEAP_NUMBER_TYPE);
//    __ b(eq, &heap_number);
    __ GetObjectType(a0, t4, t4);
    __ bcond(eq, &heap_number, t4, Operand(HEAP_NUMBER_TYPE));
    __ nop();   // NOP_ADDED
    // Comparing JS objects with <=, >= is complicated.
    if (cc != eq) {
//      __ cmp(r4, Operand(FIRST_JS_OBJECT_TYPE));
//      __ b(ge, slow);
    __ bcond(greater, slow, t4, Operand(FIRST_JS_OBJECT_TYPE));
    __ nop();   // NOP_ADDED
    }
  }
  __ bind(&return_equal);
  if (cc == less) {
    __ li(v0, Operand(GREATER));  // Things aren't less than themselves.
  } else if (cc == greater) {
    __ li(v0, Operand(LESS));     // Things aren't greater than themselves.
  } else {
    __ li(v0, Operand(0));        // Things are <=, >=, ==, === themselves.
  }
//  __ mov(pc, Operand(lr));  // Return.
  __ jr(Operand(ra));
  __ nop(); // NOP_ADDED

  // For less and greater we don't have to check for NaN since the result of
  // x < x is false regardless.  For the others here is some code to check
  // for NaN.
  if (cc != less && cc != greater) {
    __ bind(&heap_number);
    // It is a heap number, so return non-equal if it's NaN and equal if it's
    // not NaN.
    __ ldc1(f4, FieldMemOperand(a0, HeapNumber::kValueOffset));
    __ ldc1(f6, FieldMemOperand(a0, HeapNumber::kValueOffset));
    __ c(UN, D, f4, f6);
    __ bc1f(&return_equal);
    __ nop();   // NOP_ADDED
    if (cc != eq) {
      __ jcond(Operand(ra), eq, a0, Operand(zero_reg));
      if (cc == less_equal) {
        __ li(v0, Operand(GREATER));  // NaN <= NaN should fail.
      } else {
        __ li(v0, Operand(LESS));     // NaN >= NaN should fail.
      }
    }
    __ jr(Operand(ra));
    __ nop();   // NOP_ADDED
  }
  // No fall through here.

  __ bind(&not_identical);
}


// See comment at call site.
static void EmitSmiNonsmiComparison(MacroAssembler* masm,
                                    Label* both_loaded_as_doubles,
                                    Label* slow,
                                    bool strict) {
  Label lhs_is_smi;
  __ andi(t0, a0, Operand(kSmiTagMask));
  __ bcond(eq, &lhs_is_smi, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Rhs is a Smi.
  // Check whether the non-smi is a heap number.
//  __ CompareObjectType(r0, r4, r4, HEAP_NUMBER_TYPE);
  __ GetObjectType(a0, t4, t4);
  if (strict) {
    // If lhs was not a number and rhs was a Smi then strict equality cannot
    // succeed.  Return non-equal (r0 is already not zero)
//    __ mov(pc, Operand(lr), LeaveCC, ne);  // Return.
    __ mov(v0,a0);
    __ jcond(Operand(ra), ne, t4, Operand(HEAP_NUMBER_TYPE));
    __ nop();   // NOP_ADDED
  } else {
    // Smi compared non-strictly with a non-Smi non-heap-number.  Call
    // the runtime.
//    __ b(ne, slow);
    __ bcond(ne, slow, t4, Operand(HEAP_NUMBER_TYPE));
    __ nop();   // NOP_ADDED
  }

  // Rhs is a smi, lhs is a number.
  // Convert a1 to double.
  __ mtc1(f12, a1);
  __ cvt_d_s(f12,f12);
  __ ldc1(f14, FieldMemOperand(a0, HeapNumber::kValueOffset));

  // We now have both loaded as doubles.
//  __ pop(lr);
//  __ jmp(rhs_not_nan);
  __ b(both_loaded_as_doubles);
  __ nop(); // NOP_ADDED

  __ bind(&lhs_is_smi);
  // Lhs is a Smi.  Check whether the non-smi is a heap number.
//  __ CompareObjectType(r1, r4, r4, HEAP_NUMBER_TYPE);
  __ GetObjectType(a1, t4, t4);
  if (strict) {
    // If lhs was not a number and rhs was a Smi then strict equality cannot
    // succeed.  Return non-equal.
//    __ mov(r0, Operand(1), LeaveCC, ne);  // Non-zero indicates not equal.
//    __ mov(pc, Operand(lr), LeaveCC, ne);  // Return.
    __ li(v0, Operand(1));
    __ jcond(Operand(ra), ne, t4, Operand(HEAP_NUMBER_TYPE));
    __ nop();   // NOP_ADDED
  } else {
    // Smi compared non-strictly with a non-Smi non-heap-number.  Call
    // the runtime.
//    __ b(ne, slow);
    __ bcond(ne, slow, t4, Operand(HEAP_NUMBER_TYPE));
    __ nop();   // NOP_ADDED
  }

  // Lhs is a smi, rhs is a number.
  // r0 is Smi and r1 is heap number.
//  __ push(lr);
//  __ ldr(r2, FieldMemOperand(r1, HeapNumber::kValueOffset));
//  __ ldr(r3, FieldMemOperand(r1, HeapNumber::kValueOffset + kPointerSize));
//  __ mov(r7, Operand(r0));
//  ConvertToDoubleStub stub2(r1, r0, r7, r6);
//  __ Call(stub2.GetCode(), RelocInfo::CODE_TARGET);
//  __ pop(lr);
  // Convert a1 to double.
  __ mtc1(f14, a0);
  __ cvt_d_s(f14,f14);
  __ ldc1(f12, FieldMemOperand(a1, HeapNumber::kValueOffset));
//  // Fall through to both_loaded_as_doubles.
}


void EmitNanCheck(MacroAssembler* masm, Condition cc) {
  // We use the coprocessor c.cond instructions.
  Label one_is_nan, neither_is_nan;

  // Test the Unordered condition on both doubles. This is false if any of them
  // is Nan.
//  __ break_(0x00071);
  __ c(UN, D, f12, f14);
  __ bc1f(&neither_is_nan);
  __ nop();
  __ bc1t(&one_is_nan);
  __ nop();

  // At least one is nan
  __ bind(&one_is_nan);
  // NaN comparisons always fail.
  // Load whatever we need in r0 to make the comparison fail.
  if (cc == less || cc == less_equal) {
//    __ mov(r0, Operand(GREATER));
    __ li(v0, Operand(GREATER));
  } else {
//    __ mov(r0, Operand(LESS));
    __ li(v0, Operand(LESS));
  }
//  __ mov(pc, Operand(lr));  // Return.
  __ jr(Operand(ra));

  __ bind(&neither_is_nan);
}


// See comment at call site.
static void EmitTwoNonNanDoubleComparison(MacroAssembler* masm, Condition cc) {
  // f12 and f14 have the two doubles.  Neither is a NaN.
  // Call a native function to do a comparison between two non-NaNs.
  // Call C routine that may not cause GC or other trouble.
  // We use a call_was and return manually because we need arguments slots to
  // be freed.
//    __ mov(r5, Operand(ExternalReference::compare_doubles()));
//    __ Jump(r5);  // Tail call.
//  __ break_(0x00091);
  __ addiu(sp, sp, -8);
  __ sw(s3, MemOperand(sp, 4));
  __ sw(ra, MemOperand(sp, 0));
  
  __ li(t9, Operand(ExternalReference::compare_doubles()));
  __ mov(s3, sp);                   // Save sp
  __ li(t0, Operand(~7));           // Load sp mask
  __ and_(sp, sp, Operand(t0));     // Align sp.
  __ Call(t9);                      // Call the code
  __ addiu(sp, sp, Operand(-StandardFrameConstants::kRArgsSlotsSize));
  __ mov(sp, s3);                   // Restore sp.

  __ lw(s3, MemOperand(sp, 4));
  __ lw(ra, MemOperand(sp, 0));
  __ addiu(sp, sp, 8);
  
  __ jr(ra);
  __ nop();
}


// See comment at call site.
static void EmitStrictTwoHeapObjectCompare(MacroAssembler* masm) {
    // If either operand is a JSObject or an oddball value, then they are
    // not equal since their pointers are different.
    // There is no test for undetectability in strict equality.
    ASSERT(LAST_TYPE == JS_FUNCTION_TYPE);
    Label first_non_object;
    // Get the type of the first operand into r2 and compare it with
    // FIRST_JS_OBJECT_TYPE.
//    __ CompareObjectType(r0, r2, r2, FIRST_JS_OBJECT_TYPE);
//    __ b(lt, &first_non_object);
    __ GetObjectType(a0, t2, t2);
    __ bcond(less, &first_non_object, t2, Operand(FIRST_JS_OBJECT_TYPE));
    __ nop();   // NOP_ADDED

    // Return non-zero (r0 is not zero)
    Label return_not_equal;
    __ bind(&return_not_equal);
//    __ mov(pc, Operand(lr));  // Return.
    __ li(v0, Operand(1));
    __ jr(ra);

    __ bind(&first_non_object);
    // Check for oddballs: true, false, null, undefined.
//    __ cmp(r2, Operand(ODDBALL_TYPE));
//    __ b(eq, &return_not_equal);
    __ bcond(eq, &return_not_equal, t2, Operand(ODDBALL_TYPE));
    __ nop();

//    __ CompareObjectType(r1, r3, r3, FIRST_JS_OBJECT_TYPE);
//    __ b(ge, &return_not_equal);
    __ GetObjectType(a1, t3, t3);
    __ bcond(greater, &return_not_equal, t3, Operand(FIRST_JS_OBJECT_TYPE));

    // Check for oddballs: true, false, null, undefined.
//    __ cmp(r3, Operand(ODDBALL_TYPE));
//    __ b(eq, &return_not_equal);
    __ bcond(eq, &return_not_equal, t3, Operand(ODDBALL_TYPE));
    __ nop();
}


// See comment at call site.
static void EmitCheckForTwoHeapNumbers(MacroAssembler* masm,
                                       Label* both_loaded_as_doubles,
                                       Label* not_heap_numbers,
                                       Label* slow) {
//  __ CompareObjectType(r0, r2, r2, HEAP_NUMBER_TYPE);
//  __ b(ne, not_heap_numbers);
//  __ CompareObjectType(r1, r3, r3, HEAP_NUMBER_TYPE);
//  __ b(ne, slow);  // First was a heap number, second wasn't.  Go slow case.
  __ GetObjectType(a0, t2, t2);
  __ bcond(ne, not_heap_numbers, t2, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  __ GetObjectType(a1, t3, t3);
  // First was a heap number, second wasn't.  Go slow case.
  __ bcond(ne, not_heap_numbers, t3, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED

  // Both are heap numbers.  Load them up then jump to the code we have
  // for that.
  __ ldc1(f12, FieldMemOperand(a0, HeapNumber::kValueOffset));
  __ ldc1(f14, FieldMemOperand(a1, HeapNumber::kValueOffset));
  __ b(both_loaded_as_doubles);
  __ nop(); // NOP_ADDED
}


// Fast negative check for symbol-to-symbol equality.
static void EmitCheckForSymbols(MacroAssembler* masm, Label* slow) {
  // rt is object type of a0.
//  __ tst(r2, Operand(kIsNotStringMask));
//  __ b(ne, slow);
  __ andi(t4, t2, Operand(kIsNotStringMask));
  __ bcond(ne, slow, t4, Operand(zero_reg));
  __ nop(); // NOP_ADDED
//  __ tst(r2, Operand(kIsSymbolMask));
//  __ b(eq, slow);
  __ andi(t4, t2, Operand(kIsSymbolMask));
  __ bcond(ne, slow, t4, Operand(zero_reg));
  __ nop(); // NOP_ADDED
//  __ CompareObjectType(r1, r3, r3, FIRST_NONSTRING_TYPE);
//  __ b(ge, slow);
  __ GetObjectType(a1, t3, t3);
  __ bcond(greater, slow, t3, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED
//  __ tst(r3, Operand(kIsSymbolMask));
//  __ b(eq, slow);
  __ andi(t5, t3, Operand(kIsSymbolMask));
  __ bcond(ne, slow, t5, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Both are symbols.  We already checked they weren't the same pointer
  // so they are not equal.
//  __ mov(r0, Operand(1));   // Non-zero indicates not equal.
//  __ mov(pc, Operand(lr));  // Return.
  __ li(v0, Operand(1));   // Non-zero indicates not equal.
  __ jr(ra);
  __ nop(); // NOP_ADDED
}


// On entry a0 and a1 are the things to be compared.  On exit v0 is 0,
// positive or negative to indicate the result of the comparison.
void CompareStub::Generate(MacroAssembler* masm) {
//  __ break_(0x00174);
  Label slow;  // Call builtin.
  Label not_smis, both_loaded_as_doubles;

  // NOTICE! This code is only reached after a smi-fast-case check, so
  // it is certain that at least one operand isn't a smi.

  // Handle the case where the objects are identical.  Either returns the answer
  // or goes to slow.  Only falls through if the objects were not identical.
  EmitIdenticalObjectComparison(masm, &slow, cc_);

  // If either is a Smi (we know that not both are), then they can only
  // be strictly equal if the other is a HeapNumber.
  ASSERT_EQ(0, kSmiTag);
  ASSERT_EQ(0, Smi::FromInt(0));
  __ and_(t2, a0, Operand(a1));
  __ andi(t2, t2, Operand(kSmiTagMask));
  __ bcond(ne, &not_smis, t2, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  // One operand is a smi.  EmitSmiNonsmiComparison generates code that can:
  // 1) Return the answer.
  // 2) Go to slow.
  // 3) Fall through to both_loaded_as_doubles.
  // 4) Jump to rhs_not_nan.
  // In cases 3 and 4 we have found out we were dealing with a number-number
  // comparison and the numbers have been loaded into f12 and f14 as doubles.
  EmitSmiNonsmiComparison(masm, &both_loaded_as_doubles, &slow, strict_);

  __ bind(&both_loaded_as_doubles);
  // f12, f14 are the double representations of the left hand side
  // and the right hand side.

  // Checks for NaN in the doubles we have loaded.  Can return the answer or
  // fall through if neither is a NaN.  Also binds rhs_not_nan.
  EmitNanCheck(masm, cc_);

  // Compares two doubles in r0, r1, r2, r3 that are not NaNs.  Returns the
  // answer.  Never falls through.
  EmitTwoNonNanDoubleComparison(masm, cc_);

  __ bind(&not_smis);
//  __ break_(0x00099);
  // At this point we know we are dealing with two different objects,
  // and neither of them is a Smi.  The objects are in r0 and r1.
  if (strict_) {
    // This returns non-equal for some object types, or falls through if it
    // was not lucky.
    EmitStrictTwoHeapObjectCompare(masm);
  }

  Label check_for_symbols;
  // Check for heap-number-heap-number comparison.  Can jump to slow case,
  // or load both doubles into r0, r1, r2, r3 and jump to the code that handles
  // that case.  If the inputs are not doubles then jumps to check_for_symbols.
  // In this case r2 will contain the type of r0.
  EmitCheckForTwoHeapNumbers(masm,
                             &both_loaded_as_doubles,
                             &check_for_symbols,
                             &slow);

  __ bind(&check_for_symbols);
  if (cc_ == eq) {
    // Either jumps to slow or returns the answer.  Assumes that r2 is the type
    // of r0 on entry.
    EmitCheckForSymbols(masm, &slow);
  }

  __ bind(&slow);
#ifdef NO_NATIVES
  UNIMPLEMENTED();
  __ break_(0x5608);   // Check below. Return has to be implemented.
#else
//  __ push(lr);
//  __ push(r1);
//  __ push(r0);
  __ multi_push_reversed(a0.bit() | a1.bit() | ra.bit());
  // Figure out which native to call and setup the arguments.
  Builtins::JavaScript native;
  int arg_count = 1;  // Not counting receiver.
  if (cc_ == eq) {
    native = strict_ ? Builtins::STRICT_EQUALS : Builtins::EQUALS;
  } else {
    native = Builtins::COMPARE;
    int ncr;  // NaN compare result
    if (cc_ == less || cc_ == less_equal) {
      ncr = GREATER;
    } else {
      ASSERT(cc_ == greater || cc_ == greater_equal);  // remaining cases
      ncr = LESS;
    }
    arg_count++;
//    __ mov(r0, Operand(Smi::FromInt(ncr)));
//    __ push(r0);
    __ li(a0, Operand(Smi::FromInt(ncr)));
    __ push(a0);
  }

  // Call the native; it returns -1 (less), 0 (equal), or 1 (greater)
  // tagged as a small integer.
//  __ mov(r0, Operand(arg_count));
//  __ InvokeBuiltin(native, CALL_JS);
//  __ cmp(r0, Operand(0));
//  __ pop(pc);
  __ li(a0, Operand(arg_count));
  __ InvokeBuiltin(native, CALL_JS);
  __ nop(); // NOP_ADDED
  // Setup comparison
  __ mov(s5, v0);
  __ mov(s6, zero_reg);
  // Return
  __ pop(ra);
  __ jr(ra);
  __ nop();
#endif
}


// Allocates a heap number or jumps to the label if the young space is full and
// a scavenge is needed.
static void AllocateHeapNumber(
    MacroAssembler* masm,
    Label* need_gc,       // Jump here if young space is full.
    Register result,  // The tagged address of the new heap number.
    Register scratch1,  // A scratch register.
    Register scratch2) {  // Another scratch register.

//  // Allocate an object in the heap for the heap number and tag it as a heap
//  // object.
//  __ AllocateInNewSpace(HeapNumber::kSize / kPointerSize,
//                        result,
//                        scratch1,
//                        scratch2,
//                        need_gc,
//                        TAG_OBJECT);
//

  // We ask for four more bytes to align it as we need and align the result.
  // (HeapNumber::kSize is modified to be 4-byte bigger)
  __ AllocateInNewSpace((HeapNumber::kSize) / kPointerSize,
                        result,
                        scratch1,
                        scratch2,
                        need_gc,
                        TAG_OBJECT);

  // Align to 8 bytes
  __ addiu(result, result, 7-1);  // -1 because result is tagged
  __ andi(result, result, Operand(~7));
  __ ori(result, result, Operand(1));  // Tag it back.

#ifdef DEBUG
////// TODO(MIPS.6)
//  // Check that the result is 8-byte aligned.
//  __ andi(scratch2, result, Operand(7));
//  __ xori(scratch2, scratch2, Operand(1));  // Fail if the tag is missing.
//  __ Check(eq,
//          "Error in HeapNumber allocation (not 8-byte aligned or tag missing)",
//          scratch2, Operand(zero_reg));
#endif

  // Get heap number map and store it in the allocated object.
//  __ LoadRoot(scratch1, Heap::kHeapNumberMapRootIndex);
//  __ str(scratch1, FieldMemOperand(result, HeapObject::kMapOffset));
  __ LoadRoot(scratch1, Heap::kHeapNumberMapRootIndex);
  __ sw(scratch1, FieldMemOperand(result, HeapObject::kMapOffset));
}


// We fall into this code if the operands were Smis, but the result was
// not (eg. overflow).  We branch into this code (to the not_smi label) if
// the operands were not both Smi.  The operands are in r0 and r1.  In order
// to call the C-implemented binary fp operation routines we need to end up
// with the double precision floating point operands in r0 and r1 (for the
// value in r1) and r2 and r3 (for the value in r0).
// CURRENT: static void HandleBinaryOpSlowCases
static void HandleBinaryOpSlowCases(MacroAssembler* masm,
                                    Label* not_smi,
                                    const Builtins::JavaScript& builtin,
                                    Token::Value operation,
                                    OverwriteMode mode) {
  // TODO(MIPS.1): Implement overflow cases.
  Label slow, do_the_call;
  Label a0_is_smi, a1_is_smi, finished_loading_a0, finished_loading_a1;
  // Smi-smi case (overflow).
  // Since both are Smis there is no heap number to overwrite, so allocate.
  // The new heap number is in r5.  r6 and r7 are scratch.
  // We should not meet this case yet, as we do not check for smi-smi overflows
  // in GenericBinaryOpStub::Generate
//  AllocateHeapNumber(masm, &slow, r5, r6, r7);
//  // Write Smi from r0 to r3 and r2 in double format.  r6 is scratch.
//  __ mov(r7, Operand(r0));
//  ConvertToDoubleStub stub1(r3, r2, r7, r6);
//  __ push(lr);
//  __ Call(stub1.GetCode(), RelocInfo::CODE_TARGET);
//  // Write Smi from r1 to r1 and r0 in double format.  r6 is scratch.
//  __ mov(r7, Operand(r1));
//  ConvertToDoubleStub stub2(r1, r0, r7, r6);
//  __ Call(stub2.GetCode(), RelocInfo::CODE_TARGET);
//  __ pop(lr);
//  __ jmp(&do_the_call);  // Tail call.  No return.

//  // We jump to here if something goes wrong (one param is not a number of any
//  // sort or new-space allocation fails).
  __ bind(&slow);
#ifdef NO_NATIVES
  __ break_(0x00707);   // We should not come here yet.
#else
//  __ push(r1);
//  __ push(r0);
//  __ mov(r0, Operand(1));  // Set number of arguments.
  __ push(a1);
  __ push(a0);
  __ li(a0, Operand(1));  // Set number of arguments.
//  __ break_(0x5622);
  __ InvokeBuiltin(builtin, JUMP_JS);  // Tail call.  No return.
  __ nop(); // NOP_ADDED
#endif

  // We branch here if at least one of r0 and r1 is not a Smi.
  // Currently we should always get here. See comment about smi-smi case before.
  __ bind(not_smi);
  if (mode == NO_OVERWRITE) {
    // In the case where there is no chance of an overwritable float we may as
    // well do the allocation immediately while a0 and a1 are untouched.
    AllocateHeapNumber(masm, &slow, t5, t6, t7);
  }

  // Check if a0 is smi or not
  __ andi(t0, a0, Operand(kSmiTagMask));
  __ bcond(eq, &a0_is_smi, t0, Operand(zero_reg));  // It's a Smi so don't check it's a heap number.
  __ nop(); // NOP_ADDED
  __ GetObjectType(a0, t0, t0);
  __ bcond(ne, &slow, t0, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  if (mode == OVERWRITE_RIGHT) {
    __ mov(t5, a0);
  }
  // As we have only 2 arguments which are doubles, so we pass them in f12 and
  // f14 coprocessor registers.
  __ ldc1(f14, FieldMemOperand(a0, HeapNumber::kValueOffset));
  __ b(&finished_loading_a0);
  __ nop(); // NOP_ADDED
  __ bind(&a0_is_smi);      // BIND a0_is_smi
  if (mode == OVERWRITE_RIGHT) {
    // We can't overwrite a Smi so get address of new heap number into r5.
    AllocateHeapNumber(masm, &slow, t5, t6, t7);
  }
  // We move a0 to coprocessor and convert it to a double.
  __ mtc1(f14, a0);
  __ cvt_d_w(f14, f14);
  __ bind(&finished_loading_a0);    // BIND finished_loading_a0

  __ andi(t1, a1, Operand(kSmiTagMask));
  __ bcond(eq, &a1_is_smi, t1, Operand(zero_reg));  // It's a Smi so don't check it's a heap number.
  __ nop(); // NOP_ADDED
  __ GetObjectType(a1, t1, t1);
  __ bcond(ne, &slow, t1, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  if (mode == OVERWRITE_LEFT) {
    __ mov(t5, a1);  // Overwrite this heap number.
  }
  // Calling convention says that first double is in r0 and r1.
  __ ldc1(f12, FieldMemOperand(a1, HeapNumber::kValueOffset));
  __ b(&finished_loading_a1);
  __ nop(); // NOP_ADDED
  __ bind(&a1_is_smi);      // BIND a1_is_smi
  if (mode == OVERWRITE_LEFT) {
    // We can't overwrite a Smi so get address of new heap number into r5.
    AllocateHeapNumber(masm, &slow, t5, t6, t7);
  }
  // We move a0 to coprocessor and convert it to a double.
  __ mtc1(f12, a0);
  __ cvt_d_w(f12, f12);
  __ bind(&finished_loading_a1);    // BIND finished_loading_a1

  __ bind(&do_the_call);
  // f12: left value
  // f14: right value
  // t5: Address of heap number for result.
  __ addiu(sp, sp, -12);
  __ sw(s3, MemOperand(sp, 8));
  __ sw(ra, MemOperand(sp, 4));  // For later
  __ sw(t5, MemOperand(sp, 0));  // Address of heap number that is answer.
  // Call C routine that may not cause GC or other trouble.
  // We need to align sp as we use floating point, so we save it in s3.
  __ li(t9, Operand(ExternalReference::double_fp_operation(operation)));
  __ mov(s3, sp);                   // Save sp
  __ li(t3, Operand(~7));           // Load sp mask
  __ and_(sp, sp, Operand(t3));     // Align sp. We use the branch delay slot.
  __ Call(t9);                      // Call the code
  __ addiu(sp, sp, Operand(-StandardFrameConstants::kRArgsSlotsSize));
  __ mov(sp, s3);                   // Restore sp.
  // Store answer in the overwritable heap number.
  __ lw(t5, MemOperand(sp, 0));
  // Store double returned in f0
  __ sdc1(f0, MemOperand(t5, HeapNumber::kValueOffset - kHeapObjectTag));
  // Copy result address to v0
  __ mov(v0, t5);
//  __ break_(0x00109);
  // And we are done.
  __ lw(ra, MemOperand(sp, 4));
  __ lw(s3, MemOperand(sp, 8));
  __ Jump(ra);
  __ addiu(sp, sp, 12); // Restore sp
}


// Tries to get a signed int32 out of a double precision floating point heap
// number.  Rounds towards 0.  Fastest for doubles that are in the ranges
// -0x7fffffff to -0x40000000 or 0x40000000 to 0x7fffffff.  This corresponds
// almost to the range of signed int32 values that are not Smis.  Jumps to the
// label 'slow' if the double isn't in the range -0x80000000.0 to 0x80000000.0
// (excluding the endpoints).
static void GetInt32(MacroAssembler* masm,
                     Register source,
                     Register dest,
                     Label* slow) {
  
  // Load the double value.
  __ ldc1(f12, FieldMemOperand(source, HeapNumber::kValueOffset));
  // Convert it.
  __ cvt_w_d(f4, f12);
  __ mfc1(f4, dest);
  // TODO(MIPS.3): Implement case where we could not convert it.

}


// For bitwise ops where the inputs are not both Smis we here try to determine
// whether both inputs are either Smis or at least heap numbers that can be
// represented by a 32 bit signed value.  We truncate towards zero as required
// by the ES spec.  If this is the case we do the bitwise op and see if the
// result is a Smi.  If so, great, otherwise we try to find a heap number to
// write the answer into (either by allocating or by overwriting).
// On entry the operands are in a0 and a1.  On exit the answer is in v0.
void GenericBinaryOpStub::HandleNonSmiBitwiseOp(MacroAssembler* masm) {
  Label slow, result_not_a_smi;
  Label a0_is_smi, a1_is_smi;
  Label done_checking_a0, done_checking_a1;


  __ andi(t1, a1, Operand(kSmiTagMask));
  __ bcond(eq, &a1_is_smi, t1, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  __ GetObjectType(a1, t4, t4);
  __ bcond(ne, &slow, t4, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  GetInt32(masm, a1, a3, &slow);
  __ b(&done_checking_a1);
  __ bind(&a1_is_smi);
  __ sra(a3, a1, kSmiTagSize);
  __ bind(&done_checking_a1);

  __ andi(t0, a0, Operand(kSmiTagMask));
  __ bcond(eq, &a0_is_smi, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  __ GetObjectType(a0, t4, t4);
  __ bcond(ne, &slow, t4, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  GetInt32(masm, a0, a2, &slow);
  __ b(&done_checking_a0);
  __ bind(&a0_is_smi);
  __ sra(a2, a0, kSmiTagSize);
  __ bind(&done_checking_a0);

  // a0 and a1: Original operands (Smi or heap numbers).
  // a2 and a3: Signed int32 operands.

  switch (op_) {
    case Token::BIT_OR:  __ or_(v1, a2, Operand(a3)); break;
    case Token::BIT_XOR: __ xor_(v1, a2, Operand(a3)); break;
    case Token::BIT_AND: __ and_(v1, a2, Operand(a3)); break;
    case Token::SAR:
      __ srav(v1, a2, a3);
      break;
    case Token::SHR:
      __ srlv(v1, a2, a3);
      // SHR is special because it is required to produce a positive answer.
      // The code below for writing into heap numbers isn't capable of writing
      // the register as an unsigned int so we go to slow case if we hit this
      // case.
      __ andi(t3, v1, Operand(0x80000000));
      __ bcond(ne, &slow, t3, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      break;
    case Token::SHL:
        __ sllv(v1, a2, a3);
      break;
    default: UNREACHABLE();
  }
  // check that the *signed* result fits in a smi
  __ add(t3, v1, Operand(0x40000000));
  __ andi(t3, t3, Operand(0x80000000));
  __ bcond(ne, &slow, t3, Operand(zero_reg));
  __ nop(); // NOP_ADDED
  // Smi tag result.
  __ sll(v0, v1, kSmiTagMask);
  __ Ret();

  Label have_to_allocate, got_a_heap_number;
  __ bind(&result_not_a_smi);
  switch (mode_) {
    case OVERWRITE_RIGHT: {
      // t0 has not been changed since  __ andi(t0, a0, Operand(kSmiTagMask));
      __ bcond(eq, &have_to_allocate, t0, Operand(zero_reg));
      __ mov(t5, a0);
      break;
    }
    case OVERWRITE_LEFT: {
      // t1 has not been changed since  __ andi(t1, a1, Operand(kSmiTagMask));
      __ bcond(eq, &have_to_allocate, t1, Operand(zero_reg));
      __ nop(); // NOP_ADDED
      __ mov(t5, a1);
      break;
    }
    case NO_OVERWRITE: {
      // Get a new heap number in t5.  t6 and t7 are scratch.
      AllocateHeapNumber(masm, &slow, t5, t6, t7);
    }
    default: break;
  }

  __ bind(&got_a_heap_number);
  // v1: Answer as signed int32.
  // t5: Heap number to write answer into.

  // Nothing can go wrong now, so move the heap number to r0, which is the
  // result.
  __ mov(v0, t5);

  // Convert our int32 (v1) in our heap number (a0).
  __ mtc1(f12, v1);
  __ cvt_d_w(f4, f12);
  __ sdc1(f4, MemOperand(t5, HeapNumber::kValueOffset - kHeapObjectTag));

  if (mode_ != NO_OVERWRITE) {
    __ bind(&have_to_allocate);
    // Get a new heap number in t5.  t6 and t7 are scratch.
    AllocateHeapNumber(masm, &slow, t5, t6, t7);
    __ jmp(&got_a_heap_number);
    __ nop();   // NOP_ADDED
  }

  // If all else failed then we go to the runtime system.
  __ bind(&slow);
  __ push(a1);  // restore stack
  __ push(a0);
  __ li(a0, Operand(1));  // 1 argument (not counting receiver).
  switch (op_) {
    case Token::BIT_OR:
      __ InvokeBuiltin(Builtins::BIT_OR, JUMP_JS);
      break;
    case Token::BIT_AND:
      __ InvokeBuiltin(Builtins::BIT_AND, JUMP_JS);
      break;
    case Token::BIT_XOR:
      __ InvokeBuiltin(Builtins::BIT_XOR, JUMP_JS);
      break;
    case Token::SAR:
      __ InvokeBuiltin(Builtins::SAR, JUMP_JS);
      break;
    case Token::SHR:
      __ InvokeBuiltin(Builtins::SHR, JUMP_JS);
      break;
    case Token::SHL:
      __ InvokeBuiltin(Builtins::SHL, JUMP_JS);
      break;
    default:
      UNREACHABLE();
  }
}


// Can we multiply by x with max two shifts and an add.
// This answers yes to all integers from 2 to 10.
static bool IsEasyToMultiplyBy(int x) {
  if (x < 2) return false;                          // Avoid special cases.
  if (x > (Smi::kMaxValue + 1) >> 2) return false;  // Almost always overflows.
  if (IsPowerOf2(x)) return true;                   // Simple shift.
  if (PopCountLessThanEqual2(x)) return true;       // Shift and add and shift.
  if (IsPowerOf2(x + 1)) return true;               // Patterns like 11111.
  return false;
}


// Can multiply by anything that IsEasyToMultiplyBy returns true for.
// Source and destination may be the same register.  This routine does
// not set carry and overflow the way a mul instruction would.
static void MultiplyByKnownInt(MacroAssembler* masm,
                               Register source,
                               Register destination,
                               int known_int) {
  if (IsPowerOf2(known_int)) {
    __ sll(destination, source, BitPosition(known_int));
  } else if (PopCountLessThanEqual2(known_int)) {
    int first_bit = BitPosition(known_int);
    int second_bit = BitPosition(known_int ^ (1 << first_bit));
    __ sll(t0, source, second_bit - first_bit);
    __ add(destination, source, Operand(t0));
    if (first_bit != 0) {
      __ sll(destination, destination, first_bit);
    }
  } else {
    ASSERT(IsPowerOf2(known_int + 1));  // Patterns like 1111.
    int the_bit = BitPosition(known_int + 1);
    __ sll(t0, source, the_bit);
    __ sub(destination, t0, Operand(source));
  }
}


//// This function (as opposed to MultiplyByKnownInt) takes the known int in a
//// a register for the cases where it doesn't know a good trick, and may deliver
//// a result that needs shifting.
//static void MultiplyByKnownInt2(
//    MacroAssembler* masm,
//    Register result,
//    Register source,
//    Register known_int_register,   // Smi tagged.
//    int known_int,
//    int* required_shift) {  // Including Smi tag shift
//  switch (known_int) {
//    case 3:
//      __ add(result, source, Operand(source, LSL, 1));
//      *required_shift = 1;
//      break;
//    case 5:
//      __ add(result, source, Operand(source, LSL, 2));
//      *required_shift = 1;
//      break;
//    case 6:
//      __ add(result, source, Operand(source, LSL, 1));
//      *required_shift = 2;
//      break;
//    case 7:
//      __ rsb(result, source, Operand(source, LSL, 3));
//      *required_shift = 1;
//      break;
//    case 9:
//      __ add(result, source, Operand(source, LSL, 3));
//      *required_shift = 1;
//      break;
//    case 10:
//      __ add(result, source, Operand(source, LSL, 2));
//      *required_shift = 2;
//      break;
//    default:
//      ASSERT(!IsPowerOf2(known_int));  // That would be very inefficient.
//      __ mul(result, source, known_int_register);
//      *required_shift = 0;
//  }
//}


void GenericBinaryOpStub::Generate(MacroAssembler* masm) {
  // TODO(MIPS.1): Implement overflow cases.
  // a1 : x
  // a0 : y
  // result : v0

  // All ops need to know whether we are dealing with two Smis.  Set up r2 to
  // tell us that.
  __ or_(t2, a1, Operand(a0));  // t2 = x | y;

  switch (op_) {
    case Token::ADD: {
      Label not_smi;
      // Fast path.
      ASSERT(kSmiTag == 0);  // Adjust code below.
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &not_smi, t3, Operand(zero_reg));
      __ nop();
      __ Ret();
      __ addu(v0, a1, Operand(a0));  // Add y optimistically.

      HandleBinaryOpSlowCases(masm,
                              &not_smi,
                              Builtins::ADD,
                              Token::ADD,
                              mode_);
      break;
    }

    case Token::SUB: {
      Label not_smi;
//      // Fast path.
      ASSERT(kSmiTag == 0);  // Adjust code below.
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &not_smi, t3, Operand(zero_reg));
      __ nop();
      __ Ret();
      __ subu(v0, a1, Operand(a0));  // Subtract y optimistically.

      HandleBinaryOpSlowCases(masm,
                              &not_smi,
                              Builtins::SUB,
                              Token::SUB,
                              mode_);
      break;
    }

    case Token::MUL: {
      Label not_smi;
      ASSERT(kSmiTag == 0);  // adjust code below
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &not_smi, t3, Operand(zero_reg));
      __ nop();
      // Remove tag from one operand (but keep sign), so that result is Smi.
      __ sra(t0, a0, kSmiTagSize);
      // Do multiplication
      __ Ret();
      __ mul(v0, a1, Operand(t0));

      HandleBinaryOpSlowCases(masm,
                              &not_smi,
                              Builtins::MUL,
                              Token::MUL,
                              mode_);
      break;
    }

    case Token::DIV: {
      Label not_smi;
      ASSERT(kSmiTag == 0);  // adjust code below
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &not_smi, t3, Operand(zero_reg));
      __ nop();
      // Remove tags
      __ sra(t0, a0, kSmiTagSize);
      __ sra(t1, a1, kSmiTagSize);
      // Divide
      __ div(t1, Operand(t0));
      __ mflo(v0);
      __ Ret();
      __ sll(v0, v0, 1);

      HandleBinaryOpSlowCases(masm,
                              &not_smi,
                              op_ == Token::MOD ? Builtins::MOD : Builtins::DIV,
                              op_,
                              mode_);
      break;
    }

    case Token::MOD: {
      Label not_smi;
      ASSERT(kSmiTag == 0);  // adjust code below
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &not_smi, t3, Operand(zero_reg));
      __ nop();
      // Remove tag from one operand (but keep sign), so that result is Smi.
      __ sra(t0, a0, kSmiTagSize);
      __ div(a1, Operand(a0));
      __ Ret();
      __ mfhi(v0);

      HandleBinaryOpSlowCases(masm,
                              &not_smi,
                              op_ == Token::MOD ? Builtins::MOD : Builtins::DIV,
                              op_,
                              mode_);
      break;
    }

    case Token::BIT_OR:
    case Token::BIT_AND:
    case Token::BIT_XOR:
    case Token::SAR:
    case Token::SHR:
    case Token::SHL: {
      Label slow;
      ASSERT(kSmiTag == 0);  // adjust code below
      __ andi(t3, t2, Operand(kSmiTagMask));
      __ bcond(ne, &slow, t3, Operand(zero_reg));
      __ nop();
      switch (op_) {
        case Token::BIT_OR:  __ or_(v0, a0, Operand(a1)); break;
        case Token::BIT_AND: __ and_(v0, a0, Operand(a1)); break;
        case Token::BIT_XOR: __ xor_(v0, a0, Operand(a1)); break;
        case Token::SAR:
          // Remove tags from operands.
          __ sra(a2, a0, kSmiTagSize);
          __ sra(a3, a1, kSmiTagSize);
          // Shift
          __ srav(v0, a3, a2);
          // Smi tag result.
          __ sll(v0, v0, kSmiTagMask);
          break;
        case Token::SHR:
          // Remove tags from operands.
          __ sra(a2, a0, kSmiTagSize);
          __ sra(a3, a1, kSmiTagSize);
          // Shift
          __ srlv(v0, a3, a2);
          // Unsigned shift is not allowed to produce a negative number, so
          // check the sign bit and the sign bit after Smi tagging.
          __ andi(t3, v0, Operand(0xc0000000));
          __ bcond(ne, &slow, t3, Operand(zero_reg));
          __ nop(); // NOP_ADDED
          // Smi tag result.
          __ sll(v0, v0, kSmiTagMask);
          break;
        case Token::SHL:
          // Remove tags from operands.
          __ sra(a2, a0, kSmiTagSize);
          __ sra(a3, a1, kSmiTagSize);
          // Shift
          __ sllv(v0, a3, a2);
          // Check that the signed result fits in a Smi.
          __ addiu(t3, v0, Operand(0x40000000));
          __ andi(t3, t3, Operand(0x80000000));
          __ bcond(ne, &slow, t3, Operand(zero_reg));
          __ nop(); // NOP_ADDED
          // Smi tag result.
          __ sll(v0, v0, kSmiTagMask);
          break;
        default: UNREACHABLE();
      }
      __ Ret();
      __ bind(&slow);
      HandleNonSmiBitwiseOp(masm);
      break;
    }

    default: UNREACHABLE();
  }
  // This code should be unreachable.
  __ stop("Unreachable");
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  // Do tail-call to runtime routine.  Runtime routines expect at least one
  // argument, so give it a Smi.
//  __ mov(r0, Operand(Smi::FromInt(0)));
//  __ push(r0);
  __ li(a0, Operand(Smi::FromInt(0)));
  __ push(a0);
  __ addiu(sp, sp, -StandardFrameConstants::kRArgsSlotsSize);
  __ TailCallRuntime(ExternalReference(Runtime::kStackGuard), 1, 1);
  __ nop();
  __ addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);

  __ StubReturn(1);
}


void UnarySubStub::Generate(MacroAssembler* masm) {
  Label undo;
  Label slow;
  Label not_smi;

  // Enter runtime system if the value is not a smi.
//  __ tst(r0, Operand(kSmiTagMask));
//  __ b(ne, &not_smi);
  __ andi(t0, a0, Operand(kSmiTagMask));
  __ bcond(ne, &not_smi, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Enter runtime system if the value of the expression is zero
  // to make sure that we switch between 0 and -0.
//  __ cmp(r0, Operand(0));
//  __ b(eq, &slow);
  __ bcond(eq, &slow, a0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // The value of the expression is a smi that is not zero.  Try
  // optimistic subtraction '0 - value'.
//  __ rsb(r1, r0, Operand(0), SetCC);
//  __ b(vs, &slow);
  // Check for overflow. Since we substract from 0 the only overflow case is
  // when a0 is 0b10...0 = -(2^31)
  __ li(t0, Operand(1<<31));
  __ bcond(eq, &slow, a0, Operand(t0));
  __ subu(v0, zero_reg, Operand(a0));    // Optimistic sub in the branch delay slot.

//  __ mov(r0, Operand(r1));  // Set r0 to result.
  __ StubReturn(1);

  //TODO(MIPS.5): Call to InvokeBuiltin(Builtins::UNARY_MINUS, JUMP_JS);
  // Enter runtime system.
  __ bind(&slow);
  __ break_(0x6973);
////  __ push(r0);
////  __ mov(r0, Operand(0));  // Set number of arguments.
////  __ InvokeBuiltin(Builtins::UNARY_MINUS, JUMP_JS);
//  __ push(a0);
//  __ li(a0, Operand(0));  // Set number of arguments.
//  __ InvokeBuiltin(Builtins::UNARY_MINUS, JUMP_JS);
//  __ nop(); // NOP_ADDED

  __ bind(&not_smi);
//  __ CompareObjectType(r0, r1, r1, HEAP_NUMBER_TYPE);
//  __ b(ne, &slow);
  __ GetObjectType(a0, a1, a1);
  __ bcond(ne, &slow, a1, Operand(HEAP_NUMBER_TYPE));
  __ nop(); // NOP_ADDED
  // a0 is a heap number.  Get a new heap number in a1.
  if (overwrite_) {
//    __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
//    __ eor(r2, r2, Operand(HeapNumber::kSignMask));  // Flip sign.
//    __ str(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
    __ lw(a2, FieldMemOperand(a0, HeapNumber::kExponentOffset));
    __ xor_(a2, a2, Operand(HeapNumber::kSignMask));  // Flip sign.
    __ sw(a2, FieldMemOperand(a0, HeapNumber::kExponentOffset));
    __ mov(v0, a0); // Set result
  } else {
//    AllocateHeapNumber(masm, &slow, r1, r2, r3);
//    __ ldr(r3, FieldMemOperand(r0, HeapNumber::kMantissaOffset));
//    __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
//    __ str(r3, FieldMemOperand(r1, HeapNumber::kMantissaOffset));
//    __ eor(r2, r2, Operand(HeapNumber::kSignMask));  // Flip sign.
//    __ str(r2, FieldMemOperand(r1, HeapNumber::kExponentOffset));
//    __ mov(r0, Operand(r1));
    AllocateHeapNumber(masm, &slow, t1, t2, t3);
    __ lw(t3, FieldMemOperand(a0, HeapNumber::kMantissaOffset));
    __ lw(t2, FieldMemOperand(a0, HeapNumber::kExponentOffset));
    __ sw(t3, FieldMemOperand(t1, HeapNumber::kMantissaOffset));
    __ xor_(t2, t2, Operand(HeapNumber::kSignMask));  // Flip sign.
    __ sw(t2, FieldMemOperand(t1, HeapNumber::kExponentOffset));
    __ mov(v0, t1);
  }
  __ StubReturn(1);
}


int CEntryStub::MinorKey() {
  ASSERT(result_size_ <= 2);
  // Result returned in r0 or r0+r1 by default.
  return 0;
}


void CEntryStub::GenerateThrowTOS(MacroAssembler* masm) {
  // Called from CEntryStub::GenerateBody.

  // r0-> v0 holds the exception. cf l.6075 lrdi32(a0, Operand(reinterpret_cast<int32_t>(failure))); in CEntryStub::GenerateBody


  // Adjust this code if not the case.
  ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);

  // Drop the sp to the top of the handler.
//  __ mov(r3, Operand(ExternalReference(Top::k_handler_address)));
//  __ ldr(sp, MemOperand(r3));
  __ li(a3, Operand(ExternalReference(Top::k_handler_address)));
  __ lw(sp, MemOperand(a3));
  

  // Restore the next handler and frame pointer, discard handler state.
  ASSERT(StackHandlerConstants::kNextOffset == 0);
//  __ pop(r2);
//  __ str(r2, MemOperand(r3));
  __ pop(a2);
  __ sw(a2, MemOperand(a3));
  ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
//  __ ldm(ia_w, sp, r3.bit() | fp.bit());  // r3: discarded state.
  __ multi_pop_reversed(a3.bit() | fp.bit());

  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
//  __ cmp(fp, Operand(0));
//  // Set cp to NULL if fp is NULL.
//  __ mov(cp, Operand(0), LeaveCC, eq);
//  // Restore cp otherwise.
//  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset), ne);
  __ beq(fp, zero_reg, 2);
  __ mov(cp, zero_reg);
  __ lw(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
#ifdef DEBUG
//    printf("%s --- unimplemented debug\n", __func__);
//  if (FLAG_debug_code) {
////    __ mov(lr, Operand(pc));
//  }
#endif
  ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
//  __ pop(pc);
  __ pop(t9);
  __ jr(Operand(t9));
  __ nop(); // NOP_ADDED
}


void CEntryStub::GenerateThrowUncatchable(MacroAssembler* masm,
                                          UncatchableExceptionType type) {
  // Adjust this code if not the case.
  ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);

  // Drop sp to the top stack handler.
//  __ mov(r3, Operand(ExternalReference(Top::k_handler_address)));
//  __ ldr(sp, MemOperand(r3));
  __ li(sp, Operand(ExternalReference(Top::k_handler_address)));

  // Unwind the handlers until the ENTRY handler is found.
  Label loop, done;
  __ bind(&loop);
  // Load the type of the current stack handler.
  const int kStateOffset = StackHandlerConstants::kStateOffset;
//  __ ldr(r2, MemOperand(sp, kStateOffset));
//  __ cmp(r2, Operand(StackHandler::ENTRY));
//  __ b(eq, &done);
  __ lw(a2, MemOperand(sp, kStateOffset));
  __ bcond(eq, &done, a2, Operand(StackHandler::ENTRY));
  __ nop(); // NOP_ADDED
  // Fetch the next handler in the list.
  const int kNextOffset = StackHandlerConstants::kNextOffset;
//  __ ldr(sp, MemOperand(sp, kNextOffset));
//  __ jmp(&loop);
//  __ bind(&done);
  __ lw(sp, MemOperand(sp, kNextOffset));
  __ jmp(&loop);
  __ bind(&done);

  // Set the top handler address to next handler past the current ENTRY handler.
  ASSERT(StackHandlerConstants::kNextOffset == 0);
//  __ pop(r2);
//  __ str(r2, MemOperand(r3));
  __ pop(a2);
  __ sw(a2, MemOperand(a3));

  if (type == OUT_OF_MEMORY) {
    // Set external caught exception to false.
    ExternalReference external_caught(Top::k_external_caught_exception_address);
//    __ mov(r0, Operand(false));
//    __ mov(r2, Operand(external_caught));
//    __ str(r0, MemOperand(r2));
    __ li(a0, Operand(false));
    __ li(a2, Operand(external_caught));
    __ sw(a0, MemOperand(a2));

    // Set pending exception and r0 to out of memory exception.
    Failure* out_of_memory = Failure::OutOfMemoryException();
//    __ mov(r0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
//    __ mov(r2, Operand(ExternalReference(Top::k_pending_exception_address)));
//    __ str(r0, MemOperand(r2));
    __ li(a0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
    __ li(a2, Operand(ExternalReference(Top::k_pending_exception_address)));
    __ sw(a0, MemOperand(a2));
  }

  // Stack layout at this point. See also StackHandlerConstants.
  // sp ->   state (ENTRY)
  //         fp
  //         lr

  // Discard handler state (r2 is not used) and restore frame pointer.
  ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
//  __ ldm(ia_w, sp, r2.bit() | fp.bit());  // r2: discarded state.
  // TOCHECK: Is this correct?
  __ pop(fp);
  __ pop();
  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
//  __ cmp(fp, Operand(0));
  // Set cp to NULL if fp is NULL.
//  __ mov(cp, Operand(0), LeaveCC, eq);
  // Restore cp otherwise.
//  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset), ne);
  __ beq(fp, zero_reg, 2);
  __ mov(cp, zero_reg);
  __ lw(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
#ifdef DEBUG
  if (FLAG_debug_code) {
//    printf("%s --- unimplemented debug\n", __func__);
//    __ mov(lr, Operand(pc));
  }
#endif
  ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
//  __ pop(pc);
  __ pop(at);
  __ jr(Operand(at));
}

void CEntryStub::GenerateCore(MacroAssembler* masm,
                              Label* throw_normal_exception,
                              Label* throw_termination_exception,
                              Label* throw_out_of_memory_exception,
                              ExitFrame::Mode mode,
                              bool do_gc,
                              bool always_allocate) {
  // r0->a0: result parameter for PerformGC, if any
  // r4->s0: number of arguments including receiver  (C callee-saved)
  // r5->s1: pointer to builtin function  (C callee-saved)
  // r6->s2: pointer to the first argument (C callee-saved)

//  __ break_(0x00127);
  if (do_gc) {
    // Passing r0.
    ExternalReference gc_reference = ExternalReference::perform_gc_function();
    __ Call(gc_reference.address(), RelocInfo::RUNTIME_ENTRY);
    __ addiu(sp, sp, -StandardFrameConstants::kCArgsSlotsSize);
    __ addiu(sp, sp, StandardFrameConstants::kCArgsSlotsSize);
    __ nop();   // NOP_ADDED
  }

  ExternalReference scope_depth =
      ExternalReference::heap_always_allocate_scope_depth();
  if (always_allocate) {
//    __ mov(r0, Operand(scope_depth));
//    __ ldr(r1, MemOperand(r0));
//    __ add(r1, r1, Operand(1));
//    __ str(r1, MemOperand(r0));
    __ li(a0, Operand(scope_depth));
    __ lw(a1, MemOperand(a0));
    __ addi(a1, a1, 1);
    __ sw(a1, MemOperand(a0));
  }

  // Call C built-in.
  // r0 = argc, r1 = argv
//  __ mov(r0, Operand(r4));
//  __ mov(r1, Operand(r6));
//  __ break_(0x15642);
  __ mov(a0, s0);
  __ mov(a1, s2);

  // ARM TODO
  // TODO(1242173): To let the GC traverse the return address of the exit
  // frames, we need to know where the return address is. Right now,
  // we push it on the stack to be able to find it again, but we never
  // restore from it in case of changes, which makes it impossible to
  // support moving the C entry code stub. This should be fixed, but currently
  // this is OK because the CEntryStub gets generated so early in the V8 boot
  // sequence that it is not moving ever.

  // Current ra has already been saved in EnterExitFrame.
  // The push and StandardFrameConstants::kMarkerOffset are linked. See
  // StackFrame::ComputeType().
//  masm->add(lr, pc, Operand(4));  // compute return address: (pc + 8) + 4
//  masm->push(lr);
//  masm->Jump(r5);

  // Get future return address.
  masm->bal((int16_t)1);
  masm->addiu(sp, sp, Operand(-4));
  masm->addiu(ra, ra, Operand(9*4)); // 9 extra instructions
  masm->sw(ra, MemOperand(sp));

  // We don't use SetupAlignedCall because we need to know exactly how many
  // instructions we have here.
  masm->push(s3);                        // Save s3 on the stack
  masm->mov(s3, sp);                     // Save sp
  masm->li(t0, Operand(~7));        // Load sp mask
  masm->and_(sp, sp, Operand(t0));  // Align sp.
  
  masm->jalr(Operand(s1));
  masm->addiu(sp, sp, -StandardFrameConstants::kCArgsSlotsSize);

  masm->mov(sp, s3);      // Restore sp.
  masm->pop(s3);          // Restore s3
  

  if (always_allocate) {
    // It's okay to clobber r2->a2 and r3->a3 here. Don't mess with r0 and r1
    // though (contain the result).
//    __ mov(r2, Operand(scope_depth));
//    __ ldr(r3, MemOperand(r2));
//    __ sub(r3, r3, Operand(1));
//    __ str(r3, MemOperand(r2));
    __ li(a2, Operand(scope_depth));
    __ lw(a3, MemOperand(a2));
    __ addi(a3, a3, -1);
    __ sw(a3, MemOperand(a2));
  }

  // check for failure result
  Label failure_returned;
  ASSERT(((kFailureTag + 1) & kFailureTagMask) == 0);
  // Lower 2 bits of r2 are 0 if r0 has failure tag.
//  __ add(r2, r0, Operand(1));
//  __ tst(r2, Operand(kFailureTagMask));
//  __ b(eq, &failure_returned);
  __ addiu(a2, v0, 1);
  __ andi(ip, a2, Operand(kFailureTagMask));
  __ bcond(eq, &failure_returned, ip, Operand(zero_reg));
  __ nop();

  // Exit C frame and return.
  // r0:r1->v0:v1: result
  // sp: stack pointer
  // fp: frame pointer
  __ LeaveExitFrame(mode);

  // check if we should retry or throw exception
  Label retry;
  __ bind(&failure_returned);
//  __ break_(0x09990);
  ASSERT(Failure::RETRY_AFTER_GC == 0);
//  __ tst(r0, Operand(((1 << kFailureTypeTagSize) - 1) << kFailureTagSize));
//  __ b(eq, &retry);
  __ andi(t0, v0, Operand(((1 << kFailureTypeTagSize) - 1) << kFailureTagSize));
  __ bcond(eq, &retry, t0, Operand(zero_reg));
  __ nop(); // NOP_ADDED

  // Special handling of out of memory exceptions.
  Failure* out_of_memory = Failure::OutOfMemoryException();
//  __ cmp(r0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
//  __ b(eq, throw_out_of_memory_exception);
  __ bcond(eq, throw_out_of_memory_exception,
            v0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
  __ nop(); // NOP_ADDED

  // Retrieve the pending exception and clear the variable.
//  __ mov(ip, Operand(ExternalReference::the_hole_value_location()));
//  __ ldr(r3, MemOperand(ip));
//  __ mov(ip, Operand(ExternalReference(Top::k_pending_exception_address)));
//  __ ldr(r0, MemOperand(ip));
//  __ str(r3, MemOperand(ip));
  __ li(ip, Operand(ExternalReference::the_hole_value_location()));
  __ lw(a3, MemOperand(ip));
  __ li(ip, Operand(ExternalReference(Top::k_pending_exception_address)));
  __ lw(v0, MemOperand(ip));
  __ sw(a3, MemOperand(ip));

  // Special handling of termination exceptions which are uncatchable
  // by javascript code.
//  __ cmp(r0, Operand(Factory::termination_exception()));
//  __ b(eq, throw_termination_exception);
  __ bcond(eq, throw_termination_exception,
            v0, Operand(Factory::termination_exception()));
  __ nop(); // NOP_ADDED

  // Handle normal exception.
  __ jmp(throw_normal_exception);
  __ nop(); // NOP_ADDED

  __ bind(&retry);  // pass last failure (r0) as parameter (r0) when retrying
}

void CEntryStub::GenerateBody(MacroAssembler* masm, bool is_debug_break) {
  // Called from JavaScript; parameters are on stack as if calling JS function
  // a0: number of arguments including receiver
  // a1: pointer to builtin function
  // fp: frame pointer  (restored after C call)
  // sp: stack pointer  (restored as callee's sp after C call)
  // cp: current context  (C callee-saved)

  // NOTE: Invocations of builtins may return failure objects
  // instead of a proper result. The builtin entry handles
  // this by performing a garbage collection and retrying the
  // builtin once.

  ExitFrame::Mode mode = is_debug_break
      ? ExitFrame::MODE_DEBUG
      : ExitFrame::MODE_NORMAL;

  // Enter the exit frame that transitions from JavaScript to C++.
  __ EnterExitFrame(mode);

  // s0: number of arguments (C callee-saved)
  // s1: pointer to builtin function (C callee-saved)
  // s2: pointer to first argument (C callee-saved)

  Label throw_normal_exception;
  Label throw_termination_exception;
  Label throw_out_of_memory_exception;

  // Call into the runtime system.
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               mode,
               false,
               false);

  // Do space-specific GC and retry runtime call.
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               mode,
               true,
               false);

  // Do full GC and retry runtime call one final time.
  Failure* failure = Failure::InternalError();
  __ li(v0, Operand(reinterpret_cast<int32_t>(failure)));
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               mode,
               true,
               true);

  __ bind(&throw_out_of_memory_exception);
  GenerateThrowUncatchable(masm, OUT_OF_MEMORY);

  __ bind(&throw_termination_exception);
  GenerateThrowUncatchable(masm, TERMINATION);

  __ bind(&throw_normal_exception);
  GenerateThrowTOS(masm);
}

void JSEntryStub::GenerateBody(MacroAssembler* masm, bool is_construct) {

  // a0: code entry
  // a1: function
  // a2: receiver
  // a3: argc
  // [sp+16]: argv      (16 = 4*kPointerSize for arguments slots)

  Label invoke, exit;

  // Called from C, so do not pop argc and args on exit (preserve sp)
  // No need to save register-passed args
  // Save callee-saved registers (incl. cp and fp), sp, and ra
  __ multi_push(kCalleeSaved | ra.bit());

  // Load argv in s0 register.
  __ lw(s0, MemOperand(sp,(kNumCalleeSaved + 1)*kPointerSize +
                           StandardFrameConstants::kCArgsSlotsSize));

  // Push a frame with special values setup to mark it as an entry frame.
  // a0: code entry
  // a1: function
  // a2: receiver
  // a3: argc
  // s0: argv

  // We build an EntryFrame.
  // |                       |
  // |-----------------------|
  // |   bad fp (0xff...f)   |                     +
  // |-----------------------| <-- new fp         |
  // |     context slot      |                    |
  // |-----------------------|                    |
  // |    function slot      |                    v
  // |-----------------------|                     -
  // |      caller fp        |
  // |-----------------------|
  // |                       |

  __ li(t0, Operand(-1));  // Push a bad frame pointer to fail if it is used.
  int marker = is_construct ? StackFrame::ENTRY_CONSTRUCT : StackFrame::ENTRY;
  __ li(t1, Operand(Smi::FromInt(marker)));
  __ li(t2, Operand(Smi::FromInt(marker)));
  __ li(t3, Operand(ExternalReference(Top::k_c_entry_fp_address)));
  __ lw(t3, MemOperand(t3));
  __ multi_push(t0.bit() | t1.bit() | t2.bit() | t3.bit());

  // Setup frame pointer for the frame to be pushed.
  __ addiu(fp, sp, Operand(-EntryFrameConstants::kCallerFPOffset));

  // Call a faked try-block that does the invoke.
  __ bal(&invoke);
  __ nop(); // NOP_ADDED

  // Caught exception: Store result (exception) in the pending
  // exception field in the JSEnv and return a failure sentinel.
  // Coming in here the fp will be invalid because the PushTryHandler below
  // sets it to 0 to signal the existence of the JSEntry frame.
  __ li(ip, Operand(ExternalReference(Top::k_pending_exception_address)));
  __ sw(v0, MemOperand(ip));    // We come back from 'invoke'. result is in v0.
  __ li(v0, Operand(reinterpret_cast<int32_t>(Failure::Exception())));
  __ b(&exit);
  __ nop(); // NOP_ADDED

  // Invoke: Link this frame into the handler chain.
  __ bind(&invoke);
  // Must preserve a0-a3 and s0.
  __ PushTryHandler(IN_JS_ENTRY, JS_ENTRY_HANDLER);
  // If an exception not caught by another handler occurs, this handler
  // returns control to the code after the bal(&invoke) above, which
  // restores all kCalleeSaved registers (including cp and fp) to their
  // saved values before returning a failure to C.

  // Clear any pending exceptions.
  __ li(ip, Operand(ExternalReference::the_hole_value_location()));
  __ lw(t0, MemOperand(ip));
  __ li(ip, Operand(ExternalReference(Top::k_pending_exception_address)));
  __ sw(t0, MemOperand(ip));

  // Invoke the function by calling through JS entry trampoline builtin.
  // Notice that we cannot store a reference to the trampoline code directly in
  // this stub, because runtime stubs are not traversed when doing GC.

  // Expected registers by Builtins::JSEntryTrampoline
  // a0: code entry
  // a1: function
  // a2: receiver
  // a3: argc
  // s0: argv
  if (is_construct) {
    ExternalReference construct_entry(Builtins::JSConstructEntryTrampoline);
    __ li(ip, Operand(construct_entry));
  } else {
    ExternalReference entry(Builtins::JSEntryTrampoline);
    __ li(ip, Operand(entry));
  }
  __ lw(ip, MemOperand(ip));  // deref address

  // Branch and link to JSEntryTrampoline.
  __ addiu(t9, ip, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ jalr(Operand(t9));
  __ nop(); // NOP_ADDED

  // Unlink this frame from the handler chain. When reading the
  // address of the next handler, there is no need to use the address
  // displacement since the current stack pointer (sp) points directly
  // to the stack handler.
  __ lw(a3, MemOperand(sp, StackHandlerConstants::kNextOffset));
  __ li(ip, Operand(ExternalReference(Top::k_handler_address)));
  __ sw(a3, MemOperand(ip));

  // This restores sp to its position before PushTryHandler.
  __ addiu(sp, sp, Operand(StackHandlerConstants::kSize));


  __ bind(&exit);  // v0 holds result
  // Restore the top frame descriptors from the stack.
  __ pop(a3);
  __ li(ip, Operand(ExternalReference(Top::k_c_entry_fp_address)));
  __ sw(a3, MemOperand(ip));

  // Reset the stack to the callee saved registers.
  __ addiu(sp, sp, Operand(-EntryFrameConstants::kCallerFPOffset));

  // Restore callee-saved registers and return.
  __ multi_pop(kCalleeSaved | ra.bit());
  __ Jump(ra);
  __ nop(); // NOP_ADDED

}


// This stub performs an instanceof, calling the builtin function if
// necessary.  Uses a1 for the object, a0 for the function that it may
// be an instance of (these are fetched from the stack).
void InstanceofStub::Generate(MacroAssembler* masm) {
  // Get the object - slow case for smis (we may need to throw an exception
  // depending on the rhs).
  Label slow, loop, is_instance, is_not_instance;
//  __ ldr(r0, MemOperand(sp, 1 * kPointerSize));
//  __ BranchOnSmi(r0, &slow);
  __ lw(a0, MemOperand(sp, 1 * kPointerSize));
  __ BranchOnSmi(a0, &slow);
  __ nop(); // NOP_ADDED

  // Check that the left hand is a JS object and put map in r3.
//  __ CompareObjectType(r0, r3, r2, FIRST_JS_OBJECT_TYPE);
//  __ b(lt, &slow);
//  __ cmp(r2, Operand(LAST_JS_OBJECT_TYPE));
//  __ b(gt, &slow);
  __ GetObjectType(a0, a3, a2);
  __ bcond(less, &slow, a2, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED
  __ bcond(greater, &slow, a2, Operand(LAST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED

  // Get the prototype of the function (r4 is result, r2 is scratch).
//  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
//  __ TryGetFunctionPrototype(r1, r4, r2, &slow);
  __ lw(a1, MemOperand(sp, 0 * kPointerSize));
  __ TryGetFunctionPrototype(a1, t4, a2, &slow);

  // Check that the function prototype is a JS object.
//  __ BranchOnSmi(r4, &slow);
//  __ CompareObjectType(r4, r5, r5, FIRST_JS_OBJECT_TYPE);
//  __ b(lt, &slow);
//  __ cmp(r5, Operand(LAST_JS_OBJECT_TYPE));
//  __ b(gt, &slow);
  __ BranchOnSmi(t4, &slow);
  __ nop(); // NOP_ADDED
  __ GetObjectType(t4, t5, t5);
  __ bcond(less, &slow, t5, Operand(FIRST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED
  __ bcond(greater, &slow, t5, Operand(LAST_JS_OBJECT_TYPE));
  __ nop(); // NOP_ADDED

  // Register mapping: r3 is object map and r4 is function prototype.
  // Get prototype of object into r2.
//  __ ldr(r2, FieldMemOperand(r3, Map::kPrototypeOffset));
  __ lw(a2, FieldMemOperand(a3, Map::kPrototypeOffset));

  // Loop through the prototype chain looking for the function prototype.
  __ bind(&loop);
//  __ cmp(r2, Operand(r4));
//  __ b(eq, &is_instance);
//  __ LoadRoot(ip, Heap::kNullValueRootIndex);
//  __ cmp(r2, ip);
//  __ b(eq, &is_not_instance);
//  __ ldr(r2, FieldMemOperand(r2, HeapObject::kMapOffset));
//  __ ldr(r2, FieldMemOperand(r2, Map::kPrototypeOffset));
//  __ jmp(&loop);
  __ bcond(eq, &is_instance, a2, Operand(t4));
  __ nop(); // NOP_ADDED
  __ LoadRoot(ip, Heap::kNullValueRootIndex);
  __ bcond(eq, &is_not_instance, a2, Operand(ip));
  __ nop(); // NOP_ADDED
  __ lw(a2, FieldMemOperand(a2, HeapObject::kMapOffset));
  __ lw(a2, FieldMemOperand(a2, Map::kPrototypeOffset));
  __ b(&loop);
  __ nop(); // NOP_ADDED

  __ bind(&is_instance);
//  __ mov(r0, Operand(Smi::FromInt(0)));
//  __ pop();
//  __ pop();
//  __ mov(pc, Operand(lr));  // Return.
  __ li(v0, Operand(Smi::FromInt(0)));
  __ pop();
  __ pop();
  __ jr(ra);
  __ nop(); // NOP_ADDED

  __ bind(&is_not_instance);
  __ li(v0, Operand(Smi::FromInt(1)));
  __ pop();
  __ pop();
  __ jr(ra);
  __ nop(); // NOP_ADDED

  // Slow-case.  Tail call builtin.
  __ bind(&slow);
  // TODO(MIPS.5)
  __ break_(0x00666);
//  __ li(a0, Operand(1));  // Arg count without receiver.
//  __ InvokeBuiltin(Builtins::INSTANCE_OF, JUMP_JS);
//  __ nop(); // NOP_ADDED
}


void ArgumentsAccessStub::GenerateReadLength(MacroAssembler* masm) {
  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor;
//  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
//  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
//  __ cmp(r3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
//  __ b(eq, &adaptor);
  __ lw(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ lw(a3, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ bcond(eq, &adaptor, a3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ nop(); // NOP_ADDED

  // Nothing to do: The formal number of parameters has already been
  // passed in register r0 by calling function. Just return it.
  __ mov(v0,a0);
//  __ Jump(lr);
  __ Jump(ra);
  __ nop(); // NOP_ADDED

  // Arguments adaptor case: Read the arguments length from the
  // adaptor frame and return it.
  __ bind(&adaptor);
//  __ ldr(r0, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
//  __ Jump(lr);
  __ lw(v0, MemOperand(a2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ Jump(ra);
  __ nop(); // NOP_ADDED
}


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  // The displacement is the offset of the last parameter (if any)
  // relative to the frame pointer.
  static const int kDisplacement =
      StandardFrameConstants::kCallerSPOffset - kPointerSize;

  // Check that the key is a smiGenerateReadElement.
  Label slow;
//  __ BranchOnNotSmi(r1, &slow);
  __ BranchOnNotSmi(a1, &slow);

  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor;
//  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
//  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
//  __ cmp(r3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
//  __ b(eq, &adaptor);
  __ lw(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ lw(a3, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ bcond(eq, &adaptor, a3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ nop(); // NOP_ADDED

  // Check index against formal parameters count limit passed in
  // through register eax. Use unsigned comparison to get negative
  // check for free.
//  __ cmp(r1, r0);
//  __ b(cs, &slow);
  __ bcond(greater_equal, &slow, a1, Operand(a0));
  __ nop(); // NOP_ADDED

  // Read the argument from the stack and return it.
//  __ sub(r3, r0, r1);
//  __ add(r3, fp, Operand(r3, LSL, kPointerSizeLog2 - kSmiTagSize));
//  __ ldr(r0, MemOperand(r3, kDisplacement));
//  __ Jump(lr);
  __ sub(a0, a0, Operand(a1));
  __ sll(t3, a3, kPointerSizeLog2 - kSmiTagSize);
  __ addu(a3, fp, Operand(t3));
  __ lw(v0, MemOperand(a3, kDisplacement));
  __ Jump(ra);
  __ nop(); // NOP_ADDED

  // Arguments adaptor case: Check index against actual arguments
  // limit found in the arguments adaptor frame. Use unsigned
  // comparison to get negative check for free.
  __ bind(&adaptor);
//  __ ldr(r0, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
//  __ cmp(r1, r0);
//  __ b(cs, &slow);
  __ lw(a0, MemOperand(a2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ bcond(greater_equal, &slow, a1, Operand(a0));
  __ nop(); // NOP_ADDED

  // Read the argument from the adaptor frame and return it.
//  __ sub(r3, r0, r1);
//  __ add(r3, r2, Operand(r3, LSL, kPointerSizeLog2 - kSmiTagSize));
//  __ ldr(r0, MemOperand(r3, kDisplacement));
//  __ Jump(lr);
  __ sub(a3, a0, Operand(a1));
  __ sll(t3, a3, kPointerSizeLog2 - kSmiTagSize);
  __ addu(a3, a2, Operand(t3));
  __ lw(v0, MemOperand(a3, kDisplacement));
  __ Jump(ra);
  __ nop(); // NOP_ADDED

  // Slow-case: Handle non-smi or out-of-bounds access to arguments
  // by calling the runtime system.
  __ bind(&slow);
//  __ push(r1);
  __ push(a1);
  __ TailCallRuntime(ExternalReference(Runtime::kGetArgumentsProperty), 1, 1);
  __ nop(); // NOP_ADDED
}


void ArgumentsAccessStub::GenerateNewObject(MacroAssembler* masm) {
  // Check if the calling frame is an arguments adaptor frame.
  Label runtime;
//  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
//  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
//  __ cmp(r3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
//  __ b(ne, &runtime);
  __ lw(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ lw(a3, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ bcond(ne, &runtime, a3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ nop(); // NOP_ADDED

  // Patch the arguments.length and the parameters pointer.
//  __ ldr(r0, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
//  __ str(r0, MemOperand(sp, 0 * kPointerSize));
//  __ add(r3, r2, Operand(r0, LSL, kPointerSizeLog2 - kSmiTagSize));
//  __ add(r3, r3, Operand(StandardFrameConstants::kCallerSPOffset));
//  __ str(r3, MemOperand(sp, 1 * kPointerSize));
  __ lw(a0, MemOperand(a2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ sw(a0, MemOperand(sp, 0 * kPointerSize));
  __ sll(t0, a0, kPointerSizeLog2 - kSmiTagSize);
  __ add(a3, a2, t0);
  __ add(a3, a3, Operand(StandardFrameConstants::kCallerSPOffset));
  __ sw(a3, MemOperand(sp, 1 * kPointerSize));

  // Do the runtime call to allocate the arguments object.
  __ bind(&runtime);
  __ TailCallRuntime(ExternalReference(Runtime::kNewArgumentsFast), 3, 1);
  __ nop(); // NOP_ADDED
}


void CallFunctionStub::Generate(MacroAssembler* masm) {
  Label slow;
  __ break_(0x7108);
  // Get the function to call from the stack.
  // function, receiver [, arguments]
//  __ ldr(r1, MemOperand(sp, (argc_ + 1) * kPointerSize));
  __ lw(a1, MemOperand(sp, kPointerSize * (argc_)));
//                            + StandardFrameConstants::kRArgsSlotsSize +1) ));

  // Check that the function is really a JavaScript function.
  // a1: pushed function (to be verified)
  __ BranchOnSmi(a1, &slow, t0);
  __ nop(); // NOP_ADDED
  // Get the map of the function object.
  __ GetObjectType(a1, a2, a2);
  __ bcond(ne, &slow, a2, Operand(JS_FUNCTION_TYPE));
  __ nop(); // NOP_ADDED

  // Fast-case: Invoke the function now.
  // a1: pushed function
  ParameterCount actual(argc_);
  __ InvokeFunction(a1, actual, JUMP_FUNCTION, false);
//  // Args slots are allocated in InvokeFunction.
//  __ addiu(sp, sp, StandardFrameConstants::kRArgsSlotsSize);

  // Slow-case: Non-function called.
  __ bind(&slow);
//  __ mov(r0, Operand(argc_));  // Setup the number of arguments.
//  __ mov(r2, Operand(0));
//  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
//  __ Jump(Handle<Code>(Builtins::builtin(Builtins::ArgumentsAdaptorTrampoline)),
//          RelocInfo::CODE_TARGET);
#ifdef NO_NATIVES
  UNIMPLEMENTED();
  __ break_(0x7162);
#else
  __ li(a0, Operand(argc_));  // Setup the number of arguments.
  __ li(a2, Operand(0));
  __ GetBuiltinEntry(a3, Builtins::CALL_NON_FUNCTION);
  __ break_(0x7147);
  __ li(v1, Operand(1)); // Tell ArgumentsAdaptorTrampoline we need args slots
  __ Jump(Handle<Code>(Builtins::builtin(Builtins::ArgumentsAdaptorTrampoline)),
          RelocInfo::CODE_TARGET);
  __ nop(); // NOP_ADDED
#endif
}


int CompareStub::MinorKey() {
  // Encode the two parameters in a unique 16 bit value.
  ASSERT(static_cast<unsigned>(cc_) >> 28 < (1 << 15));
  return (static_cast<unsigned>(cc_) >> 27) | (strict_ ? 1 : 0);
}


#undef __

} }  // namespace v8::internal
