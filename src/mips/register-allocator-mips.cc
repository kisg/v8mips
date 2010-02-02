#include "v8.h"

#include "codegen-inl.h"
#include "register-allocator-inl.h"

namespace v8 {
namespace internal {

// -------------------------------------------------------------------------
// Result implementation.

void Result::ToRegister() {
  UNIMPLEMENTED();
}


void Result::ToRegister(Register target) {
  UNIMPLEMENTED();
}


// -------------------------------------------------------------------------
// RegisterAllocator implementation.

Result RegisterAllocator::AllocateByteRegisterWithoutSpilling() {
  // No byte registers on MIPS.
  UNREACHABLE();
  return Result();
}


} }  // namespace v8::internal
