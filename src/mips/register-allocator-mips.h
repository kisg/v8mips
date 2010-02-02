#ifndef V8_MIPS_REGISTER_ALLOCATOR_MIPS_H_
#define V8_MIPS_REGISTER_ALLOCATOR_MIPS_H_

#include	"mips/constants-mips.h"

namespace v8 {
namespace internal {

class RegisterAllocatorConstants : public AllStatic {
 public:
  static const int kNumRegisters = assembler::mips::kNumRegisters;
  static const int kInvalidRegister = assembler::mips::kInvalidRegister;
};


} }  // namespace v8::internal

#endif  // V8_MIPS_REGISTER_ALLOCATOR_MIPS_H_
