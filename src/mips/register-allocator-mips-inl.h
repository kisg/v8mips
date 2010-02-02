#ifndef V8_IA32_REGISTER_ALLOCATOR_IA32_INL_H_
#define V8_IA32_REGISTER_ALLOCATOR_IA32_INL_H_

#include "v8.h"
#include "mips/assembler-mips.h"

namespace v8 {
namespace internal {

// -------------------------------------------------------------------------
// RegisterAllocator implementation.

// TODO : implement bool RegisterAllocator::IsReserved
bool RegisterAllocator::IsReserved(Register reg) {
  // The code for this test relies on the order of register codes.
  return reg.is(cp) || reg.is(s8_fp) || reg.is(sp);
}


// TODO : modifiy kNumbers for reserved registers
// TODO : cf other architectures code

int RegisterAllocator::ToNumber(Register reg) {
  ASSERT(reg.is_valid() && !IsReserved(reg));
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
		14,	// t
		15,	// t7
		16,	// t8
		17,	// t9
		18,	// s0
		19,	// s1
		20,	// s2
		21,	// s3
		22,	// s4
		23,	// s5
		24,	// s6
		25,	// s7
		26,	// k0
		27,	// k1
		28,	// gp
		29,	// sp
		30,	// s8_fp
		31,	// ra
  };
  return kNumbers[reg.code()];
}


Register RegisterAllocator::ToRegister(int num) {
  ASSERT(num >= 0 && num < kNumRegisters);
  const Register kRegisters[] = {
		zero_reg,
		at,
		v0,
		v1,
		a0,
		a1,
		a2,
		a3,
		t0,
		t1,
		t2,
		t3,
		t4,
		t5,
		t6,
		t7,
		s0,
		s1,
		s2,
		s3,
		s4,
		s5,
		s6,
		s7,
		t8,
		t9,
		k0,
		k1,
		gp,
		sp,
		s8_fp,
		ra
  };
  return kRegisters[num];
}


void RegisterAllocator::Initialize() {
  Reset();
  // The non-reserved a1 and ra registers are live on JS function entry.
  Use(a1);  // JS function.
  Use(ra);  // Return address.
}


} }  // namespace v8::internal

#endif  // V8_IA32_REGISTER_ALLOCATOR_IA32_INL_H_
