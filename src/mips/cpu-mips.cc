// FROM ARM code below
// CPU specific code for arm independent of OS goes here.

#include "v8.h"
#include "cpu.h"

#include <sys/syscall.h> 
#include <unistd.h>

#ifdef __mips
#include <asm/cachectl.h>
#endif // #ifdef __mips

namespace v8 {
namespace internal {

void CPU::Setup() {
  // Nothing to do.
}

// TODO : Implement void CPU::FlushICache(void* start, size_t size) {
void CPU::FlushICache(void* start, size_t size) {
#ifdef __mips
  int res;
  
//  int cacheflush(char *addr, int nbytes, int cache);
// See  http://www.linux-mips.org/wiki/Cacheflush_Syscall
  res = syscall(__NR_cacheflush, start, size, ICACHE);

  if(res)
    V8_Fatal(__FILE__, __LINE__, "Failed to flush the instruction cache");

#endif    // #ifdef __mips
}


void CPU::DebugBreak() {
#ifdef __mips
  asm volatile("break");
#endif // #ifdef __mips
}

} }  // namespace v8::internal
