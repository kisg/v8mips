#!/bin/sh

INTERFACE=test-interface-mips.cc
TESTCORE=test-mips.cc
TESTCORE_O=$PWD/obj/debug/mips/test-mips.o
PATH_TO_TEST=$PWD/src/mips
ARGS="-Wall -Werror"
V8ARGS="-Iinclude libv8_g.a -lpthread"

echo "g++ ${PATH_TO_TEST}/${INTERFACE} -o mips-test-interface ${TESTCORE_O} ${V8ARGS} ${ARGS}"
g++ ${PATH_TO_TEST}/${INTERFACE} -o mips-test-interface ${TESTCORE_O} ${V8ARGS} ${ARGS}
