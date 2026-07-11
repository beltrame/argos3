# Converts a binary file into a C++ header defining a byte array.
# Used to embed compiled Filament materials (.filamat) into plugins.
#
# Usage:
#   cmake -DINPUT=<file> -DOUTPUT=<header> -DSYMBOL=<identifier> -P ARGoSHexifyFile.cmake

file(READ ${INPUT} _hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytes ${_hex})
file(WRITE ${OUTPUT}
"/* Generated from ${INPUT} - do not edit */
#include <cstddef>
namespace argos {
   extern const unsigned char ${SYMBOL}[] = {
${_bytes}
   };
   extern const size_t ${SYMBOL}_SIZE = sizeof(${SYMBOL});
}
")
