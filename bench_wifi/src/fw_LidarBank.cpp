// RUN mode: compiles the UNMODIFIED match firmware file ../../src/LidarBank.cpp in
// its own translation unit (its anonymous namespaces stay private).
// If src/ gains a new .cpp, add a wrapper like this one.
#include "../../src/LidarBank.cpp"
