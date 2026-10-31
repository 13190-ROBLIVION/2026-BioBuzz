#include "Calculations.h"

// The math is currently defined inline in Calculations.h.
// This translation unit stays in the CMake build so that future
// out-of-line implementations (e.g. iterative solvers, filters)
// compile into libftcrobotcontroller.so without touching CMakeLists.txt.
