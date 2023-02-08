//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// trisolve.cpp
//
// Code generation for function 'trisolve'
//

// Include files
#include "trisolve.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace internal {
void trisolve(const double A[3721], double B[3721])
{
  for (int j{0}; j < 61; j++) {
    int jBcol;
    jBcol = 61 * j - 1;
    for (int k{0}; k < 61; k++) {
      double d;
      int i;
      int kAcol;
      kAcol = 61 * k - 1;
      i = (k + jBcol) + 1;
      d = B[i];
      if (d != 0.0) {
        int i1;
        B[i] = d / A[(k + kAcol) + 1];
        i1 = k + 2;
        for (int b_i{i1}; b_i < 62; b_i++) {
          int i2;
          i2 = b_i + jBcol;
          B[i2] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

} // namespace internal
} // namespace coder
} // namespace pos_MPC

// End of code generation (trisolve.cpp)
