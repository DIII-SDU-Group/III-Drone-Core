//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xpotrf.cpp
//
// Code generation for function 'xpotrf'
//

// Include files
#include "xpotrf.h"
#include "mpcmoveCodeGeneration_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace internal {
namespace lapack {
int xpotrf(double A[2116])
{
  int info;
  int j;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 46)) {
    double c;
    double ssq;
    int idxAjj;
    int k;
    idxAjj = j + j * 46;
    ssq = 0.0;
    if (j >= 1) {
      for (k = 0; k < j; k++) {
        c = A[j + k * 46];
        ssq += c * c;
      }
    }
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 46) {
        int i;
        int ia0;
        int idxAjp1j;
        ia0 = j + 2;
        idxAjp1j = idxAjj + 2;
        if (j != 0) {
          i = (j + 46 * (j - 1)) + 2;
          for (int iac{ia0}; iac <= i; iac += 46) {
            k = iac - j;
            c = -A[j + div_nde_s32_floor(k - 2, 46) * 46];
            k += 44;
            for (int ia{iac}; ia <= k; ia++) {
              int i1;
              i1 = ((idxAjj + ia) - iac) + 1;
              A[i1] += A[ia - 1] * c;
            }
          }
        }
        ssq = 1.0 / ssq;
        i = (idxAjj - j) + 46;
        for (k = idxAjp1j; k <= i; k++) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder
} // namespace pos_MPC

// End of code generation (xpotrf.cpp)
