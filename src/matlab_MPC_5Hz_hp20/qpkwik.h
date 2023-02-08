//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// qpkwik.h
//
// Code generation for function 'qpkwik'
//

#ifndef QPKWIK_H
#define QPKWIK_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
void qpkwik(const double Linv[3721], const double Hinv[3721],
            const double f[61], const double Ac[14640], const double b[240],
            short iA[240], double x[61], double lambda[240], double *status);

}
} // namespace pos_MPC

#endif
// End of code generation (qpkwik.h)
