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
void qpkwik(const double Linv[2116], const double Hinv[2116],
            const double f[46], const double Ac[8280], const double b[180],
            short iA[180], double x[46], double lambda[180], double *status);

}
} // namespace pos_MPC

#endif
// End of code generation (qpkwik.h)
