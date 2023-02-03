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
void qpkwik(const double Linv[256], const double Hinv[256], const double f[16],
            const double Ac[960], const double b[60], short iA[60],
            double x[16], double lambda[60], double *status);

}
} // namespace pos_MPC

#endif
// End of code generation (qpkwik.h)
