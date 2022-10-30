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
void qpkwik(const double Linv[3025], const double Hinv[3025],
            const double f[55], const double Ac[12540], const double b[228],
            short iA[228], double x[55], double lambda[228], double *status);

}
} // namespace pos_MPC

#endif
// End of code generation (qpkwik.h)
