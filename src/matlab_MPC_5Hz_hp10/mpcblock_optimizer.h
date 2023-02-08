//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcblock_optimizer.h
//
// Code generation for function 'mpcblock_optimizer'
//

#ifndef MPCBLOCK_OPTIMIZER_H
#define MPCBLOCK_OPTIMIZER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
void mpcblock_optimizer(
    const double rseq[60], const double vseq[11], const double umin[3],
    const double umax[3], const double ymin[6], const double ymax[6],
    const double x[9], const double old_u[3], const boolean_T iA[120],
    const double Mlim[120], const double Mx[1080], const double Mu1[360],
    const double Mv[1320], const double utarget[30], const double uoff[3],
    double H[961], const double Ac[3720], const double ywt[6],
    const double uwt[3], const double duwt[3], const double Jm[900],
    const double SuJm[1800], const double Su1[180], const double Sx[540],
    const double Hv[660], const double I1[90], const double Mrows[120],
    double u[3], double *cost, double useq[33], double *status,
    boolean_T iAout[120], double *slack);

}
} // namespace pos_MPC

#endif
// End of code generation (mpcblock_optimizer.h)
