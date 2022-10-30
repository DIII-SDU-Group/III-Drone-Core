//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcmoveCodeGeneration_types.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef MPCMOVECODEGENERATION_TYPES_H
#define MPCMOVECODEGENERATION_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
namespace pos_MPC {
struct struct10_T {
  double Uopt[63];
  double Yopt[126];
  double Xopt[189];
  double Topt[21];
  double Slack;
  double Iterations;
  double Cost;
};

struct struct6_T {
  double ym[6];
  double ref[6];
};

struct struct7_T {
  double y[6];
  double u[3];
  double du[3];
};

struct struct8_T {
  double MVMax[3];
  double MVMin[3];
  double OutputMax[6];
  double OutputMin[6];
};

struct struct5_T {
  struct6_T signals;
  struct7_T weights;
  struct8_T limits;
};

struct struct4_T {
  double Plant[6];
  double Disturbance[3];
  double LastMove[3];
  double Covariance[81];
  boolean_T iA[240];
};

} // namespace pos_MPC

#endif
// End of code generation (mpcmoveCodeGeneration_types.h)
