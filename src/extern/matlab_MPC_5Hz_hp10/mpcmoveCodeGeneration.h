//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcmoveCodeGeneration.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef MPCMOVECODEGENERATION_H
#define MPCMOVECODEGENERATION_H

// Include files
#include "mpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
extern void mpcmoveCodeGeneration(struct4_T *mpcmovestate,
                                  const struct5_T *mpcmovedata, double u[3],
                                  struct10_T *Info);

}
} // namespace pos_MPC

#endif
// End of code generation (mpcmoveCodeGeneration.h)
