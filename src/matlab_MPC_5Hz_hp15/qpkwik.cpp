//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// qpkwik.cpp
//
// Code generation for function 'qpkwik'
//

// Include files
#include "qpkwik.h"
#include "minOrMax.h"
#include "mpcmoveCodeGeneration_rtwutil.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
namespace pos_MPC {
namespace coder {
static void DropConstraint(short kDrop, short iA[180], short *nA,
                           short iC[180]);

static double KWIKfactor(const double Ac[8280], const short iC[180], short nA,
                         const double Linv[2116], double RLinv[2116],
                         double D[2116], double H[2116]);

} // namespace coder
static double rt_hypotd_snf(double u0, double u1);

} // namespace pos_MPC

// Function Definitions
namespace pos_MPC {
namespace coder {
static void DropConstraint(short kDrop, short iA[180], short *nA, short iC[180])
{
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    short i;
    i = static_cast<short>(*nA - 1);
    for (short b_i{kDrop}; b_i <= i; b_i++) {
      iC[b_i - 1] = iC[b_i];
    }
  }
  iC[*nA - 1] = 0;
  *nA = static_cast<short>(*nA - 1);
}

static double KWIKfactor(const double Ac[8280], const short iC[180], short nA,
                         const double Linv[2116], double RLinv[2116],
                         double D[2116], double H[2116])
{
  double A[2116];
  double Q[2116];
  double R[2116];
  double TL[2116];
  double tau[46];
  double work[46];
  double Status;
  double c;
  int b_i;
  int exitg1;
  int i;
  int i1;
  int i2;
  int ia;
  int iac;
  int ii;
  int ix0;
  int knt;
  int lastc;
  int lastv;
  boolean_T exitg2;
  Status = 1.0;
  std::memset(&RLinv[0], 0, 2116U * sizeof(double));
  i = nA;
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < 46; i1++) {
      i2 = i1 + 46 * b_i;
      RLinv[i2] = 0.0;
      for (ix0 = 0; ix0 < 46; ix0++) {
        RLinv[i2] += Linv[i1 + 46 * ix0] * Ac[(iC[b_i] + 180 * ix0) - 1];
      }
    }
  }
  std::copy(&RLinv[0], &RLinv[2116], &A[0]);
  std::memset(&tau[0], 0, 46U * sizeof(double));
  std::memset(&work[0], 0, 46U * sizeof(double));
  for (b_i = 0; b_i < 46; b_i++) {
    ii = b_i * 46 + b_i;
    if (b_i + 1 < 46) {
      double atmp;
      atmp = A[ii];
      ix0 = ii + 2;
      tau[b_i] = 0.0;
      c = internal::blas::xnrm2(45 - b_i, A, ii + 2);
      if (c != 0.0) {
        double beta1;
        double d;
        d = A[ii];
        beta1 = rt_hypotd_snf(d, c);
        if (d >= 0.0) {
          beta1 = -beta1;
        }
        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = -1;
          i1 = (ii - b_i) + 46;
          do {
            knt++;
            for (lastc = ix0; lastc <= i1; lastc++) {
              A[lastc - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while (!(std::abs(beta1) >= 1.0020841800044864E-292));
          beta1 =
              rt_hypotd_snf(atmp, internal::blas::xnrm2(45 - b_i, A, ii + 2));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }
          tau[b_i] = (beta1 - atmp) / beta1;
          c = 1.0 / (atmp - beta1);
          for (lastc = ix0; lastc <= i1; lastc++) {
            A[lastc - 1] *= c;
          }
          for (lastc = 0; lastc <= knt; lastc++) {
            beta1 *= 1.0020841800044864E-292;
          }
          atmp = beta1;
        } else {
          tau[b_i] = (beta1 - d) / beta1;
          c = 1.0 / (d - beta1);
          i1 = (ii - b_i) + 46;
          for (lastc = ix0; lastc <= i1; lastc++) {
            A[lastc - 1] *= c;
          }
          atmp = beta1;
        }
      }
      A[ii] = 1.0;
      if (tau[b_i] != 0.0) {
        lastv = 46 - b_i;
        knt = (ii - b_i) + 45;
        while ((lastv > 0) && (A[knt] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 45 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          knt = (ii + (lastc - 1) * 46) + 46;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia + 1 <= knt + lastv) {
              if (A[ia] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        ix0 = ii + 47;
        if (lastc != 0) {
          if (0 <= lastc - 1) {
            std::memset(&work[0], 0, lastc * sizeof(double));
          }
          i1 = (ii + 46 * (lastc - 1)) + 47;
          for (iac = ix0; iac <= i1; iac += 46) {
            c = 0.0;
            i2 = (iac + lastv) - 1;
            for (ia = iac; ia <= i2; ia++) {
              c += A[ia - 1] * A[(ii + ia) - iac];
            }
            knt = div_nde_s32_floor((iac - ii) - 47, 46);
            work[knt] += c;
          }
        }
        internal::blas::xgerc(lastv, lastc, -tau[b_i], ii + 1, work, A,
                              ii + 47);
      }
      A[ii] = atmp;
    } else {
      tau[45] = 0.0;
    }
  }
  for (ix0 = 0; ix0 < 46; ix0++) {
    for (b_i = 0; b_i <= ix0; b_i++) {
      knt = b_i + 46 * ix0;
      R[knt] = A[knt];
    }
    i1 = ix0 + 2;
    if (i1 <= 46) {
      std::memset(&R[(ix0 * 46 + i1) + -1], 0, (47 - i1) * sizeof(double));
    }
    work[ix0] = 0.0;
  }
  for (b_i = 45; b_i >= 0; b_i--) {
    ii = (b_i + b_i * 46) + 47;
    if (b_i + 1 < 46) {
      A[ii - 47] = 1.0;
      if (tau[b_i] != 0.0) {
        lastv = 46 - b_i;
        knt = ii - b_i;
        while ((lastv > 0) && (A[knt - 2] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 45 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          knt = ii + (lastc - 1) * 46;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia <= (knt + lastv) - 1) {
              if (A[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        if (lastc != 0) {
          if (0 <= lastc - 1) {
            std::memset(&work[0], 0, lastc * sizeof(double));
          }
          i1 = ii + 46 * (lastc - 1);
          for (iac = ii; iac <= i1; iac += 46) {
            c = 0.0;
            i2 = (iac + lastv) - 1;
            for (ia = iac; ia <= i2; ia++) {
              c += A[ia - 1] * A[((ii + ia) - iac) - 47];
            }
            knt = div_nde_s32_floor(iac - ii, 46);
            work[knt] += c;
          }
        }
        internal::blas::xgerc(lastv, lastc, -tau[b_i], ii - 46, work, A, ii);
      }
      ix0 = ii - 45;
      i1 = (ii - b_i) - 1;
      for (lastc = ix0; lastc <= i1; lastc++) {
        A[lastc - 1] *= -tau[b_i];
      }
    }
    A[ii - 47] = 1.0 - tau[b_i];
    for (ix0 = 0; ix0 < b_i; ix0++) {
      A[(ii - ix0) - 48] = 0.0;
    }
  }
  for (ix0 = 0; ix0 < 46; ix0++) {
    std::copy(&A[ix0 * 46], &A[static_cast<int>(ix0 * 46 + 46U)], &Q[ix0 * 46]);
  }
  b_i = 0;
  do {
    exitg1 = 0;
    if (b_i <= nA - 1) {
      if (std::abs(R[b_i + 46 * b_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        b_i++;
      }
    } else {
      short j;
      short k;
      for (b_i = 0; b_i < 46; b_i++) {
        for (ix0 = 0; ix0 < 46; ix0++) {
          c = 0.0;
          for (i1 = 0; i1 < 46; i1++) {
            c += Linv[i1 + 46 * b_i] * Q[i1 + 46 * ix0];
          }
          TL[b_i + 46 * ix0] = c;
        }
      }
      std::memset(&RLinv[0], 0, 2116U * sizeof(double));
      for (j = nA; j >= 1; j--) {
        i1 = 46 * (j - 1);
        i2 = (j + i1) - 1;
        RLinv[i2] = 1.0;
        for (k = j; k <= nA; k++) {
          ix0 = (j + 46 * (k - 1)) - 1;
          RLinv[ix0] /= R[i2];
        }
        if (j > 1) {
          i2 = j;
          for (b_i = 0; b_i <= i2 - 2; b_i++) {
            for (k = j; k <= nA; k++) {
              ix0 = 46 * (k - 1);
              knt = b_i + ix0;
              RLinv[knt] -= R[b_i + i1] * RLinv[(j + ix0) - 1];
            }
          }
        }
      }
      for (b_i = 0; b_i < 46; b_i++) {
        short c_i;
        c_i = static_cast<short>(b_i + 1);
        for (j = c_i; j < 47; j++) {
          short i3;
          i1 = b_i + 46 * (j - 1);
          H[i1] = 0.0;
          i3 = static_cast<short>(nA + 1);
          for (k = i3; k < 47; k++) {
            i2 = 46 * (k - 1);
            H[i1] -= TL[b_i + i2] * TL[(j + i2) - 1];
          }
          H[(j + 46 * b_i) - 1] = H[i1];
        }
      }
      for (ix0 = 0; ix0 < i; ix0++) {
        j = static_cast<short>(ix0 + 1);
        for (b_i = 0; b_i < 46; b_i++) {
          i1 = b_i + 46 * ix0;
          D[i1] = 0.0;
          for (k = j; k <= nA; k++) {
            i2 = 46 * (k - 1);
            D[i1] += TL[b_i + i2] * RLinv[ix0 + i2];
          }
        }
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return Status;
}

} // namespace coder
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

namespace coder {
void qpkwik(const double Linv[2116], const double Hinv[2116],
            const double f[46], const double Ac[8280], const double b[180],
            short iA[180], double x[46], double lambda[180], double *status)
{
  double D[2116];
  double H[2116];
  double RLinv[2116];
  double U[2116];
  double cTol[180];
  double Opt[92];
  double Rhs[92];
  double r[46];
  double z[46];
  double cMin;
  double cVal;
  double rMin;
  int b_i;
  int ct;
  int i;
  int i1;
  int k;
  short iC[180];
  short c_i;
  short kDrop;
  short nA;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T guard1{false};
  *status = 1.0;
  std::memset(&lambda[0], 0, 180U * sizeof(double));
  std::memset(&x[0], 0, 46U * sizeof(double));
  std::memset(&r[0], 0, 46U * sizeof(double));
  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 180; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }
  nA = 0;
  for (i = 0; i < 180; i++) {
    if (iA[i] == 1) {
      nA = static_cast<short>(nA + 1);
      iC[nA - 1] = static_cast<short>(i + 1);
    }
  }
  guard1 = false;
  if (nA > 0) {
    int exitg3;
    unsigned short b_x;
    unsigned short q;
    std::memset(&Opt[0], 0, 92U * sizeof(double));
    for (i = 0; i < 46; i++) {
      Rhs[i] = f[i];
      Rhs[i + 46] = 0.0;
    }
    DualFeasible = false;
    kDrop = static_cast<short>(3 * nA);
    if (kDrop <= 50) {
      kDrop = 50;
    }
    q = static_cast<unsigned short>(static_cast<unsigned short>(kDrop) / 10U);
    b_x = static_cast<unsigned short>(
        static_cast<unsigned int>(static_cast<unsigned short>(kDrop)) -
        static_cast<unsigned short>(q * 10));
    if ((b_x > 0) && (b_x >= 5)) {
      q = static_cast<unsigned short>(q + 1);
    }
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= 904.0)) {
        cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
        if (cVal < 0.0) {
          if (ColdReset) {
            *status = -2.0;
            exitg3 = 2;
          } else {
            nA = 0;
            std::memset(&iA[0], 0, 180U * sizeof(short));
            std::memset(&iC[0], 0, 180U * sizeof(short));
            ColdReset = true;
          }
        } else {
          b_i = nA;
          for (int j{0}; j < b_i; j++) {
            kDrop = static_cast<short>(j + 1);
            Rhs[static_cast<short>(j + 47) - 1] = b[iC[j] - 1];
            for (c_i = kDrop; c_i <= nA; c_i++) {
              ct = (c_i + 46 * j) - 1;
              U[ct] = 0.0;
              i1 = nA;
              for (k = 0; k < i1; k++) {
                i = 46 * k;
                U[ct] += RLinv[(c_i + i) - 1] * RLinv[j + i];
              }
              U[j + 46 * (c_i - 1)] = U[ct];
            }
          }
          b_i = nA;
          for (i = 0; i < 46; i++) {
            cVal = 0.0;
            for (i1 = 0; i1 < 46; i1++) {
              cVal += H[i + 46 * i1] * Rhs[i1];
            }
            Opt[i] = cVal;
            for (k = 0; k < b_i; k++) {
              Opt[i] += D[i + 46 * k] * Rhs[k + 46];
            }
          }
          b_i = nA;
          for (i = 0; i < b_i; i++) {
            cVal = 0.0;
            for (i1 = 0; i1 < 46; i1++) {
              cVal += D[i1 + 46 * i] * Rhs[i1];
            }
            Opt[i + 46] = cVal;
            i1 = nA;
            for (k = 0; k < i1; k++) {
              Opt[i + 46] += U[i + 46 * k] * Rhs[k + 46];
            }
          }
          cVal = -1.0E-12;
          kDrop = 0;
          b_i = nA;
          for (i = 0; i < b_i; i++) {
            cMin = Opt[static_cast<short>(i + 47) - 1];
            lambda[iC[i] - 1] = cMin;
            if ((cMin < cVal) && (i + 1 <= nA)) {
              kDrop = static_cast<short>(i + 1);
              cVal = cMin;
            }
          }
          if (kDrop <= 0) {
            DualFeasible = true;
            std::copy(&Opt[0], &Opt[46], &x[0]);
          } else {
            (*status)++;
            if (static_cast<int>(*status) > q) {
              nA = 0;
              std::memset(&iA[0], 0, 180U * sizeof(short));
              std::memset(&iC[0], 0, 180U * sizeof(short));
              ColdReset = true;
            } else {
              lambda[iC[kDrop - 1] - 1] = 0.0;
              DropConstraint(kDrop, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          std::memset(&lambda[0], 0, 180U * sizeof(double));
          for (i = 0; i < 46; i++) {
            cVal = 0.0;
            for (b_i = 0; b_i < 46; b_i++) {
              cVal += -Hinv[i + 46 * b_i] * f[b_i];
            }
            x[i] = cVal;
          }
        }
        exitg3 = 1;
      }
    } while (exitg3 == 0);
    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (i = 0; i < 46; i++) {
      cVal = 0.0;
      for (b_i = 0; b_i < 46; b_i++) {
        cVal += -Hinv[i + 46 * b_i] * f[b_i];
      }
      x[i] = cVal;
    }
    guard1 = true;
  }
  if (guard1) {
    double Xnorm0;
    boolean_T exitg2;
    Xnorm0 = b_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= 904.0)) {
      double b_Ac;
      short kNext;
      cMin = -1.0E-6;
      kNext = 0;
      for (i = 0; i < 180; i++) {
        if (!cTolComputed) {
          for (k = 0; k < 46; k++) {
            z[k] = std::abs(Ac[i + 180 * k] * x[k]);
          }
          cTol[i] = std::fmax(cTol[i], internal::maximum(z));
        }
        if (iA[i] == 0) {
          b_Ac = 0.0;
          for (b_i = 0; b_i < 46; b_i++) {
            b_Ac += Ac[i + 180 * b_i] * x[b_i];
          }
          cVal = (b_Ac - b[i]) / cTol[i];
          if (cVal < cMin) {
            cMin = cVal;
            kNext = static_cast<short>(i + 1);
          }
        }
      }
      cTolComputed = true;
      if (kNext <= 0) {
        exitg2 = true;
      } else if (*status == 904.0) {
        *status = 0.0;
        exitg2 = true;
      } else {
        int exitg1;
        do {
          exitg1 = 0;
          if ((kNext > 0) && (*status <= 904.0)) {
            boolean_T guard2{false};
            guard2 = false;
            if (nA == 0) {
              for (b_i = 0; b_i < 46; b_i++) {
                cMin = 0.0;
                for (i1 = 0; i1 < 46; i1++) {
                  cMin += Hinv[b_i + 46 * i1] * Ac[(kNext + 180 * i1) - 1];
                }
                z[b_i] = cMin;
              }
              guard2 = true;
            } else {
              cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
              if (cVal <= 0.0) {
                *status = -2.0;
                exitg1 = 1;
              } else {
                for (b_i = 0; b_i < 2116; b_i++) {
                  U[b_i] = -H[b_i];
                }
                for (b_i = 0; b_i < 46; b_i++) {
                  cMin = 0.0;
                  for (i1 = 0; i1 < 46; i1++) {
                    cMin += U[b_i + 46 * i1] * Ac[(kNext + 180 * i1) - 1];
                  }
                  z[b_i] = cMin;
                }
                b_i = nA;
                for (i = 0; i < b_i; i++) {
                  b_Ac = 0.0;
                  for (i1 = 0; i1 < 46; i1++) {
                    b_Ac += Ac[(kNext + 180 * i1) - 1] * D[i1 + 46 * i];
                  }
                  r[i] = b_Ac;
                }
                guard2 = true;
              }
            }
            if (guard2) {
              double t1;
              boolean_T exitg4;
              kDrop = 0;
              t1 = 0.0;
              ColdReset = true;
              DualFeasible = true;
              if (nA > 0) {
                ct = 0;
                exitg4 = false;
                while ((!exitg4) && (ct <= nA - 1)) {
                  if (r[ct] >= 1.0E-12) {
                    DualFeasible = false;
                    exitg4 = true;
                  } else {
                    ct++;
                  }
                }
              }
              if ((nA != 0) && (!DualFeasible)) {
                b_i = nA;
                for (i = 0; i < b_i; i++) {
                  cMin = r[i];
                  if (cMin > 1.0E-12) {
                    cMin = lambda[iC[i] - 1] / cMin;
                    if ((kDrop == 0) || (cMin < rMin)) {
                      rMin = cMin;
                      kDrop = static_cast<short>(i + 1);
                    }
                  }
                }
                if (kDrop > 0) {
                  t1 = rMin;
                  ColdReset = false;
                }
              }
              cVal = 0.0;
              for (k = 0; k < 46; k++) {
                cVal += z[k] * Ac[(kNext + 180 * k) - 1];
              }
              if (cVal <= 0.0) {
                cVal = 0.0;
                DualFeasible = true;
              } else {
                b_Ac = 0.0;
                for (b_i = 0; b_i < 46; b_i++) {
                  b_Ac += Ac[(kNext + 180 * b_i) - 1] * x[b_i];
                }
                cVal = (b[kNext - 1] - b_Ac) / cVal;
                DualFeasible = false;
              }
              if (ColdReset && DualFeasible) {
                *status = -1.0;
                exitg1 = 1;
              } else {
                if (DualFeasible) {
                  cMin = t1;
                } else if (ColdReset) {
                  cMin = cVal;
                } else {
                  cMin = std::fmin(t1, cVal);
                }
                b_i = nA;
                for (i = 0; i < b_i; i++) {
                  ct = iC[i] - 1;
                  lambda[ct] -= cMin * r[i];
                  if (lambda[ct] < 0.0) {
                    lambda[ct] = 0.0;
                  }
                }
                lambda[kNext - 1] += cMin;
                if (cMin == t1) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }
                if (!DualFeasible) {
                  for (b_i = 0; b_i < 46; b_i++) {
                    x[b_i] += cMin * z[b_i];
                  }
                  if (cMin == cVal) {
                    if (nA == 46) {
                      *status = -1.0;
                      exitg1 = 1;
                    } else {
                      nA = static_cast<short>(nA + 1);
                      iC[nA - 1] = kNext;
                      c_i = nA;
                      exitg4 = false;
                      while ((!exitg4) && (c_i > 1)) {
                        short i2;
                        kDrop = iC[c_i - 1];
                        i2 = iC[c_i - 2];
                        if (kDrop > i2) {
                          exitg4 = true;
                        } else {
                          iC[c_i - 1] = i2;
                          iC[c_i - 2] = kDrop;
                          c_i = static_cast<short>(c_i - 1);
                        }
                      }
                      iA[kNext - 1] = 1;
                      kNext = 0;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cVal = b_norm(x);
            if (std::abs(cVal - Xnorm0) > 0.001) {
              Xnorm0 = cVal;
              for (k = 0; k < 180; k++) {
                cTol[k] = std::fmax(std::abs(b[k]), 1.0);
              }
              cTolComputed = false;
            }
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

} // namespace coder
} // namespace pos_MPC

// End of code generation (qpkwik.cpp)
