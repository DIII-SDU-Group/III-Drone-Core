//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcblock_optimizer.cpp
//
// Code generation for function 'mpcblock_optimizer'
//

// Include files
#include "mpcblock_optimizer.h"
#include "minOrMax.h"
#include "qpkwik.h"
#include "rt_nonfinite.h"
#include "trisolve.h"
#include "xpotrf.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace pos_MPC {
static double rt_powd_snf(double u0, double u1);

}

// Function Definitions
namespace pos_MPC {
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

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
    boolean_T iAout[120], double *slack)
{
  static const signed char A[100]{
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
      0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  double WySuJm[1800];
  double Linv[961];
  double b_Linv[961];
  double I2Jm[900];
  double WduJm[900];
  double b_SuJm[900];
  double b_WuI2Jm[900];
  double Kv[330];
  double Kx[270];
  double Bc[120];
  double b_Mlim[120];
  double b_I1[90];
  double b_Su1[90];
  double aux2[60];
  double b_Sx[60];
  double aux[31];
  double varargin_1[31];
  double K[30];
  double aux3[30];
  double Wy[6];
  double ymax_incr[6];
  double ymin_incr[6];
  double Wdu[3];
  double Wu[3];
  double umin_incr[3];
  double WuI2Jm;
  double normH;
  double s;
  int Tries;
  int WuI2Jm_tmp;
  int b_i;
  int b_kidx;
  int i;
  int i1;
  int kidx;
  short iAnew[120];
  short b_ixw;
  short ixw;
  signed char b[961];
  signed char b_I[9];
  boolean_T ymax_incr_flag[6];
  boolean_T ymin_incr_flag[6];
  boolean_T umax_incr_flag[3];
  boolean_T umin_incr_flag[3];
  boolean_T guard1{false};
  *cost = 0.0;
  std::memset(&iAout[0], 0, 120U * sizeof(boolean_T));
  *slack = 0.0;
  for (kidx = 0; kidx < 6; kidx++) {
    WuI2Jm = ywt[kidx];
    if (WuI2Jm < 0.0) {
      Wy[kidx] = 0.0;
    } else {
      Wy[kidx] = WuI2Jm * WuI2Jm;
    }
  }
  if (uwt[0] < 0.0) {
    Wu[0] = 0.0;
  } else {
    Wu[0] = uwt[0] * uwt[0];
  }
  if (duwt[0] < 0.0) {
    Wdu[0] = 0.0;
  } else {
    Wdu[0] = duwt[0] * duwt[0];
  }
  if (uwt[1] < 0.0) {
    Wu[1] = 0.0;
  } else {
    Wu[1] = uwt[1] * uwt[1];
  }
  if (duwt[1] < 0.0) {
    Wdu[1] = 0.0;
  } else {
    Wdu[1] = duwt[1] * duwt[1];
  }
  if (uwt[2] < 0.0) {
    Wu[2] = 0.0;
  } else {
    Wu[2] = uwt[2] * uwt[2];
  }
  if (duwt[2] < 0.0) {
    Wdu[2] = 0.0;
  } else {
    Wdu[2] = duwt[2] * duwt[2];
  }
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  b_kidx = -1;
  for (Tries = 0; Tries < 10; Tries++) {
    for (kidx = 0; kidx < 3; kidx++) {
      signed char b_i1;
      signed char i2;
      signed char i3;
      b_i1 = b_I[3 * kidx];
      i2 = b_I[3 * kidx + 1];
      i3 = b_I[3 * kidx + 2];
      for (i1 = 0; i1 < 10; i1++) {
        WuI2Jm_tmp = A[i1 + 10 * Tries];
        b_WuI2Jm[b_kidx + 1] = WuI2Jm_tmp * b_i1;
        b_WuI2Jm[b_kidx + 2] = WuI2Jm_tmp * i2;
        b_WuI2Jm[b_kidx + 3] = WuI2Jm_tmp * i3;
        b_kidx += 3;
      }
    }
  }
  for (i = 0; i < 30; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      WuI2Jm = 0.0;
      for (b_kidx = 0; b_kidx < 30; b_kidx++) {
        WuI2Jm += b_WuI2Jm[i + 30 * b_kidx] * Jm[b_kidx + 30 * i1];
      }
      I2Jm[i + 30 * i1] = WuI2Jm;
    }
  }
  ixw = 1;
  for (b_i = 0; b_i < 60; b_i++) {
    for (i = 0; i < 30; i++) {
      kidx = b_i + 60 * i;
      WySuJm[kidx] = Wy[ixw - 1] * SuJm[kidx];
    }
    ixw = static_cast<short>(ixw + 1);
    if (ixw > 6) {
      ixw = 1;
    }
  }
  ixw = 1;
  b_ixw = 1;
  for (b_i = 0; b_i < 30; b_i++) {
    for (i = 0; i < 30; i++) {
      WuI2Jm_tmp = b_i + 30 * i;
      b_WuI2Jm[WuI2Jm_tmp] = Wu[ixw - 1] * I2Jm[WuI2Jm_tmp];
    }
    ixw = static_cast<short>(ixw + 1);
    if (ixw > 3) {
      ixw = 1;
    }
    for (i = 0; i < 30; i++) {
      kidx = b_i + 30 * i;
      WduJm[kidx] = Wdu[b_ixw - 1] * Jm[kidx];
    }
    b_ixw = static_cast<short>(b_ixw + 1);
    if (b_ixw > 3) {
      b_ixw = 1;
    }
    for (i = 0; i < 30; i++) {
      WuI2Jm = 0.0;
      for (i1 = 0; i1 < 60; i1++) {
        WuI2Jm += SuJm[i1 + 60 * b_i] * WySuJm[i1 + 60 * i];
      }
      b_SuJm[b_i + 30 * i] = WuI2Jm;
    }
  }
  for (i = 0; i < 30; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      WuI2Jm = 0.0;
      s = 0.0;
      for (b_kidx = 0; b_kidx < 30; b_kidx++) {
        kidx = b_kidx + 30 * i;
        Tries = b_kidx + 30 * i1;
        s += Jm[kidx] * WduJm[Tries];
        WuI2Jm += I2Jm[kidx] * b_WuI2Jm[Tries];
      }
      H[i + 31 * i1] = (b_SuJm[i + 30 * i1] + s) + WuI2Jm;
    }
  }
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      WuI2Jm = 0.0;
      for (b_kidx = 0; b_kidx < 60; b_kidx++) {
        WuI2Jm += Su1[b_kidx + 60 * i] * WySuJm[b_kidx + 60 * i1];
      }
      kidx = i + 3 * i1;
      b_Su1[kidx] = WuI2Jm;
      WuI2Jm = 0.0;
      for (b_kidx = 0; b_kidx < 30; b_kidx++) {
        WuI2Jm += I1[b_kidx + 30 * i] * b_WuI2Jm[b_kidx + 30 * i1];
      }
      b_I1[kidx] = WuI2Jm;
    }
  }
  for (i = 0; i < 90; i++) {
    b_Su1[i] += b_I1[i];
  }
  for (i = 0; i < 900; i++) {
    b_WuI2Jm[i] = -b_WuI2Jm[i];
  }
  for (i = 0; i < 9; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      WuI2Jm = 0.0;
      for (b_kidx = 0; b_kidx < 60; b_kidx++) {
        WuI2Jm += Sx[b_kidx + 60 * i] * WySuJm[b_kidx + 60 * i1];
      }
      Kx[i + 9 * i1] = WuI2Jm;
    }
  }
  for (i = 0; i < 11; i++) {
    for (i1 = 0; i1 < 30; i1++) {
      WuI2Jm = 0.0;
      for (b_kidx = 0; b_kidx < 60; b_kidx++) {
        WuI2Jm += Hv[b_kidx + 60 * i] * WySuJm[b_kidx + 60 * i1];
      }
      Kv[i + 11 * i1] = WuI2Jm;
    }
  }
  for (i = 0; i < 1800; i++) {
    WySuJm[i] = -WySuJm[i];
  }
  b_kidx = 0;
  for (i = 0; i < 961; i++) {
    WuI2Jm = H[i];
    b_Linv[i] = WuI2Jm;
    Linv[i] = WuI2Jm;
  }
  kidx = internal::lapack::xpotrf(Linv);
  guard1 = false;
  if (kidx == 0) {
    for (kidx = 0; kidx < 31; kidx++) {
      varargin_1[kidx] = Linv[kidx + 31 * kidx];
    }
    if (!(internal::minimum(varargin_1) > 1.4901161193847656E-7)) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    boolean_T exitg2;
    normH = 0.0;
    b_i = 0;
    exitg2 = false;
    while ((!exitg2) && (b_i < 31)) {
      s = 0.0;
      for (Tries = 0; Tries < 31; Tries++) {
        s += std::abs(H[b_i + 31 * Tries]);
      }
      if (std::isnan(s)) {
        normH = rtNaN;
        exitg2 = true;
      } else {
        if (s > normH) {
          normH = s;
        }
        b_i++;
      }
    }
    if (normH >= 1.0E+10) {
      b_kidx = 2;
    } else {
      boolean_T exitg1;
      Tries = 0;
      exitg1 = false;
      while ((!exitg1) && (Tries <= 4)) {
        boolean_T guard2{false};
        normH = rt_powd_snf(10.0, static_cast<double>(Tries)) *
                1.4901161193847656E-7;
        std::memset(&b[0], 0, 961U * sizeof(signed char));
        for (kidx = 0; kidx < 31; kidx++) {
          b[kidx + 31 * kidx] = 1;
        }
        for (i = 0; i < 961; i++) {
          WuI2Jm = b_Linv[i] + normH * static_cast<double>(b[i]);
          b_Linv[i] = WuI2Jm;
          Linv[i] = WuI2Jm;
        }
        kidx = internal::lapack::xpotrf(Linv);
        guard2 = false;
        if (kidx == 0) {
          for (kidx = 0; kidx < 31; kidx++) {
            varargin_1[kidx] = Linv[kidx + 31 * kidx];
          }
          if (internal::minimum(varargin_1) > 1.4901161193847656E-7) {
            b_kidx = 1;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          b_kidx = 3;
          Tries++;
        }
      }
    }
  }
  if (b_kidx > 1) {
    for (b_i = 0; b_i < 3; b_i++) {
      u[b_i] = old_u[b_i] + uoff[b_i];
      for (i = 0; i < 11; i++) {
        useq[i + 11 * b_i] = u[b_i];
      }
    }
    *status = -2.0;
  } else {
    double b_Kv;
    std::memset(&b[0], 0, 961U * sizeof(signed char));
    for (kidx = 0; kidx < 31; kidx++) {
      b[kidx + 31 * kidx] = 1;
    }
    for (Tries = 0; Tries < 31; Tries++) {
      for (b_i = 0; b_i < 31; b_i++) {
        kidx = b_i + 31 * Tries;
        b_Linv[kidx] = b[kidx];
      }
    }
    internal::trisolve(Linv, b_Linv);
    for (i = 0; i < 120; i++) {
      WuI2Jm = 0.0;
      for (i1 = 0; i1 < 9; i1++) {
        WuI2Jm += Mx[i + 120 * i1] * x[i1];
      }
      s = 0.0;
      for (i1 = 0; i1 < 11; i1++) {
        s += Mv[i + 120 * i1] * vseq[i1];
      }
      Bc[i] = -(
          ((Mlim[i] + WuI2Jm) + ((Mu1[i] * old_u[0] + Mu1[i + 120] * old_u[1]) +
                                 Mu1[i + 240] * old_u[2])) +
          s);
    }
    for (i = 0; i < 6; i++) {
      ymax_incr_flag[i] = false;
      ymax_incr[i] = 0.0;
      ymin_incr_flag[i] = false;
      ymin_incr[i] = 0.0;
    }
    umax_incr_flag[0] = false;
    Wdu[0] = 0.0;
    umin_incr_flag[0] = false;
    umin_incr[0] = 0.0;
    umax_incr_flag[1] = false;
    Wdu[1] = 0.0;
    umin_incr_flag[1] = false;
    umin_incr[1] = 0.0;
    umax_incr_flag[2] = false;
    Wdu[2] = 0.0;
    umin_incr_flag[2] = false;
    umin_incr[2] = 0.0;
    for (b_i = 0; b_i < 120; b_i++) {
      WuI2Jm = Mrows[b_i];
      if (WuI2Jm <= 60.0) {
        if (WuI2Jm - 1.0 == 0.0) {
          normH = 0.0;
        } else {
          normH = std::fmod(WuI2Jm - 1.0, 6.0);
          if (normH == 0.0) {
            normH = 0.0;
          }
        }
        if (!ymax_incr_flag[static_cast<int>(normH + 1.0) - 1]) {
          s = -ymax[static_cast<int>(normH + 1.0) - 1] - (-Mlim[b_i]);
        } else {
          s = ymax_incr[static_cast<int>(normH + 1.0) - 1];
        }
        ymax_incr[static_cast<int>(normH + 1.0) - 1] = s;
        ymax_incr_flag[static_cast<int>(normH + 1.0) - 1] = true;
        Bc[b_i] += s;
      } else if (WuI2Jm <= 120.0) {
        if ((WuI2Jm - 60.0) - 1.0 == 0.0) {
          normH = 0.0;
        } else {
          normH = std::fmod((WuI2Jm - 60.0) - 1.0, 6.0);
          if (normH == 0.0) {
            normH = 0.0;
          }
        }
        if (!ymin_incr_flag[static_cast<int>(normH + 1.0) - 1]) {
          s = ymin[static_cast<int>(normH + 1.0) - 1] - (-Mlim[b_i]);
        } else {
          s = ymin_incr[static_cast<int>(normH + 1.0) - 1];
        }
        ymin_incr[static_cast<int>(normH + 1.0) - 1] = s;
        ymin_incr_flag[static_cast<int>(normH + 1.0) - 1] = true;
        Bc[b_i] += s;
      } else if (WuI2Jm <= 150.0) {
        if ((WuI2Jm - 120.0) - 1.0 == 0.0) {
          normH = 0.0;
        } else {
          normH = std::fmod((WuI2Jm - 120.0) - 1.0, 3.0);
          if (normH == 0.0) {
            normH = 0.0;
          }
        }
        if (!umax_incr_flag[static_cast<int>(normH + 1.0) - 1]) {
          s = -umax[static_cast<int>(normH + 1.0) - 1] - (-Mlim[b_i]);
        } else {
          s = Wdu[static_cast<int>(normH + 1.0) - 1];
        }
        Wdu[static_cast<int>(normH + 1.0) - 1] = s;
        umax_incr_flag[static_cast<int>(normH + 1.0) - 1] = true;
        Bc[b_i] += s;
      } else {
        if (std::isnan(((WuI2Jm - 120.0) - 30.0) - 1.0)) {
          normH = rtNaN;
        } else if (((WuI2Jm - 120.0) - 30.0) - 1.0 == 0.0) {
          normH = 0.0;
        } else {
          normH = std::fmod(((WuI2Jm - 120.0) - 30.0) - 1.0, 3.0);
          if (normH == 0.0) {
            normH = 0.0;
          }
        }
        if (!umin_incr_flag[static_cast<int>(normH + 1.0) - 1]) {
          s = umin[static_cast<int>(normH + 1.0) - 1] - (-Mlim[b_i]);
        } else {
          s = umin_incr[static_cast<int>(normH + 1.0) - 1];
        }
        umin_incr[static_cast<int>(normH + 1.0) - 1] = s;
        umin_incr_flag[static_cast<int>(normH + 1.0) - 1] = true;
        Bc[b_i] += s;
      }
    }
    std::memset(&varargin_1[0], 0, 31U * sizeof(double));
    for (b_i = 0; b_i < 30; b_i++) {
      normH = 0.0;
      for (i = 0; i < 9; i++) {
        normH += Kx[i + 9 * b_i] * x[i];
      }
      s = 0.0;
      for (i = 0; i < 60; i++) {
        s += WySuJm[i + 60 * b_i] * rseq[i];
      }
      b_Kv = 0.0;
      for (i = 0; i < 11; i++) {
        b_Kv += Kv[i + 11 * b_i] * vseq[i];
      }
      WuI2Jm = 0.0;
      for (i = 0; i < 30; i++) {
        WuI2Jm += b_WuI2Jm[i + 30 * b_i] * utarget[i];
      }
      varargin_1[b_i] =
          (((normH + s) +
            ((b_Su1[3 * b_i] * old_u[0] + b_Su1[3 * b_i + 1] * old_u[1]) +
             b_Su1[3 * b_i + 2] * old_u[2])) +
           b_Kv) +
          WuI2Jm;
    }
    for (b_i = 0; b_i < 120; b_i++) {
      iAnew[b_i] = iA[b_i];
    }
    for (i = 0; i < 31; i++) {
      for (i1 = 0; i1 < 31; i1++) {
        WuI2Jm = 0.0;
        for (b_kidx = 0; b_kidx < 31; b_kidx++) {
          WuI2Jm += b_Linv[b_kidx + 31 * i] * b_Linv[b_kidx + 31 * i1];
        }
        Linv[i + 31 * i1] = WuI2Jm;
      }
    }
    qpkwik(b_Linv, Linv, varargin_1, Ac, Bc, iAnew, aux, b_Mlim, status);
    for (b_i = 0; b_i < 120; b_i++) {
      iAout[b_i] = (iAnew[b_i] != 0);
    }
    if ((*status < 0.0) || (*status == 0.0)) {
      std::memset(&aux[0], 0, 31U * sizeof(double));
    }
    *slack = aux[30];
    u[0] = old_u[0] + aux[0];
    u[1] = old_u[1] + aux[1];
    u[2] = old_u[2] + aux[2];
    if (*status > 0.0) {
      for (i = 0; i < 60; i++) {
        WuI2Jm = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          WuI2Jm += Sx[i + 60 * i1] * x[i1];
        }
        s = 0.0;
        for (i1 = 0; i1 < 11; i1++) {
          s += Hv[i + 60 * i1] * vseq[i1];
        }
        aux2[i] = ((WuI2Jm + ((Su1[i] * old_u[0] + Su1[i + 60] * old_u[1]) +
                              Su1[i + 120] * old_u[2])) +
                   s) -
                  rseq[i];
      }
      WuI2Jm = old_u[0];
      s = old_u[1];
      normH = old_u[2];
      for (i = 0; i < 30; i++) {
        aux3[i] = ((I1[i] * WuI2Jm + I1[i + 30] * s) + I1[i + 60] * normH) -
                  utarget[i];
      }
      b_kidx = -1;
      kidx = -1;
      for (i1 = 0; i1 < 10; i1++) {
        K[b_kidx + 1] = Wu[0];
        K[b_kidx + 2] = Wu[1];
        K[b_kidx + 3] = Wu[2];
        b_kidx += 3;
        for (Tries = 0; Tries < 6; Tries++) {
          b_Sx[(kidx + Tries) + 1] = Wy[Tries];
        }
        kidx += 6;
      }
      normH = 0.0;
      for (i = 0; i < 30; i++) {
        WuI2Jm = aux3[i];
        normH += WuI2Jm * (K[i] * WuI2Jm);
      }
      s = 0.0;
      for (i = 0; i < 60; i++) {
        WuI2Jm = aux2[i];
        s += WuI2Jm * (b_Sx[i] * WuI2Jm);
      }
      b_Kv = 0.0;
      for (i = 0; i < 31; i++) {
        WuI2Jm = 0.0;
        for (i1 = 0; i1 < 31; i1++) {
          WuI2Jm += H[i + 31 * i1] * aux[i1];
        }
        b_Kv += aux[i] * (WuI2Jm + 2.0 * varargin_1[i]);
      }
      *cost = (normH + s) + b_Kv;
    }
    normH = old_u[0] + uoff[0];
    s = old_u[1] + uoff[1];
    b_Kv = old_u[2] + uoff[2];
    for (i = 0; i < 30; i++) {
      WuI2Jm = 0.0;
      for (i1 = 0; i1 < 30; i1++) {
        WuI2Jm += I2Jm[i + 30 * i1] * aux[i1];
      }
      aux3[i] = WuI2Jm + ((I1[i] * normH + I1[i + 30] * s) + I1[i + 60] * b_Kv);
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 10; i1++) {
        useq[i1 + 11 * i] = aux3[i + 3 * i1];
      }
      useq[11 * i + 10] = useq[11 * i + 9];
    }
  }
}

} // namespace coder
} // namespace pos_MPC

// End of code generation (mpcblock_optimizer.cpp)
