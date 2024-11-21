/*
 * forward_kinematic_with_ofst.c
 *
 * Code generation for function 'forward_kinematic_with_ofst'
 *
 */

/* Include files */
#include "forward_kinematic_with_ofst.h"
#include "forward_kinematic_with_ofst_data.h"
#include "forward_kinematic_with_ofst_initialize.h"
#include "forward_kinematic_with_ofst_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void forward_kinematic_with_ofst(double theta[7], double a_arr[7],
                                 double alpha_arr[7],
                                 double d_arr[7],
                                 double theta_arr[7], double carte[3],
                                 double eulVal[3], double *bet, int *LOrR,
                                 int *FOrB)
{
  static const double alpha_arr_woOfst[7] = {0.0,
                                             -1.5707963267948966,
                                             1.5707963267948966,
                                             -1.5707963267948966,
                                             1.5707963267948966,
                                             -1.5707963267948966,
                                             1.5707963267948966};
  static const double dv[7] = {0.0,
                               -1.5707963267948966,
                               1.5707963267948966,
                               -1.5707963267948966,
                               1.5707963267948966,
                               -1.5707963267948966,
                               1.5707963267948966};
  static const short b_d_arr[7] = {0, 0, 300, 0, 300, 0, 0};
  static const short iv3[7] = {0, 0, 300, 0, 229, 0, 0};
  static const signed char b_iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char b_a_arr[7] = {0, 0, 0, 30, -30, 0, 0};
  static const signed char iv2[7] = {0, 0, 0, 30, -30, 0, -71};
  static const signed char iv4[4] = {0, 0, -1, 0};
  static const signed char iv5[4] = {0, 0, 0, 1};
  static const signed char b_iv1[3] = {-71, 0, 0};
  double T01_tmp[16];
  double TP6_inv[16];
  double T_0_6[16];
  double forwardMat[16];
  double R63[9];
  double Tx_alpha_tmp;
  double absxk;
  double carte_idx_0;
  double carte_idx_1;
  double carte_idx_2;
  double fir_res;
  double p_idx_0;
  double p_idx_1;
  double scale;
  double t;
  double theta1;
  double theta2;
  double theta3;
  double y_beta;
  int TP6_inv_tmp;
  int b_i;
  int i;
  int i1;
  int i2;
  signed char forwardMat_tmp[16];
  if (!isInitialized_forward_kinematic_with_ofst)
  {
    forward_kinematic_with_ofst_initialize();
  }
  *FOrB = -1;
  *LOrR = !(theta[3] < 0.0);
  if ((theta[4] >= -1.5707963267948966) && (theta[4] <= 1.5707963267948966))
  {
    *FOrB = 1;
  }
  else if (((theta[4] > 1.5707963267948966) &&
            (theta[4] <= 3.1415926535897931)) ||
           ((theta[4] >= -3.1415926535897931) &&
            (theta[4] < -1.5707963267948966)))
  {
    *FOrB = 0;
  }
  for (i = 0; i < 16; i++)
  {
    forwardMat_tmp[i] = 0;
  }
  forwardMat_tmp[0] = 1;
  forwardMat_tmp[5] = 1;
  forwardMat_tmp[10] = 1;
  forwardMat_tmp[15] = 1;
  for (i = 0; i < 16; i++)
  {
    forwardMat[i] = forwardMat_tmp[i];
  }
  for (b_i = 0; b_i < 7; b_i++)
  {
    fir_res = alpha_arr[b_i];
    Tx_alpha_tmp = sin(fir_res);
    scale = cos(fir_res);
    y_beta = theta_arr[b_i] + theta[b_i];
    absxk = sin(y_beta);
    y_beta = cos(y_beta);
    TP6_inv[1] = 0.0;
    TP6_inv[5] = scale;
    TP6_inv[9] = -Tx_alpha_tmp;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[6] = Tx_alpha_tmp;
    TP6_inv[10] = scale;
    TP6_inv[14] = 0.0;
    TP6_inv[0] = 1.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = 1.0;
    TP6_inv[4] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = a_arr[b_i];
    TP6_inv[1] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = y_beta;
    TP6_inv[4] = -absxk;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[1] = absxk;
    TP6_inv[5] = y_beta;
    TP6_inv[9] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[2] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[14] = d_arr[b_i];
    TP6_inv[0] = 1.0;
    TP6_inv[1] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
  }
  carte_idx_0 = forwardMat[12];
  carte[0] = forwardMat[12];
  carte_idx_1 = forwardMat[13];
  carte[1] = forwardMat[13];
  carte_idx_2 = forwardMat[14];
  carte[2] = forwardMat[14];
  /* z->y->x, 外旋 R = X(α)*Y(β)*Z(γ)  Tait-Bryan angles */
  /*  if ((abs(rotm(3,3)) < EPS && abs(rotm(2,3)) > EPS) || (abs(rotm(3,3)) <
   * EPS && abs(rotm(2,3)) < EPS)) */
  /*      euler(2) = asin(rotm(1,3)); */
  /*      sin_eul3 = -rotm(2,3)/cos(euler(2)); */
  /*      cos_eul3 = rotm(3,3)/cos(euler(2)); */
  /*      euler(3) = atan2(sin_eul3, cos_eul3); */
  /*  else */
  /*      euler(3) = atan(-rotm(2,3)/rotm(3,3)); % X(α) */
  /*      sin_beta = asin(rotm(1,3)); */
  /*      cos_beta = rotm(3,3)/cos(euler(3)); */
  /*   */
  /*      if (sin_beta >= EPS && cos_beta >= EPS)  % 0~90 */
  /*          euler(2) = atan(sin_beta/cos_beta); */
  /*      elseif (sin_beta >= EPS && cos_beta < -EPS) % 90~180 */
  /*          euler(2) = atan(sin_beta/cos_beta)+pi; */
  /*      elseif (sin_beta < -EPS && cos_beta >= EPS) % -90~0 */
  /*          euler(2) = atan(sin_beta/cos_beta); % -180~-90 */
  /*      elseif (sin_beta < -EPS && cos_beta < -EPS) */
  /*          euler(2) = atan(sin_beta/cos_beta)-pi; */
  /*      else */
  /*          euler(2) = asin(rotm(1,3)); */
  /*      end */
  /*  end */
  /*  %  */
  /*  if (abs(rotm(1,1)) < EPS) */
  /*      rotm(1,1) = 0; */
  /*  end */
  /*  if (abs(cos(euler(2))) > EPS) */
  /*      euler(1) = atan2(-rotm(1,2)/cos(euler(2)), rotm(1,1)/cos(euler(2))); %
   * Z(γ) */
  /*  else */
  /*      euler(1) = asin(rotm(2,1)); */
  /*  end */
  y_beta = rt_atan2d_snf(forwardMat[8], sqrt(forwardMat[0] * forwardMat[0] +
                                             forwardMat[4] * forwardMat[4]));
  if (fabs(y_beta - 1.5707963267948966) < 1.0E-6)
  {
    eulVal[0] = 0.0;
    eulVal[2] = rt_atan2d_snf(forwardMat[1], forwardMat[5]);
  }
  else if (fabs(y_beta - -1.5707963267948966) < 1.0E-6)
  {
    eulVal[0] = 0.0;
    eulVal[2] = rt_atan2d_snf(forwardMat[1], forwardMat[5]);
  }
  else
  {
    Tx_alpha_tmp = cos(y_beta);
    eulVal[2] = rt_atan2d_snf(-forwardMat[4] / Tx_alpha_tmp,
                              forwardMat[0] / Tx_alpha_tmp);
    eulVal[0] = rt_atan2d_snf(-forwardMat[9] / Tx_alpha_tmp,
                              forwardMat[10] / Tx_alpha_tmp);
  }
  eulVal[1] = y_beta;
  /*  计算臂型角 */
  for (i = 0; i < 16; i++)
  {
    forwardMat[i] = forwardMat_tmp[i];
  }
  for (b_i = 0; b_i < 6; b_i++)
  {
    fir_res = dv[b_i];
    Tx_alpha_tmp = sin(fir_res);
    scale = cos(fir_res);
    fir_res = theta[b_i];
    absxk = sin(fir_res);
    y_beta = cos(fir_res);
    TP6_inv[1] = 0.0;
    TP6_inv[5] = scale;
    TP6_inv[9] = -Tx_alpha_tmp;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[6] = Tx_alpha_tmp;
    TP6_inv[10] = scale;
    TP6_inv[14] = 0.0;
    TP6_inv[0] = 1.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = 1.0;
    TP6_inv[4] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = iv2[b_i];
    TP6_inv[1] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = y_beta;
    TP6_inv[4] = -absxk;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[1] = absxk;
    TP6_inv[5] = y_beta;
    TP6_inv[9] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[2] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[14] = iv3[b_i];
    TP6_inv[0] = 1.0;
    TP6_inv[1] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
  }
  /*     %% 解臂型角的思路，将7Dof带ofset的构型，重新逆解回不带ofset的构型 */
  /* 末端相当于沿着6系x方向前移了71，6系x方向是朝里的 */
  for (i = 0; i < 16; i++)
  {
    TP6_inv[i] = iv[i];
  }
  for (i = 0; i < 3; i++)
  {
    TP6_inv_tmp = i << 2;
    TP6_inv[TP6_inv_tmp] = b_iv[3 * i];
    TP6_inv[TP6_inv_tmp + 1] = b_iv[3 * i + 1];
    TP6_inv[TP6_inv_tmp + 2] = b_iv[3 * i + 2];
    TP6_inv[i + 12] = b_iv1[i];
  }
  for (i = 0; i < 4; i++)
  {
    fir_res = forwardMat[i];
    Tx_alpha_tmp = forwardMat[i + 4];
    scale = forwardMat[i + 8];
    t = forwardMat[i + 12];
    for (i1 = 0; i1 < 4; i1++)
    {
      i2 = i1 << 2;
      T_0_6[i + i2] =
          ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
           scale * TP6_inv[i2 + 2]) +
          t * TP6_inv[i2 + 3];
    }
  }
  /* P系(基于臂型角后撤的坐标系)转化为原始的W系 */
  scale = 3.3121686421112381E-170;
  absxk = fabs(T_0_6[12]);
  if (absxk > 3.3121686421112381E-170)
  {
    y_beta = 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / 3.3121686421112381E-170;
    y_beta = t * t;
  }
  absxk = fabs(T_0_6[13]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y_beta = y_beta * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y_beta += t * t;
  }
  absxk = fabs(T_0_6[14]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y_beta = y_beta * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y_beta += t * t;
  }
  y_beta = scale * sqrt(y_beta);
  if (602.9925372672534 - y_beta > 1.0E-6)
  {
    theta3 =
        -((4.8120576328758515 - acos((181800.0 - y_beta * y_beta) / 181800.0)) -
          1.4711276743037347);
  }
  else
  {
    theta3 = theta[3];
    /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
    /*  这边是6Dof逆解时肘部奇异 */
  }
  /* 以上求得 theta3 */
  for (i = 0; i < 16; i++)
  {
    TP6_inv[i] = iv[i];
  }
  for (i = 0; i < 3; i++)
  {
    fir_res = T_0_6[i];
    R63[3 * i] = fir_res;
    TP6_inv_tmp = i << 2;
    TP6_inv[TP6_inv_tmp] = fir_res;
    fir_res = T_0_6[i + 4];
    R63[3 * i + 1] = fir_res;
    TP6_inv[TP6_inv_tmp + 1] = fir_res;
    fir_res = T_0_6[i + 8];
    R63[3 * i + 2] = fir_res;
    TP6_inv[TP6_inv_tmp + 2] = fir_res;
  }
  for (i = 0; i < 9; i++)
  {
    R63[i] = -R63[i];
  }
  fir_res = T_0_6[12];
  Tx_alpha_tmp = T_0_6[13];
  scale = T_0_6[14];
  for (i = 0; i < 3; i++)
  {
    TP6_inv[i + 12] =
        (R63[i] * fir_res + R63[i + 3] * Tx_alpha_tmp) + R63[i + 6] * scale;
  }
  for (i = 0; i < 4; i++)
  {
    fir_res = TP6_inv[i];
    Tx_alpha_tmp = TP6_inv[i + 4];
    scale = TP6_inv[i + 8];
    t = TP6_inv[i + 12];
    for (i1 = 0; i1 < 4; i1++)
    {
      i2 = i1 << 2;
      forwardMat[i + i2] =
          ((fir_res * (double)iv1[i2] + Tx_alpha_tmp * (double)iv1[i2 + 1]) +
           scale * (double)iv1[i2 + 2]) +
          t * (double)iv1[i2 + 3];
    }
  }
  absxk = sin(theta3);
  p_idx_1 = cos(theta3);
  y_beta = forwardMat[14] / ((30.0 * p_idx_1 - 30.0) + 300.0 * absxk);
  fir_res = fabs(y_beta);
  if ((fir_res >= -1.0) && (fir_res <= 1.0))
  {
    theta2 = asin(y_beta);
    /* 这里有2组解 */
    /*  theta2 = -pi-theta2; */
  }
  else
  {
    /*  这边是6Dof逆解z方向上的肘部奇异 */
    theta2 = theta[4];
    /*  error('with singularity, no solution'); */
  }
  /*  以上求得 theta2 */
  scale = cos(theta2);
  theta1 = rt_atan2d_snf(forwardMat[13], forwardMat[12]) +
           rt_atan2d_snf((30.0 * absxk - 300.0 * p_idx_1) - 300.0,
                         scale * ((-300.0 * absxk + 30.0) - 30.0 * p_idx_1));
  y_beta = sin(theta1);
  Tx_alpha_tmp = cos(theta1);
  T_0_6[0] = Tx_alpha_tmp;
  T_0_6[4] = -y_beta;
  T_0_6[8] = 0.0;
  T_0_6[12] = 0.0;
  T_0_6[1] = y_beta;
  T_0_6[5] = Tx_alpha_tmp;
  T_0_6[9] = 0.0;
  T_0_6[13] = 0.0;
  TP6_inv[0] = scale;
  y_beta = -sin(theta2);
  TP6_inv[4] = y_beta;
  TP6_inv[8] = 0.0;
  TP6_inv[12] = 0.0;
  TP6_inv[2] = y_beta;
  TP6_inv[6] = -scale;
  TP6_inv[10] = 0.0;
  TP6_inv[14] = 0.0;
  T_0_6[2] = 0.0;
  T_0_6[3] = 0.0;
  TP6_inv[1] = 0.0;
  TP6_inv[3] = 0.0;
  T_0_6[6] = 0.0;
  T_0_6[7] = 0.0;
  TP6_inv[5] = 0.0;
  TP6_inv[7] = 0.0;
  T_0_6[10] = 1.0;
  T_0_6[11] = 0.0;
  TP6_inv[9] = 1.0;
  TP6_inv[11] = 0.0;
  T_0_6[14] = 0.0;
  T_0_6[15] = 1.0;
  TP6_inv[13] = 300.0;
  TP6_inv[15] = 1.0;
  for (i = 0; i < 4; i++)
  {
    fir_res = T_0_6[i];
    Tx_alpha_tmp = T_0_6[i + 4];
    scale = T_0_6[i + 8];
    t = T_0_6[i + 12];
    for (i1 = 0; i1 < 4; i1++)
    {
      i2 = i1 << 2;
      T01_tmp[i + i2] =
          ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
           scale * TP6_inv[i2 + 2]) +
          t * TP6_inv[i2 + 3];
    }
  }
  TP6_inv[0] = p_idx_1;
  TP6_inv[4] = -absxk;
  TP6_inv[8] = 0.0;
  TP6_inv[12] = 30.0;
  TP6_inv[2] = absxk;
  TP6_inv[6] = p_idx_1;
  TP6_inv[10] = 0.0;
  TP6_inv[14] = 0.0;
  for (i = 0; i < 4; i++)
  {
    TP6_inv_tmp = i << 2;
    i1 = iv4[i];
    TP6_inv[TP6_inv_tmp + 1] = i1;
    i2 = iv5[i];
    TP6_inv[TP6_inv_tmp + 3] = i2;
    fir_res = TP6_inv[TP6_inv_tmp];
    Tx_alpha_tmp = TP6_inv[TP6_inv_tmp + 2];
    for (TP6_inv_tmp = 0; TP6_inv_tmp < 4; TP6_inv_tmp++)
    {
      T_0_6[i + (TP6_inv_tmp << 2)] =
          ((T01_tmp[TP6_inv_tmp] * fir_res +
            T01_tmp[TP6_inv_tmp + 4] * (double)i1) +
           T01_tmp[TP6_inv_tmp + 8] * Tx_alpha_tmp) +
          T01_tmp[TP6_inv_tmp + 12] * (double)i2;
    }
  }
  for (i = 0; i < 3; i++)
  {
    fir_res = T_0_6[i];
    Tx_alpha_tmp = T_0_6[i + 4];
    scale = T_0_6[i + 8];
    for (i1 = 0; i1 < 3; i1++)
    {
      i2 = i1 << 2;
      R63[i + 3 * i1] =
          (fir_res * forwardMat[i2] + Tx_alpha_tmp * forwardMat[i2 + 1]) +
          scale * forwardMat[i2 + 2];
    }
  }
  /* 以上求得 theta5 */
  scale = acos(-R63[7]);
  if (fabs(scale - 3.1415926535897931) < 1.0E-6)
  {
    absxk = theta[2];
    /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
    y_beta = rt_atan2d_snf(-R63[3], -R63[5]) - theta[2];
  }
  else
  {
    y_beta = sin(scale - 3.1415926535897931);
    absxk = rt_atan2d_snf(R63[8] / y_beta, R63[6] / y_beta);
    y_beta = rt_atan2d_snf(-R63[4] / y_beta, R63[1] / y_beta);
  }
  /* 以上求得 theta4,theta6 */
  absxk = 3.1415926535897931 - absxk;
  Tx_alpha_tmp = theta[6];
  theta[0] = y_beta;
  theta[4] = theta2;
  theta[5] = -theta1;
  theta[6] = Tx_alpha_tmp;
  /*   */
  theta[1] = scale - 3.1415926535897931;
  theta[2] = absxk - 3.1415926535897931;
  theta[3] = theta3;
  for (i = 0; i < 16; i++)
  {
    forwardMat[i] = forwardMat_tmp[i];
  }
  memset(&R63[0], 0, 9U * sizeof(double));
  for (b_i = 0; b_i < 6; b_i++)
  {
    fir_res = alpha_arr_woOfst[b_i];
    Tx_alpha_tmp = sin(fir_res);
    scale = cos(fir_res);
    fir_res = theta[b_i];
    absxk = sin(fir_res);
    y_beta = cos(fir_res);
    TP6_inv[1] = 0.0;
    TP6_inv[5] = scale;
    TP6_inv[9] = -Tx_alpha_tmp;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[6] = Tx_alpha_tmp;
    TP6_inv[10] = scale;
    TP6_inv[14] = 0.0;
    TP6_inv[0] = 1.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = 1.0;
    TP6_inv[4] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = b_a_arr[b_i];
    TP6_inv[1] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[0] = y_beta;
    TP6_inv[4] = -absxk;
    TP6_inv[8] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[1] = absxk;
    TP6_inv[5] = y_beta;
    TP6_inv[9] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[2] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[7] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[11] = 0.0;
    TP6_inv[14] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = forwardMat[i];
      Tx_alpha_tmp = forwardMat[i + 4];
      scale = forwardMat[i + 8];
      t = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        T_0_6[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    TP6_inv[2] = 0.0;
    TP6_inv[6] = 0.0;
    TP6_inv[10] = 1.0;
    TP6_inv[14] = b_d_arr[b_i];
    TP6_inv[0] = 1.0;
    TP6_inv[1] = 0.0;
    TP6_inv[3] = 0.0;
    TP6_inv[4] = 0.0;
    TP6_inv[5] = 1.0;
    TP6_inv[7] = 0.0;
    TP6_inv[8] = 0.0;
    TP6_inv[9] = 0.0;
    TP6_inv[11] = 0.0;
    TP6_inv[12] = 0.0;
    TP6_inv[13] = 0.0;
    TP6_inv[15] = 1.0;
    for (i = 0; i < 4; i++)
    {
      fir_res = T_0_6[i];
      Tx_alpha_tmp = T_0_6[i + 4];
      scale = T_0_6[i + 8];
      t = T_0_6[i + 12];
      for (i1 = 0; i1 < 4; i1++)
      {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((fir_res * TP6_inv[i2] + Tx_alpha_tmp * TP6_inv[i2 + 1]) +
             scale * TP6_inv[i2 + 2]) +
            t * TP6_inv[i2 + 3];
      }
    }
    if (b_i + 1 == 4)
    {
      for (i = 0; i < 3; i++)
      {
        TP6_inv_tmp = i << 2;
        R63[3 * i] = forwardMat[TP6_inv_tmp];
        R63[3 * i + 1] = forwardMat[TP6_inv_tmp + 1];
        R63[3 * i + 2] = forwardMat[TP6_inv_tmp + 2];
      }
    }
  }
  theta3 = carte_idx_2;
  p_idx_0 = -carte_idx_0;
  p_idx_1 = -carte_idx_1;
  carte_idx_0 = -carte_idx_1 * -0.0 - carte_idx_2 * -0.0;
  carte_idx_1 = -carte_idx_2 - p_idx_0 * -0.0;
  carte_idx_2 = p_idx_0 * -0.0 - (-p_idx_1);
  /*  零位面法向量为n */
  scale = 3.3121686421112381E-170;
  t = carte_idx_0 / 3.3121686421112381E-170;
  y_beta = t * t;
  absxk = fabs(carte_idx_1);
  if (absxk > 3.3121686421112381E-170)
  {
    t = 3.3121686421112381E-170 / absxk;
    y_beta = y_beta * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / 3.3121686421112381E-170;
    y_beta += t * t;
  }
  absxk = fabs(carte_idx_2);
  if (absxk > scale)
  {
    t = scale / absxk;
    y_beta = y_beta * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y_beta += t * t;
  }
  y_beta = scale * sqrt(y_beta);
  scale = 3.3121686421112381E-170;
  carte_idx_0 /= y_beta;
  absxk = fabs(p_idx_0);
  if (absxk > 3.3121686421112381E-170)
  {
    Tx_alpha_tmp = 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / 3.3121686421112381E-170;
    Tx_alpha_tmp = t * t;
  }
  carte_idx_1 /= y_beta;
  absxk = fabs(p_idx_1);
  if (absxk > scale)
  {
    t = scale / absxk;
    Tx_alpha_tmp = Tx_alpha_tmp * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    Tx_alpha_tmp += t * t;
  }
  carte_idx_2 /= y_beta;
  absxk = fabs(theta3);
  if (absxk > scale)
  {
    t = scale / absxk;
    Tx_alpha_tmp = Tx_alpha_tmp * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    Tx_alpha_tmp += t * t;
  }
  Tx_alpha_tmp = scale * sqrt(Tx_alpha_tmp);
  p_idx_0 /= Tx_alpha_tmp;
  p_idx_1 /= Tx_alpha_tmp;
  theta3 /= Tx_alpha_tmp;
  /*  归一化的p */
  /* 接下来通过旋转轴p_e、旋转前向量n，旋转后向量n4求取轴角bet */
  y_beta = p_idx_0 * p_idx_0 * carte_idx_0;
  absxk = p_idx_0 * p_idx_1;
  Tx_alpha_tmp = absxk * carte_idx_1;
  scale = p_idx_0 * theta3 * carte_idx_2;
  theta2 = ((carte_idx_0 - y_beta) - Tx_alpha_tmp) - scale;
  theta1 = p_idx_1 * carte_idx_2 - carte_idx_1 * theta3;
  fir_res = ((R63[6] - y_beta) - Tx_alpha_tmp) - scale;
  scale = p_idx_1 * p_idx_1 * carte_idx_1;
  Tx_alpha_tmp = absxk * carte_idx_0;
  y_beta = p_idx_1 * theta3 * carte_idx_2;
  t = ((carte_idx_1 - scale) - Tx_alpha_tmp) - y_beta;
  absxk = carte_idx_0 * theta3 - p_idx_0 * carte_idx_2;
  y_beta = ((R63[7] - scale) - Tx_alpha_tmp) - y_beta;
  Tx_alpha_tmp = absxk * theta2;
  scale = theta1 * t;
  *bet = rt_atan2d_snf((fir_res * t - y_beta * theta2) / (scale - Tx_alpha_tmp),
                       (fir_res * absxk - y_beta * theta1) /
                           (Tx_alpha_tmp - scale));
}

/* End of code generation (forward_kinematic_with_ofst.c) */
