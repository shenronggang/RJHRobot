/*
 * ik_7dof_ofst.c
 *
 * Code generation for function 'ik_7dof_ofst'
 *
 */

/* Include files */
#include "ik_7dof_ofst.h"
#include "forward_kinematic_with_ofst_data.h"
#include "forward_kinematic_with_ofst_initialize.h"
#include "forward_kinematic_with_ofst_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void ik_7dof_ofst(double z_alpha, double y_beta, double x_gamma, double p_x,
                  double p_y, double p_z, double bet, const double cur_theta[7],
                  int lt_or_rt, int LOrR, int FOrB, double jntLim[14],
                  double theta[7], int *ik_state)
{
  static const short b[3] = {300, 0, 0};
  static const signed char b_b[16] = {1, 0, 0, 0, 0,  1, 0, 0,
                                      0, 0, 1, 0, 71, 0, 0, 1};
  static const signed char b_iv[4] = {0, 0, 1, 0};
  static const signed char b_iv1[4] = {0, 0, 0, 1};
  static const signed char iv2[4] = {0, 0, -1, 0};
  double T01_tmp[16];
  double T07[16];
  double T0P[16];
  double T12_tmp[16];
  double b_T0P[16];
  double R47[9];
  double b_cB[9];
  double n4[3];
  double R_end_tmp;
  double a;
  double absxk;
  double cB;
  double n_idx_0;
  double n_idx_1;
  double n_idx_2;
  double p_e_idx_0;
  double p_e_idx_1;
  double p_e_idx_2;
  double sB;
  double scale;
  double t;
  double theta1;
  double theta2;
  double theta4;
  double theta4_tmp;
  int b_i;
  int i;
  int i1;
  int initVal;
  boolean_T guard1;
  if (!isInitialized_forward_kinematic_with_ofst) {
    forward_kinematic_with_ofst_initialize();
  }
  /*  global mid_theta */
  /* ------------以下求解辅助臂----------------------------- */
  *ik_state = 0;
  /*  0-normal, 2001-2007 lower limit (left arm), 2008~2014 upper limit (left
   * arm) */
  /*  25-elbow singularity 26-shoulder singularity  27-wrist singularity
   * 28-rotation singularity */
  /*  29-elbow singularity in z-axis */
  /*  2015-2021 lower limit (right arm), 2022~2028 upper limit (right arm) */
  /*  外旋ZYX旋转 */
  theta4 = cos(y_beta);
  sB = sin(z_alpha);
  cB = cos(z_alpha);
  scale = sin(y_beta);
  R_end_tmp = cos(x_gamma);
  absxk = sin(x_gamma);
  T07[0] = theta4 * cB;
  T07[4] = -theta4 * sB;
  T07[8] = scale;
  t = cB * absxk;
  T07[1] = R_end_tmp * sB + t * scale;
  cB *= R_end_tmp;
  T07[5] = cB - absxk * scale * sB;
  T07[9] = -theta4 * absxk;
  T07[2] = absxk * sB - cB * scale;
  T07[6] = t + R_end_tmp * scale * sB;
  T07[10] = R_end_tmp * theta4;
  T07[12] = p_x;
  T07[13] = p_y;
  T07[14] = p_z;
  T07[15] = 1.0;
  /*  AA = rotm2Eul(R_end, "xyz"); */
  T07[3] = 0.0;
  T07[7] = 0.0;
  T07[11] = 0.0;
  /* 记得把这里的坐标反一下 */
  scale = 3.3121686421112381E-170;
  absxk = fabs(-p_x);
  if (absxk > 3.3121686421112381E-170) {
    sB = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    sB = t * t;
  }
  absxk = fabs(-p_y);
  if (absxk > scale) {
    t = scale / absxk;
    sB = sB * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    sB += t * t;
  }
  absxk = fabs(p_z);
  if (absxk > scale) {
    t = scale / absxk;
    sB = sB * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    sB += t * t;
  }
  sB = scale * sqrt(sB);
  guard1 = false;
  if (602.9925372672534 - sB > 1.0E-6) {
    /*  两边之和大于第三边，表示还没有伸直 */
    theta4_tmp = sB * sB;
    /* 明明我看着正角度是对的，怎么就变成了负角度 */
    /* 求得theta4后，就重定义一下机械臂 */
    a = sqrt((theta4_tmp + 90000.0) -
             600.0 * sB *
                 cos(acos(((theta4_tmp + 90900.0) - 90900.0) /
                          (2.0 * sB * 301.4962686336267)) +
                     0.099668652491161858));
    /*  DW */
    /* 与p向量组成零面位 */
    n_idx_0 = -p_y * -0.0 - -0.0 * p_z;
    cB = -p_x * -0.0;
    n_idx_1 = -p_z - cB;
    n_idx_2 = cB - p_y;
    /*  零位面法向量为n */
    scale = 3.3121686421112381E-170;
    t = n_idx_0 / 3.3121686421112381E-170;
    theta4 = t * t;
    absxk = fabs(n_idx_1);
    if (absxk > 3.3121686421112381E-170) {
      t = 3.3121686421112381E-170 / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta4 += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta4 += t * t;
    }
    theta4 = scale * sqrt(theta4);
    n_idx_0 /= theta4;
    p_e_idx_0 = -p_x / sB;
    n_idx_1 /= theta4;
    p_e_idx_1 = -p_y / sB;
    n_idx_2 /= theta4;
    p_e_idx_2 = p_z / sB;
    /*  归一化的p */
    bet += 3.1415926535897931;
    /*  绕p旋转bet角的轴角表示 */
    scale = cos(bet);
    R_end_tmp = sin(bet);
    R47[0] = p_e_idx_0 * p_e_idx_0 * (1.0 - scale) + scale;
    absxk = p_e_idx_0 * p_e_idx_1 * (1.0 - scale);
    t = p_e_idx_2 * R_end_tmp;
    R47[3] = absxk - t;
    theta4 = p_e_idx_0 * p_e_idx_2 * (1.0 - scale);
    cB = p_e_idx_1 * R_end_tmp;
    R47[6] = theta4 + cB;
    R47[1] = absxk + t;
    R47[4] = p_e_idx_1 * p_e_idx_1 * (1.0 - scale) + scale;
    absxk = p_e_idx_1 * p_e_idx_2 * (1.0 - scale);
    t = p_e_idx_0 * R_end_tmp;
    R47[7] = absxk - t;
    R47[2] = theta4 - cB;
    R47[5] = absxk + t;
    R47[8] = p_e_idx_2 * p_e_idx_2 * (1.0 - scale) + scale;
    for (i = 0; i < 3; i++) {
      n4[i] = (R47[i] * n_idx_0 + R47[i + 3] * n_idx_1) + R47[i + 6] * n_idx_2;
    }
    /* Xk */
    /* Zk */
    n_idx_0 = n4[1] * p_e_idx_2 - p_e_idx_1 * n4[2];
    n_idx_1 = p_e_idx_0 * n4[2] - n4[0] * p_e_idx_2;
    n_idx_2 = n4[0] * p_e_idx_1 - p_e_idx_0 * n4[1];
    /* 得到k轴方向的y在0中的描述 */
    scale = 3.3121686421112381E-170;
    absxk = fabs(n_idx_0);
    if (absxk > 3.3121686421112381E-170) {
      theta4 = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta4 = t * t;
    }
    absxk = fabs(n_idx_1);
    if (absxk > scale) {
      t = scale / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta4 += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta4 += t * t;
    }
    theta4 = scale * sqrt(theta4);
    /* 归一化，真正得到k轴的y在0中的描述 */
    cB = ((theta4_tmp + 90000.0) - a * a) / (600.0 * sB);
    /* d5=DW cB=cos(DBW) */
    sB = sqrt(1.0 - cB * cB);
    /*  k_R_k'  由k'变换到k的旋转矩阵 */
    b_cB[0] = cB;
    b_cB[3] = -sB;
    b_cB[6] = 0.0;
    b_cB[1] = sB;
    b_cB[4] = cB;
    b_cB[7] = 0.0;
    R47[0] = p_e_idx_0;
    R47[3] = n_idx_0 / theta4;
    R47[6] = n4[0];
    b_cB[2] = 0.0;
    R47[1] = p_e_idx_1;
    R47[4] = n_idx_1 / theta4;
    R47[7] = n4[1];
    b_cB[5] = 0.0;
    R47[2] = p_e_idx_2;
    R47[5] = n_idx_2 / theta4;
    R47[8] = n4[2];
    b_cB[8] = 1.0;
    for (i = 0; i < 3; i++) {
      p_e_idx_2 = 0.0;
      cB = R47[i];
      sB = R47[i + 3];
      a = R47[i + 6];
      for (b_i = 0; b_i < 3; b_i++) {
        p_e_idx_2 += ((cB * b_cB[3 * b_i] + sB * b_cB[3 * b_i + 1]) +
                      a * b_cB[3 * b_i + 2]) *
                     (double)b[b_i];
      }
      n4[i] = p_e_idx_2;
    }
    /*  D点坐标在0系下的表示, xD=pD0(1), yD=pD0(2), zD=pD0(3) */
    theta2 = acos(n4[2] / 300.0);
    /* 这里涉及到选哪个解 */
    /*  以上求得theta2  */
    /*  theta2 = -theta2; */
    R_end_tmp = sin(theta2);
    cB = 300.0 * R_end_tmp;
    if (fabs(cB) > 1.0E-6) {
      theta1 = rt_atan2d_snf(n4[1] / cB, n4[0] / cB);
    } else {
      theta1 = cur_theta[0];
      *ik_state = 26;
      /*  代表肩奇异 */
    }
    /*  DW */
    n4[0] = -p_x - n4[0];
    n4[1] = -p_y - n4[1];
    n4[2] = p_z - n4[2];
    n_idx_0 = -p_y * n4[2] - n4[1] * p_z;
    n_idx_1 = n4[0] * p_z - -p_x * n4[2];
    n_idx_2 = -p_x * n4[1] - n4[0] * -p_y;
    scale = 3.3121686421112381E-170;
    absxk = fabs(n_idx_0);
    if (absxk > 3.3121686421112381E-170) {
      theta4 = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta4 = t * t;
    }
    absxk = fabs(n_idx_1);
    if (absxk > scale) {
      t = scale / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta4 += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      theta4 = theta4 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta4 += t * t;
    }
    theta4 = scale * sqrt(theta4);
    n_idx_0 /= theta4;
    n_idx_1 /= theta4;
    n_idx_2 /= theta4;
    if (R_end_tmp <= 1.0E-6) {
      p_e_idx_0 = rt_atan2d_snf(-n_idx_0, n_idx_1) - theta1;
    } else if (R_end_tmp > 1.0E-6) {
      scale = cos(theta1);
      if (fabs(scale) < 1.0E-6) {
        p_e_idx_0 = rt_atan2d_snf(
            n_idx_2 / R_end_tmp,
            -(n_idx_0 + scale * cos(theta2) * n_idx_2 / R_end_tmp) /
                sin(theta1));
      } else {
        p_e_idx_0 = rt_atan2d_snf(
            n_idx_2 / R_end_tmp,
            (n_idx_1 + sin(theta1) * cos(theta2) * n_idx_2 / R_end_tmp) /
                cos(theta1));
      }
    } else {
      p_e_idx_0 = rt_atan2d_snf(
          n_idx_2 / R_end_tmp,
          (n_idx_1 + sin(theta1) * cos(theta2) * n_idx_2 / R_end_tmp) /
              cos(theta1));
    }
    /*  以上求得theta3 */
    p_e_idx_0 += 3.1415926535897931;
    /*  为啥theta3要翻180度？ */
    if (p_e_idx_0 > 3.1415926535897931) {
      p_e_idx_0 -= 6.2831853071795862;
    }
    theta2 = -theta2;
    theta4 = ((6.2831853071795862 - acos((181800.0 - theta4_tmp) / 181800.0)) -
              1.4711276743037347) -
             1.4711276743037347;
    R_end_tmp = sin(theta1);
    absxk = cos(theta1);
    scale = cos(theta2);
    n_idx_2 = sin(p_e_idx_0);
    t = cos(p_e_idx_0);
    p_e_idx_0 = cos(theta4);
    T01_tmp[0] = absxk;
    T01_tmp[4] = -R_end_tmp;
    T01_tmp[8] = 0.0;
    T01_tmp[12] = 0.0;
    T01_tmp[1] = R_end_tmp;
    T01_tmp[5] = absxk;
    T01_tmp[9] = 0.0;
    T01_tmp[13] = 0.0;
    T12_tmp[0] = scale;
    n_idx_0 = -sin(theta2);
    T12_tmp[4] = n_idx_0;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = n_idx_0;
    T12_tmp[6] = -scale;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T01_tmp[2] = 0.0;
    T01_tmp[3] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T01_tmp[6] = 0.0;
    T01_tmp[7] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T01_tmp[10] = 1.0;
    T01_tmp[11] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T01_tmp[14] = 0.0;
    T01_tmp[15] = 1.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T01_tmp[i];
      cB = T01_tmp[i + 4];
      sB = T01_tmp[i + 8];
      a = T01_tmp[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T0P[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                       sB * T12_tmp[i1 + 2]) +
                      a * T12_tmp[i1 + 3];
      }
    }
    T12_tmp[0] = t;
    T12_tmp[4] = -n_idx_2;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 30.0;
    T12_tmp[2] = n_idx_2;
    T12_tmp[6] = t;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = -300.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T0P[i];
      cB = T0P[i + 4];
      sB = T0P[i + 8];
      a = T0P[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T01_tmp[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                           sB * T12_tmp[i1 + 2]) +
                          a * T12_tmp[i1 + 3];
      }
    }
    T12_tmp[0] = p_e_idx_0;
    n_idx_0 = -sin(theta4);
    T12_tmp[4] = n_idx_0;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = n_idx_0;
    T12_tmp[6] = -p_e_idx_0;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    for (i = 0; i < 4; i++) {
      initVal = i << 2;
      b_i = b_iv[i];
      T12_tmp[initVal + 1] = b_i;
      i1 = b_iv1[i];
      T12_tmp[initVal + 3] = i1;
      p_e_idx_2 = T12_tmp[initVal];
      cB = T12_tmp[initVal + 2];
      for (initVal = 0; initVal < 4; initVal++) {
        b_T0P[i + (initVal << 2)] = ((T01_tmp[initVal] * p_e_idx_2 +
                                      T01_tmp[initVal + 4] * (double)b_i) +
                                     T01_tmp[initVal + 8] * cB) +
                                    T01_tmp[initVal + 12] * (double)i1;
      }
    }
    for (i = 0; i < 3; i++) {
      p_e_idx_2 = b_T0P[i];
      cB = b_T0P[i + 4];
      sB = b_T0P[i + 8];
      for (b_i = 0; b_i < 3; b_i++) {
        i1 = b_i << 2;
        R47[i + 3 * b_i] =
            (p_e_idx_2 * T07[i1] + cB * T07[i1 + 1]) + sB * T07[i1 + 2];
      }
    }
    p_e_idx_1 = acos(-R47[7]);
    /* 注意这里有选解问题，左右臂可能吧不同 */
    /*  以上求得theta6 */
    if (fabs(p_e_idx_1) > 1.0E-6) {
      cB = sin(p_e_idx_1);
      theta4 = rt_atan2d_snf(R47[8] / cB, R47[6] / cB);
      n_idx_1 = rt_atan2d_snf(-R47[4] / cB, R47[1] / cB);
    } else {
      theta4 = cur_theta[4];
      /*  真实机器人赋予上一时刻值 */
      n_idx_1 = rt_atan2d_snf(-R47[3], R47[5]) - cur_theta[4];
      *ik_state = 27;
      /*  代表腕奇异 */
    }
    /*  [theta1 theta2 theta3 theta4 theta5 theta6 theta7] */
    /*     %% 这边的关节6的 */
    /* -------------------从现在开始求解倒置臂------------------------ */
    absxk = sin(theta4);
    scale = cos(theta4);
    R_end_tmp = cos(p_e_idx_1);
    /*  6轴x轴方向朝上 */
    /*  T56 =[ -sin(theta6), -cos(theta6), 0, 0; */
    /*                    0,            0, 1, 0; */
    /*         -cos(theta6),  sin(theta6), 0, 0; */
    /*                    0,            0, 0, 1];  % 6轴x轴方向朝内 */
    /*  T56 =[ sin(theta6), cos(theta6), 0, 0; */
    /*                   0,            0, 1, 0; */
    /*         cos(theta6), -sin(theta6), 0, 0; */
    /*                   0,            0, 0, 1];  % 6轴x轴方向朝外 */
    /* 注意，以后右边是上标 */
    /*  p_0_6 = sqrt(T_0_6(1:3,4)'*T_0_6(1:3,4)); */
    /*     %% 到这里相当于pdf的z6轴向前平移了71mm */
    /* 末端相当于沿着6系x方向前移了71，6系x方向是朝里的 */
    T12_tmp[0] = t;
    T12_tmp[4] = -n_idx_2;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = n_idx_2;
    T12_tmp[6] = t;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = -300.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T0P[i];
      cB = T0P[i + 4];
      sB = T0P[i + 8];
      a = T0P[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T01_tmp[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                           sB * T12_tmp[i1 + 2]) +
                          a * T12_tmp[i1 + 3];
      }
    }
    T12_tmp[0] = p_e_idx_0;
    T12_tmp[4] = n_idx_0;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 30.0;
    T12_tmp[2] = n_idx_0;
    T12_tmp[6] = -p_e_idx_0;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T01_tmp[i];
      cB = T01_tmp[i + 4];
      sB = T01_tmp[i + 8];
      a = T01_tmp[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T0P[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                       sB * T12_tmp[i1 + 2]) +
                      a * T12_tmp[i1 + 3];
      }
    }
    T12_tmp[0] = scale;
    T12_tmp[4] = -absxk;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = -30.0;
    T12_tmp[2] = absxk;
    T12_tmp[6] = scale;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = -300.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T0P[i];
      cB = T0P[i + 4];
      sB = T0P[i + 8];
      a = T0P[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T01_tmp[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                           sB * T12_tmp[i1 + 2]) +
                          a * T12_tmp[i1 + 3];
      }
    }
    T12_tmp[0] = R_end_tmp;
    n_idx_0 = -sin(p_e_idx_1);
    T12_tmp[4] = n_idx_0;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = n_idx_0;
    T12_tmp[6] = -R_end_tmp;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      p_e_idx_2 = T01_tmp[i];
      cB = T01_tmp[i + 4];
      sB = T01_tmp[i + 8];
      a = T01_tmp[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T0P[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                       sB * T12_tmp[i1 + 2]) +
                      a * T12_tmp[i1 + 3];
      }
      p_e_idx_2 = T0P[i];
      cB = T0P[i + 4];
      sB = T0P[i + 8];
      a = T0P[i + 12];
      for (b_i = 0; b_i < 4; b_i++) {
        i1 = b_i << 2;
        T07[i + i1] =
            ((p_e_idx_2 * (double)b_b[i1] + cB * (double)b_b[i1 + 1]) +
             sB * (double)b_b[i1 + 2]) +
            a * (double)b_b[i1 + 3];
      }
    }
    /* 后撤得到P系 */
    scale = 3.3121686421112381E-170;
    absxk = fabs(T07[12]);
    if (absxk > 3.3121686421112381E-170) {
      cB = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      cB = t * t;
    }
    absxk = fabs(T07[13]);
    if (absxk > scale) {
      t = scale / absxk;
      cB = cB * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      cB += t * t;
    }
    absxk = fabs(T07[14]);
    if (absxk > scale) {
      t = scale / absxk;
      cB = cB * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      cB += t * t;
    }
    cB = scale * sqrt(cB);
    if (532.452974533239 - cB > 1.0E-6) {
      cB = acos((144241.0 - cB * cB) / 139265.17008929403);
      if (LOrR == 1) {
        p_e_idx_0 = -((4.84265153241579 - cB) - 1.4711276743037347);
      } else {
        p_e_idx_0 = 2.9116614490675312 - cB;
      }
      /* 以上求得 theta3 */
      for (i = 0; i < 16; i++) {
        T0P[i] = iv[i];
      }
      for (i = 0; i < 3; i++) {
        p_e_idx_2 = T07[i];
        R47[3 * i] = p_e_idx_2;
        initVal = i << 2;
        T0P[initVal] = p_e_idx_2;
        p_e_idx_2 = T07[i + 4];
        R47[3 * i + 1] = p_e_idx_2;
        T0P[initVal + 1] = p_e_idx_2;
        p_e_idx_2 = T07[i + 8];
        R47[3 * i + 2] = p_e_idx_2;
        T0P[initVal + 2] = p_e_idx_2;
      }
      for (i = 0; i < 9; i++) {
        R47[i] = -R47[i];
      }
      p_e_idx_2 = T07[12];
      cB = T07[13];
      sB = T07[14];
      for (i = 0; i < 3; i++) {
        T0P[i + 12] = (R47[i] * p_e_idx_2 + R47[i + 3] * cB) + R47[i + 6] * sB;
      }
      for (i = 0; i < 4; i++) {
        p_e_idx_2 = T0P[i];
        cB = T0P[i + 4];
        sB = T0P[i + 8];
        a = T0P[i + 12];
        for (b_i = 0; b_i < 4; b_i++) {
          i1 = b_i << 2;
          T07[i + i1] =
              ((p_e_idx_2 * (double)iv1[i1] + cB * (double)iv1[i1 + 1]) +
               sB * (double)iv1[i1 + 2]) +
              a * (double)iv1[i1 + 3];
        }
      }
      /*  plane = a*cos(theta3)-a+dd4*sin(theta3); */
      /*  theta3_ = [plane z theta3] */
      t = sin(p_e_idx_0);
      theta4 = cos(p_e_idx_0);
      cB = T07[14] / ((30.0 * theta4 - 30.0) + 300.0 * t);
      p_e_idx_2 = fabs(cB);
      if ((p_e_idx_2 >= -1.0) && (p_e_idx_2 <= 1.0)) {
        if (FOrB == 1) {
          theta2 = asin(cB);
          /*  -pi/2~pi/2 */
        } else {
          theta2 = 3.1415926535897931 - asin(cB);
          /*  pi/2~3pi/2 */
        }
        /*  以上求得 theta2 */
        scale = cos(theta2);
        theta1 = rt_atan2d_snf(T07[13], T07[12]) +
                 rt_atan2d_snf((30.0 * t - 300.0 * theta4) - 229.0,
                               scale * ((-300.0 * t + 30.0) - 30.0 * theta4));
        R_end_tmp = sin(theta1);
        absxk = cos(theta1);
        T01_tmp[0] = absxk;
        T01_tmp[4] = -R_end_tmp;
        T01_tmp[8] = 0.0;
        T01_tmp[12] = 0.0;
        T01_tmp[1] = R_end_tmp;
        T01_tmp[5] = absxk;
        T01_tmp[9] = 0.0;
        T01_tmp[13] = 0.0;
        T12_tmp[0] = scale;
        n_idx_0 = -sin(theta2);
        T12_tmp[4] = n_idx_0;
        T12_tmp[8] = 0.0;
        T12_tmp[12] = 0.0;
        T12_tmp[2] = n_idx_0;
        T12_tmp[6] = -scale;
        T12_tmp[10] = 0.0;
        T12_tmp[14] = 0.0;
        T01_tmp[2] = 0.0;
        T01_tmp[3] = 0.0;
        T12_tmp[1] = 0.0;
        T12_tmp[3] = 0.0;
        T01_tmp[6] = 0.0;
        T01_tmp[7] = 0.0;
        T12_tmp[5] = 0.0;
        T12_tmp[7] = 0.0;
        T01_tmp[10] = 1.0;
        T01_tmp[11] = 0.0;
        T12_tmp[9] = 1.0;
        T12_tmp[11] = 0.0;
        T01_tmp[14] = 0.0;
        T01_tmp[15] = 1.0;
        T12_tmp[13] = 229.0;
        T12_tmp[15] = 1.0;
        for (i = 0; i < 4; i++) {
          p_e_idx_2 = T01_tmp[i];
          cB = T01_tmp[i + 4];
          sB = T01_tmp[i + 8];
          a = T01_tmp[i + 12];
          for (b_i = 0; b_i < 4; b_i++) {
            i1 = b_i << 2;
            b_T0P[i + i1] = ((p_e_idx_2 * T12_tmp[i1] + cB * T12_tmp[i1 + 1]) +
                             sB * T12_tmp[i1 + 2]) +
                            a * T12_tmp[i1 + 3];
          }
        }
        T12_tmp[0] = theta4;
        T12_tmp[4] = -t;
        T12_tmp[8] = 0.0;
        T12_tmp[12] = 30.0;
        T12_tmp[2] = t;
        T12_tmp[6] = theta4;
        T12_tmp[10] = 0.0;
        T12_tmp[14] = 0.0;
        for (i = 0; i < 4; i++) {
          initVal = i << 2;
          b_i = iv2[i];
          T12_tmp[initVal + 1] = b_i;
          i1 = b_iv1[i];
          T12_tmp[initVal + 3] = i1;
          p_e_idx_2 = T12_tmp[initVal];
          cB = T12_tmp[initVal + 2];
          for (initVal = 0; initVal < 4; initVal++) {
            T01_tmp[i + (initVal << 2)] = ((b_T0P[initVal] * p_e_idx_2 +
                                            b_T0P[initVal + 4] * (double)b_i) +
                                           b_T0P[initVal + 8] * cB) +
                                          b_T0P[initVal + 12] * (double)i1;
          }
        }
        for (i = 0; i < 3; i++) {
          p_e_idx_2 = T01_tmp[i];
          cB = T01_tmp[i + 4];
          sB = T01_tmp[i + 8];
          for (b_i = 0; b_i < 3; b_i++) {
            i1 = b_i << 2;
            R47[i + 3 * b_i] =
                (p_e_idx_2 * T07[i1] + cB * T07[i1 + 1]) + sB * T07[i1 + 2];
          }
        }
        /* 以上求得 theta5 */
        scale = acos(-R47[7]);
        if (fabs(scale - 3.1415926535897931) < 1.0E-6) {
          theta4 = cur_theta[2];
          /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
          p_e_idx_1 = rt_atan2d_snf(-R47[3], -R47[5]) - cur_theta[2];
        } else {
          theta4_tmp = sin(scale - 3.1415926535897931);
          theta4 = rt_atan2d_snf(R47[8] / theta4_tmp, R47[6] / theta4_tmp);
          p_e_idx_1 = rt_atan2d_snf(-R47[4] / theta4_tmp, R47[1] / theta4_tmp);
        }
        theta[0] = p_e_idx_1 - 3.1415926535897931;
        theta[1] = scale - 3.1415926535897931;
        theta[2] = -theta4;
        theta[3] = -p_e_idx_0;
        theta[4] = theta2;
        theta[5] = -theta1;
        theta[6] = n_idx_1;
        /*  倒置臂if 的 end */
        guard1 = true;
      } else {
        *ik_state = 29;
        /*  这边是6Dof逆解z方向上的肘部奇异 */
        for (i = 0; i < 7; i++) {
          theta[i] = cur_theta[i];
        }
      }
    } else {
      /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
      *ik_state = 28;
      /*  这边是6Dof逆解时肘部奇异 */
      /* 肘奇异 */
      for (i = 0; i < 7; i++) {
        theta[i] = cur_theta[i];
      }
      /*  倒置臂if 的 end */
      guard1 = true;
    }
  } else {
    *ik_state = 25;
    /*  代表肘关节伸直，遇到肘奇异 */
    for (i = 0; i < 7; i++) {
      theta[i] = cur_theta[i];
    }
    guard1 = true;
  }
  if (guard1) {
    /*  大if分支的end */
    if (lt_or_rt == 0) {
      /*  0-left arm   1-right arm */
      initVal = 2008;
    } else {
      initVal = 2022;
    }
    for (b_i = 0; b_i < 7; b_i++) {
      i = b_i << 1;
      p_e_idx_2 = theta[b_i];
      if (p_e_idx_2 < jntLim[i]) {
        *ik_state = (initVal + b_i) - 7;
      } else if (p_e_idx_2 > jntLim[i + 1]) {
        *ik_state = initVal + b_i;
      }
    }
  }
}

/* End of code generation (ik_7dof_ofst.c) */
