/*
 * ik_7dof_ofst.h
 *
 * Code generation for function 'ik_7dof_ofst'
 *
 */

#ifndef IK_7DOF_OFST_H
#define IK_7DOF_OFST_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Function Declarations */
    // extern void ik_7dof_ofst(double z_alpha, double y_beta, double x_gamma,
    //                          double p_x, double p_y, double p_z, double bet,
    //                          const double cur_theta[7], double b_lt_or_rt,
    //                          double LOrR, double FOrB, double theta[7],
    //                          double *ik_state);
    void ik_7dof_ofst(double z_alpha, double y_beta, double x_gamma, double p_x,
                    double p_y, double p_z, double bet, const double cur_theta[7],
                    int b_lt_or_rt, int LOrR, int FOrB, double theta[7],
                    int *ik_state);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (ik_7dof_ofst.h) */
