/*
 * forward_kinematic_with_ofst.h
 *
 * Code generation for function 'forward_kinematic_with_ofst'
 *
 */

#ifndef FORWARD_KINEMATIC_WITH_OFST_H
#define FORWARD_KINEMATIC_WITH_OFST_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Function Declarations */
    extern void forward_kinematic_with_ofst(double theta[7], double a_arr[7],
                                            double alpha_arr[7],
                                            double d_arr[7],
                                            double theta_arr[7],
                                            double carte[3], double eulVal[3],
                                            double *bet, int *LOrR, int *FOrB);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (forward_kinematic_with_ofst.h) */
