/*
 * forward_kinematic_with_ofst_initialize.c
 *
 * Code generation for function 'forward_kinematic_with_ofst_initialize'
 *
 */

/* Include files */
#include "forward_kinematic_with_ofst_initialize.h"
#include "forward_kinematic_with_ofst_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void forward_kinematic_with_ofst_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_forward_kinematic_with_ofst = true;
}

/* End of code generation (forward_kinematic_with_ofst_initialize.c) */
