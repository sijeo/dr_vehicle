#ifndef DR_CALC_H
#define DR_CALC_H

#include "dr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file dr_cal.h 
 * @brief Small helpers for applyinng per-axis bias/scale calibration.
 * 
*/

/** @brief Apply 3-axis affine calibration: (in-bias * scale) 
 * @param cal calibration biases and scales
 * @param in input vector [3]
 * @param out output vector [3]
*/
DR_INLINE void dr_apply_cal3( const dr_cal3_t* cal, const float in[3], float out[3] )
{
    out[0] = (in[0] - cal->bias[0]) * cal->scale[0];
    out[1] = (in[1] - cal->bias[1]) * cal->scale[1];
    out[2] = (in[2] - cal->bias[2]) * cal->scale[2];
}

#ifdef __cplusplus
}
#endif  
#endif // DR_CALC_H
