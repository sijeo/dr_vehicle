#ifndef DR_MATH_H
#define DR_MATH_H

#include <math.h>
#include "dr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file dr_math.h
 * @brief Lightweight vector, quaternion, rotation, and small-matrix helpers.
 */

/* Vector helpers */
dr_vec3f_t dr_v3(float x, float y, float z);
dr_vec3f_t dr_v3_add(dr_vec3f_t a, dr_vec3f_t b);
dr_vec3f_t dr_v3_sub(dr_vec3f_t a, dr_vec3f_t b);
dr_vec3f_t dr_v3_scale(float s, dr_vec3f_t a);
float      dr_v3_dot(dr_vec3f_t a, dr_vec3f_t b);
dr_vec3f_t dr_v3_cross(dr_vec3f_t a, dr_vec3f_t b);
float      dr_v3_norm(dr_vec3f_t a);
dr_vec3f_t dr_v3_normalize(dr_vec3f_t a);

/* Quaternion helpers */
dr_quatf_t dr_quat_identity(void);
dr_quatf_t dr_q_normalize(dr_quatf_t q);
dr_quatf_t dr_q_mult(dr_quatf_t a, dr_quatf_t b);
dr_quatf_t dr_q_from_rotvec(dr_vec3f_t w);
dr_vec3f_t dr_q_rotate(dr_quatf_t q, dr_vec3f_t v);
dr_quatf_t dr_q_integrate_gyro(dr_quatf_t q, dr_vec3f_t omega, float dt);

/* Matrix helpers */
void dr_skew3(dr_vec3f_t v, float S[9]);
void dr_R_from_q(dr_quatf_t q, float R[9]);

/* General matrix ops */
void dr_mat_set_identity(float *I, int n);
void dr_mat_add(float *C, float *A, float *B, int rows, int cols);
void dr_mat_sub(float *C, float *A, float *B, int rows, int cols);
void dr_mat_mul(float *C, float *A, float *B, int m, int n, int p);
void dr_mat_transpose(float *AT, const float *A, int rows, int cols);
int  dr_mat3_inv(float inv[9], const float M[9]);

#ifdef __cplusplus
}
#endif
#endif /* DR_MATH_H */