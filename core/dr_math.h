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

 /** @brief Construct a 3D vector */
DR_INLINE dr_vec3f_t dr_v3( float x, float y, float z );

/** @brief Componentwise addition: a + b */
DR_INLINE dr_vec3f_t dr_v3_add( dr_vec3f_t a, dr_vec3f_t b );

/** @brief Componentwise subtraction: a - b */
DR_INLINE dr_vec3f_t dr_v3_sub( dr_vec3f_t a, dr_vec3f_t b );

/** @brief Scalar Multiply: s * a */
DR_INLINE dr_vec3f_t dr_v3_scale( float s, dr_vec3f_t a );

/** @brief Dot product: a . b */
DR_INLINE float dr_v3_dot( dr_vec3f_t a, dr_vec3f_t b );

/** @brief Cross product: a x b */
DR_INLINE dr_vec3f_t dr_v3_cross( dr_vec3f_t a, dr_vec3f_t b );

/** @brief Euclidean norm: ||a|| */
DR_INLINE float dr_v3_norm( dr_vec3f_t a );

/** @brief Unit Vector: a / ||a|| (returns {0,0,0}) if tiny */
DR_INLINE dr_vec3f_t dr_v3_normalize( dr_vec3f_t a );

/** @brief Identity Quaternion */
DR_INLINE dr_quatf_t dr_quat_identity( void );

/** @brief Normalize Quaternion: (guards zero norm) */
DR_INLINE dr_quatf_t dr_q_normalize( dr_quatf_t q );

/** @brief Hamilton Product: a * b */
DR_INLINE dr_quatf_t dr_q_mult( dr_quatf_t a, dr_quatf_t b );

/** @brief Quaternion from rotation vector (axis * angle[rad]) */
DR_INLINE dr_quatf_t dr_q_from_rotvec( dr_vec3f_t w );

/** @brief Rotate Vector by Quaternion (active, body->world) */
DR_INLINE dr_vec3f_t dr_q_rotate( dr_quatf_t q, dr_vec3f_t v );

/** @brief Integrate quaternion with body rate over dt (right multiply) */
DR_INLINE dr_quatf_t dr_q_integrate_gyro( dr_quatf_t q, dr_vec3f_t omega, float dt );

/** @brief Skew-symmetric 3x3 matrix [V]_x (row-major) */
DR_INLINE void dr_skew3( dr_vec3f_t v, float S[9] );

/** @brief Rotation Matrix R(q) from quaternion (body->world) */
void dr_R_from_q( dr_quatf_t q, float R[9] );

/** @brief Set Identity Matrix I(n) */
void dr_mat_set_identity( float *I, int n );

/** @brief  Matrix Add: C = A + B */
void dr_mat_add( float *C, float *A, float *B, int rows, int cols );

/** @brief  Matrix Subtract: C = A - B */
void dr_mat_sub( float *C, float *A, float *B, int rows, int cols );

/** @brief Matrix Multiply: C(mxp) = A(mxn) * B(nxp) */
void dr_mat_mul( float *C, float *A, float *B, int m, int n, int p );

/** @brief  Matrix Transpose: AT = A^T */
void dr_mat_transpose( float *AT, const float *A, int rows, int cols );

/** @brief Invert 3x3 Matrix @return 0 on success, -1 if singular */
int dr_mat3_inv(float inv[9], const float M[9]);


#ifdef __cplusplus
}
#endif
#endif // DR_MATH_H