#include "dr_math.h"

/** @brief Construct 3D vector 
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @return 3D vector dr_vec3f_t {x, y, z}
 */
DR_INLINE dr_vec3f_t dr_v3(float x, float y, float z)
{
    dr_vec3f_t v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

/** @brief Add two vectors component-wise 
 * @param a first vector
 * @param b second vector
 * @return a + b
 */
DR_INLINE dr_vec3f_t dr_v3_add(dr_vec3f_t a, dr_vec3f_t b)
{
    return dr_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/** @brief Subtract two vectors component-wise 
 * @param a first vector
 * @param b second vector
 * @return a - b
 */
DR_INLINE dr_vec3f_t dr_v3_sub(dr_vec3f_t a, dr_vec3f_t b)
{
    return dr_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/** @brief Multiply vector by scalar 
 * @param a vector
 * @param s scalar
 * @return a * s
 */
DR_INLINE dr_vec3f_t dr_v3_scale( float s, dr_vec3f_t a )
{
    return dr_v3(a.x * s, a.y * s, a.z * s);
}

/** @brief Dot product of two vectors
 * @param a first vector
 * @param b second vector
 * @return a . b (scalar)
*/
DR_INLINE float dr_v3_dot( dr_vec3f_t a, dr_vec3f_t b )
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** @brief Cross product of two vectors
 * @param a first vector
 * @param b second vector
 * @return a x b (vector)
 */
DR_INLINE dr_vec3f_t dr_v3_cross( dr_vec3f_t a, dr_vec3f_t b )
{
    return dr_v3(a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x);
}

/** @brief Euclidean norm (length of a vector )
 * @param a input vector
 * @return ||a|| (scalar)
 */
DR_INLINE float dr_v3_norm( dr_vec3f_t a )
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

/** @brief Normalize a vector; returns zero vector if near zero
 * @param a input vector
 * @return a / ||a|| (unit vector) ( or {0,0,0} if ||a|| is tiny )
 */
DR_INLINE dr_vec3f_t dr_v3_normalize( dr_vec3f_t a )
{
    float n = dr_v3_norm(a);
    if (n > 1e-9f) {
        return dr_v3_scale(1.0f / n, a);
    } else {
        return dr_v3(0.0f, 0.0f, 0.0f);
    }
}


/** @brief Identity Quaternion 
 * @return quaternion {1,0,0,0} (w,x,y,z)
*/
DR_INLINE dr_quatf_t dr_quat_identity( void )
{
    dr_quatf_t q;
    q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    return q;
}

/** @brief Normalize Quaternion: (guards zero norm)
 * @param q input quaternion
 * @return normalized quaternion (or identity if zero norm)
 */
DR_INLINE dr_quatf_t dr_q_normalize( dr_quatf_t q )
{
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    float invn;
    dr_quatf_t r;
    if (n > 1e-12f) {
        invn = 1.0f / n;
        r.w = q.w * invn; r.x = q.x * invn; r.y = q.y * invn; r.z = q.z * invn;
        return r;
    } else {
        return dr_quat_identity();
    }
}

/** @brief Hamilton Product (composition) of quaternions.
 * @param a left quaternion
 * @param b right quaternion
 * @return a * b (quaternion)
 */
DR_INLINE dr_quatf_t dr_q_mult( dr_quatf_t a, dr_quatf_t b )
{
    dr_quatf_t q;
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return q;
}

/** @brief Quaternion from rotation vector (axis * angle[rad])
 *         Uses first order approximation for small angles
 * @param w rotation vector (axis * angle[rad])
 * @return unit quaternion (w,x,y,z)
 */
DR_INLINE dr_quatf_t dr_q_from_rotvec( dr_vec3f_t w )
{
    float th = dr_v3_norm(w);
    dr_vec3f_t u;
    float s;
    if (th < 1e-8f) {
        // small angle approx
        return dr_q_normalize( (dr_quatf_t){1.0f, 0.5f * w.x, 0.5f * w.y, 0.5f * w.z});
    }
    u = dr_v3_scale(1.0f / th, w);
    s = sinf(0.5f * th);
    return dr_q_normalize((dr_quatf_t){cosf(0.5f * th), u.x * s, u.y * s, u.z * s});
}

/** @brief Rotate Vector by Quaternion (active, body->world)
 * @param q unit quaternion (w,x,y,z)
 * @param v vector to rotate    
 * @return rotated vector (q * [0,v] * q^-1)
 */
DR_INLINE dr_vec3f_t dr_q_rotate( dr_quatf_t q, dr_vec3f_t v )
{
    dr_quatf_t qc, vq, t, r;
    qc = (dr_quatf_t){q.w, -q.x, -q.y, -q.z}; // q conjugate
    vq = (dr_quatf_t){0.0f, v.x, v.y, v.z}; // vector as quaternion
    t = dr_q_mult(q, vq);
    r = dr_q_mult(t, qc);
    return dr_v3(r.x, r.y, r.z);
}

/** @brief Integrate quaternion using body angular rate over dt (right-multiply) 
 * @param q current quaternion (body->world)
 * @param omega body angular rate (rad/s)
 * @param dt timestep (s)
 * @return updated, normalized quaternion (body->world)
*/
DR_INLINE dr_quatf_t dr_q_integrate_gyro( dr_quatf_t q, dr_vec3f_t omega, float dt )
{
    dr_vec3f_t dth = dr_v3_scale(dt, omega);
    return dr_q_normalize(dr_q_mult(q, dr_q_from_rotvec(dth)));
}

/** @brief Build skew-symmetric cross-product matrix [v]_x
 * @param v 3-vector
 * @param S (out) 3x3 row-major skew-symmetric matrix
 * @return void
 */
DR_INLINE void dr_skew3( dr_vec3f_t v, float S[9] )
{
    S[0] = 0.0f;   S[1] = -v.z;  S[2] = v.y;
    S[3] = v.z;    S[4] = 0.0f;  S[5] = -v.x;
    S[6] = -v.y;   S[7] = v.x;   S[8] = 0.0f;
}

/** @brief Convert quaternion to 3x3 rotation matrix R(q) (body->world) 
 * @param q unit quaternion (w,x,y,z)
 * @param R (out) 3x3 row-major rotation matrix
 * @return void
*/
void dr_R_from_q( dr_quatf_t q, float R[9] )
{
    float w, x, y, z;
    float xx, yy, zz, xy, xz, yz, wx, wy, wz;
    w = q.w; x = q.x; y = q.y; z = q.z;
    xx = x * x; yy = y * y; zz = z * z;
    xy = x * y; xz = x * z; yz = y * z
    wx = w * x; wy = w * y; wz = w * z;
    // R = [ (1-2(yy+zz)) 2(xy-wz)   2(xz+wy)
    //       2(xy+wz)     (1-2(xx+zz)) 2(yz-wx)
    //       2(xz-wy)     2(yz+wx) (1-2(xx+yy)) ]
    R[0] = 1.0f - 2.0f * (yy + zz); R[1] = 2.0f * (xy - wz);           R[2] = 2.0f * (xz + wy);
    R[3] = 2.0f * (xy + wz);         R[4] = 1.0f - 2.0f * (xx + zz); R[5] = 2.0f * (yz - wx);
    R[6] = 2.0f * (xz - wy);         R[7] = 2.0f * (yz + wx);           R[8] = 1.0f - 2.0f * (xx + yy);
}

/** @brief Set Identity Matrix I(n) of size nxn in row-major storage
 * @param I (out) matrix to set
 * @param n size of matrix
 * @return void
*/
void dr_mat_set_identity( float *I, int n )
{
    int i, c;
    for (i = 0; i < n; i++)
        for (c = 0; c < n; c++)
            I[i * n + c] = (i == c) ? 1.0f : 0.0f;
}

/** @brief  Matrix Add: C = A + B for r x c matrices
 * @param C (out) result matrix
 * @param A left matrix
 * @param B right matrix
 * @param rows number of rows
 * @param cols number of columns
 * @return void
*/
void dr_mat_add( float *C, float *A, float *B, int rows, int cols )
{
    int N = rows * cols;
    int i;
    for (i = 0; i < N; i++) {
        C[i] = A[i] + B[i];
    }
}


/** @brief  Matrix Subtract: C = A - B for r x c matrices.
 * @param C (out) result matrix
 * @param A left matrix
 * @param B right matrix
 * @param rows number of rows
 * @param cols number of columns
 * @return void
*/
void dr_mat_sub( float *C, float *A, float *B, int rows, int cols )
{
    int N = rows * cols;
    int i;
    for (i = 0; i < N; i++) {
        C[i] = A[i] - B[i];
    }

}

/** @brief Matrix Multiply: C(mxp) = A(mxn) * B(nxp) 
 * @param C (out) result matrix
 * @param A left matrix
 * @param B right matrix
 * @param m number of rows in A, C
 * @param n number of cols in A, rows in B
 * @param p number of cols in B, C
 * @return void
*/
void dr_mat_mul( float *C, float *A, float *B, int m, int n, int p )
{
    int r, c, k;
    for (r = 0; r < m; r++) {
        for (c = 0; c < p; c++) {
            C[r * p + c] = 0.0f;
            for (k = 0; k < n; k++) {
                C[r * p + c] += A[r * n + k] * B[k * p + c];
            }
        }
    }
}

/** @brief  Matrix Transpose: AT = A^T (r x c -> c x r)
 * @param AT (out) transposed matrix
 * @param A input matrix
 * @param rows number of rows in A
 * @param cols number of columns in A
 * @return void
 */
void dr_mat_transpose( float *AT, const float *A, int rows, int cols )
{
    int r, c;
    for (r = 0; r < rows; r++) {
        for (c = 0; c < cols; c++) {
            AT[c * rows + r] = A[r * cols + c];
        }
    }
}

/** @brief Invert 3x3 Matrix using Guass-Jordan elimination 
 * @param inv (out) inverted matrix
 * @param M input matrix (3x3)
 * @return 0 on success, -1 if singular */
int dr_mat3_inv(float inv[9], const float M[9])
{
    float A[9]; // copy of M
    int i;
    float I[9] = {1,0,0, 0,1,0, 0,0,1}; // identity
    int piv;
    float amax, v, invp;
    for (i = 0; i < 9; i++) {
        A[i] = M[i];
    }
    // forward elimination
    for (i = 0; i < 3; i++) {
        piv = i;
        amax = fabsf(A[i * 3 + i]);
        for ( r = i + 1; r < 3; r++ ) {
            v = fabsf(A[r * 3 + i]);
            if ( v > amax ) {
                piv = r;
                amax = v;
            }
            if( amax < 1e-12f ) {
                return -1; // singular
            }
            if( piv != i ) {
                // swap rows
                for ( c = 0; c < 3; c++ ) {
                    v = A[i * 3 + c];
                    A[i * 3 + c] = A[piv * 3 + c];
                    A[piv * 3 + c] = v;
                    v = I[i * 3 + c];
                    I[i * 3 + c] = I[piv * 3 + c];
                    I[piv * 3 + c] = v;
                }
            }
            invp = 1.0f / A[i * 3 + i];
            for ( c = 0; c < 3; c++ ) {
                A[i * 3 + c] *= invp;
                I[i * 3 + c] *= invp;
            }
            for ( r = 0; r < 3; r++ ) {
                if ( r == i) continue;
                v = A[r * 3 + i];
                for ( c = 0; c < 3; c++ ) {
                    A[r * 3 + c] -= A[i * 3 + c] * v;
                    I[r * 3 + c] -= I[i * 3 + c] * v;
                }
            }

    }
    for (i = 0; i < 9; i++) {
        inv[i] = I[i];
    }
    return 0;

}