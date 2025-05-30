/* This method was taken from https://cookierobotics.com/073/ and converted from python to C */
#include "rotation_ekf.h"
#include <math.h>
#include <string.h>


typedef struct {
    float x[STATE_SIZE];      // [qw, qx, qy, qz, bx, by, bz]
    float P[STATE_SIZE][STATE_SIZE];
    float Q[3][3];
    float Q_bias[STATE_SIZE][STATE_SIZE];
    float R[3][3];
} EKF;

// Helper functions for vector/matrix/quaternion math
void normalize(float *v, int n) {
    float norm = 0.0;
    for (int i = 0; i < n; ++i) norm += v[i] * v[i];
    norm = sqrtf(norm);
    if (norm > 1e-12) {
        for (int i = 0; i < n; ++i) v[i] /= norm;
    }
}

void quaternion_identity(float *q) {
    q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
}

void quaternion_to_matrix(const float *q, float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float w2 = w*w, x2 = x*x, y2 = y*y, z2 = z*z;
    R[0][0] = w2 + x2 - y2 - z2;
    R[0][1] = 2 * (x*y - w*z);
    R[0][2] = 2 * (w*y + x*z);
    R[1][0] = 2 * (x*y + w*z);
    R[1][1] = w2 - x2 + y2 - z2;
    R[1][2] = 2 * (y*z - w*x);
    R[2][0] = 2 * (x*z - w*y);
    R[2][1] = 2 * (y*z + w*x);
    R[2][2] = w2 - x2 - y2 + z2;
}

void quaternion_multiply(const float *p, const float *q, float *r) {
    float pw = p[0], px = p[1], py = p[2], pz = p[3];
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    r[0] = pw*qw - px*qx - py*qy - pz*qz;
    r[1] = pw*qx + px*qw + py*qz - pz*qy;
    r[2] = pw*qy - px*qz + py*qw + pz*qx;
    r[3] = pw*qz + px*qy - py*qx + pz*qw;
}

void quaternion_from_axis_angle(const float *axis, float angle, float *q) {
    float sin_half = sinf(angle / 2.0);
    q[0] = cosf(angle / 2.0);
    q[1] = axis[0] * sin_half;
    q[2] = axis[1] * sin_half;
    q[3] = axis[2] * sin_half;
}

void quaternion_from_rotation_vector(const float *v, float *q, float eps) {
    float angle = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (angle > eps) {
        float axis[3] = { v[0]/angle, v[1]/angle, v[2]/angle };
        quaternion_from_axis_angle(axis, angle, q);
    } else {
        quaternion_identity(q);
    }
}

// Matrix and vector math helpers (implement as needed)
void mat_mult(float *A, int a_rows, int a_cols, float *B, int b_cols, float *C) {
    // C = A * B, where A is a_rows x a_cols, B is a_cols x b_cols, C is a_rows x b_cols
    for (int i = 0; i < a_rows; ++i) {
        for (int j = 0; j < b_cols; ++j) {
            C[i*b_cols + j] = 0.0;
            for (int k = 0; k < a_cols; ++k) {
                C[i*b_cols + j] += A[i*a_cols + k] * B[k*b_cols + j];
            }
        }
    }
}

// Fill in get_F, get_W, get_H, f, h as in Python, but in C
void get_F(const float *x, const float *w, float dt, float F[STATE_SIZE][STATE_SIZE]) {
    float qw = x[0], qx = x[1], qy = x[2], qz = x[3];
    float bx = x[4], by = x[5], bz = x[6];
    float wx = w[0], wy = w[1], wz = w[2];
    memset(F, 0, sizeof(float)*STATE_SIZE*STATE_SIZE);
    F[0][0] = 1;
    F[0][1] = dt*(-wx + bx)/2;
    F[0][2] = dt*(-wy + by)/2;
    F[0][3] = dt*(-wz + bz)/2;
    F[0][4] = dt*qx/2;
    F[0][5] = dt*qy/2;
    F[0][6] = dt*qz/2;

    F[1][0] = dt*(wx - bx)/2;
    F[1][1] = 1;
    F[1][2] = dt*(wz - bz)/2;
    F[1][3] = dt*(-wy + by)/2;
    F[1][4] = -dt*qw/2;
    F[1][5] = dt*qz/2;
    F[1][6] = -dt*qy/2;

    F[2][0] = dt*(wy - by)/2;
    F[2][1] = dt*(-wz + bz)/2;
    F[2][2] = 1;
    F[2][3] = dt*(wx - bx)/2;
    F[2][4] = -dt*qz/2;
    F[2][5] = -dt*qw/2;
    F[2][6] = dt*qx/2;

    F[3][0] = dt*(wz - bz)/2;
    F[3][1] = dt*(wy - by)/2;
    F[3][2] = dt*(-wx + bx)/2;
    F[3][3] = 1;
    F[3][4] = dt*qy/2;
    F[3][5] = -dt*qx/2;
    F[3][6] = -dt*qw/2;

    F[4][4] = 1;
    F[5][5] = 1;
    F[6][6] = 1;
}

void get_W(const float *x, float dt, float W[STATE_SIZE][3]) {
    float qw = x[0], qx = x[1], qy = x[2], qz = x[3];
    memset(W, 0, sizeof(float)*STATE_SIZE*3);
    W[0][0] = -qx * dt/2; W[0][1] = -qy * dt/2; W[0][2] = -qz * dt/2;
    W[1][0] =  qw * dt/2; W[1][1] = -qz * dt/2; W[1][2] =  qy * dt/2;
    W[2][0] =  qz * dt/2; W[2][1] =  qw * dt/2; W[2][2] = -qx * dt/2;
    W[3][0] = -qy * dt/2; W[3][1] =  qx * dt/2; W[3][2] =  qw * dt/2;
    // W[4:6][:] = 0
}

void get_H(const float *x, float H[3][STATE_SIZE]) {
    float qw = x[0], qx = x[1], qy = x[2], qz = x[3];
    memset(H, 0, sizeof(float)*3*STATE_SIZE);
    H[0][0] = 2*G_ACCEL*qy; H[0][1] = -2*G_ACCEL*qz; H[0][2] = 2*G_ACCEL*qw; H[0][3] = -2*G_ACCEL*qx;
    H[1][0] = -2*G_ACCEL*qx; H[1][1] = -2*G_ACCEL*qw; H[1][2] = -2*G_ACCEL*qz; H[1][3] = -2*G_ACCEL*qy;
    H[2][0] = -2*G_ACCEL*qw; H[2][1] = 2*G_ACCEL*qx; H[2][2] = 2*G_ACCEL*qy; H[2][3] = -2*G_ACCEL*qz;
}

void predict_func(const float *x, const float *w, float dt, float *x_out) {
    float q[4] = { x[0], x[1], x[2], x[3] };
    float b[3] = { x[4], x[5], x[6] };
    float d_ang[3] = { (w[0]-b[0])*dt, (w[1]-b[1])*dt, (w[2]-b[2])*dt };
    float dq[4];
    quaternion_from_rotation_vector(d_ang, dq, 0.0);
    float q_new[4];
    quaternion_multiply(q, dq, q_new);
    normalize(q_new, 4);
    for (int i = 0; i < 4; ++i) x_out[i] = q_new[i];
    for (int i = 0; i < 3; ++i) x_out[4+i] = b[i];
}

void measure_func(const float *x, float *out) {
    float q[4] = { x[0], x[1], x[2], x[3] };
    float R[3][3];
    quaternion_to_matrix(q, R);
    float v[3] = { 0, 0, -G_ACCEL };
    // out = R^T * v
    for (int i = 0; i < 3; ++i) {
        out[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            out[i] += R[j][i] * v[j];
        }
    }
}

// EKF methods
void EKF_init(EKF *ekf, const float *q0, const float *b0, float init_gyro_bias_err,
              float gyro_noise, float gyro_bias_noise, float accelerometer_noise) {
    for (int i = 0; i < 4; ++i) ekf->x[i] = q0[i];
    for (int i = 0; i < 3; ++i) ekf->x[4+i] = b0[i];
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 4; ++i) ekf->P[i][i] = 0.01;
    for (int i = 0; i < 3; ++i) ekf->P[4+i][4+i] = init_gyro_bias_err * init_gyro_bias_err;
    memset(ekf->Q, 0, sizeof(ekf->Q));
    for (int i = 0; i < 3; ++i) ekf->Q[i][i] = gyro_noise * gyro_noise;
    memset(ekf->Q_bias, 0, sizeof(ekf->Q_bias));
    for (int i = 0; i < 3; ++i) ekf->Q_bias[4+i][4+i] = gyro_bias_noise * gyro_bias_noise;
    memset(ekf->R, 0, sizeof(ekf->R));
    for (int i = 0; i < 3; ++i) ekf->R[i][i] = accelerometer_noise * accelerometer_noise;
}

void EKF_predict(EKF *ekf, const float *w, float dt) {
    float F[STATE_SIZE][STATE_SIZE];
    float W[STATE_SIZE][3];
    get_F(ekf->x, w, dt, F);
    get_W(ekf->x, dt, W);

    float x_new[STATE_SIZE];
    predict_func(ekf->x, w, dt, x_new);
    memcpy(ekf->x, x_new, sizeof(float)*STATE_SIZE);

    // P = F*P*F^T + W*Q*W^T + Q_bias

    // Temporary matrices
    float FP[STATE_SIZE][STATE_SIZE] = {0};
    float FPFt[STATE_SIZE][STATE_SIZE] = {0};
    float WQ[STATE_SIZE][3] = {0};
    float WQWt[STATE_SIZE][STATE_SIZE] = {0};

    // FP = F * P
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            FP[i][j] = 0.0;
            for (int k = 0; k < STATE_SIZE; ++k) {
                FP[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    // FPFt = FP * F^T
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            FPFt[i][j] = 0.0;
            for (int k = 0; k < STATE_SIZE; ++k) {
                FPFt[i][j] += FP[i][k] * F[j][k]; // F^T
            }
        }
    }

    // WQ = W * Q
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < 3; ++j) {
            WQ[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                WQ[i][j] += W[i][k] * ekf->Q[k][j];
            }
        }
    }

    // WQWt = WQ * W^T
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            WQWt[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                WQWt[i][j] += WQ[i][k] * W[j][k];
            }
        }
    }

    // P = FPFt + WQWt + Q_bias
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            ekf->P[i][j] = FPFt[i][j] + WQWt[i][j] + ekf->Q_bias[i][j];
        }
    }
}

void EKF_update(EKF *ekf, const float *a) {
    float a_norm[3] = { a[0], a[1], a[2] };
    normalize(a_norm, 3);
    for (int i = 0; i < 3; ++i) a_norm[i] *= G_ACCEL;

    float hx[3];
    measure_func(ekf->x, hx);

    float y[3];
    for (int i = 0; i < 3; ++i) y[i] = a_norm[i] - hx[i];

    float H[3][STATE_SIZE];
    get_H(ekf->x, H);

    // S = H*P*H^T + R
    float S[3][3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] = ekf->R[i][j];
            for (int k = 0; k < STATE_SIZE; ++k) {
                for (int l = 0; l < STATE_SIZE; ++l) {
                    S[i][j] += H[i][k] * ekf->P[k][l] * H[j][l];
                }
            }
        }
    }

    // Compute S_inv (inverse of 3x3 matrix)
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
               - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
               + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    float S_inv[3][3];
    if (fabs(det) < 1e-12) {
        // Singular, do nothing
        return;
    }
    float invdet = 1.0 / det;
    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invdet;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invdet;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invdet;
    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invdet;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invdet;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invdet;
    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invdet;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invdet;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invdet;

    // K = P*H^T * inv(S)
    float PHt[STATE_SIZE][3] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < STATE_SIZE; ++k) {
                PHt[i][j] += ekf->P[i][k] * H[j][k];
            }
        }
    }
    float K[STATE_SIZE][3] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                K[i][j] += PHt[i][k] * S_inv[k][j];
            }
        }
    }

    // x = x + K*y
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < 3; ++j) {
            ekf->x[i] += K[i][j] * y[j];
        }
    }
    // x[0:4] = normalize(x[0:4])
    normalize(ekf->x, 4);

    // P = (I - K*H)*P
    float KH[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            for (int k = 0; k < 3; ++k) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }
    float I_KH[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }
    float newP[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            for (int k = 0; k < STATE_SIZE; ++k) {
                newP[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }

    // Ensure symmetry in the covariance matrix
    // P = 0.5*(P + P^T)
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < i; ++j) {
            float sym = 0.5 * (newP[i][j] + newP[j][i]);
            newP[i][j] = sym;
            newP[j][i] = sym;
        }
    }
    memcpy(ekf->P, newP, sizeof(newP));
}