#define G_ACCEL 9.80665
#define STATE_SIZE 7
#define Q_SIZE 4
#define B_SIZE 3

typedef struct {
    float x[STATE_SIZE];      // [qw, qx, qy, qz, bx, by, bz]
    float P[STATE_SIZE][STATE_SIZE];
    float Q[3][3];
    float Q_bias[STATE_SIZE][STATE_SIZE];
    float R[3][3];
} EKF;

// Helper functions for vector/matrix/quaternion math
void normalize(float *v, int n);
void quaternion_identity(float *q);

void quaternion_to_matrix(const float *q, float R[3][3]);

void quaternion_multiply(const float *p, const float *q, float *r);

void quaternion_from_axis_angle(const float *axis, float angle, float *q);

void quaternion_from_rotation_vector(const float *v, float *q, float eps);

void quaternion_to_rotation_vector(const float *q, float *v) ;

// Matrix and vector math helpers (implement as needed)
void mat_mult(float *A, int a_rows, int a_cols, float *B, int b_cols, float *C);

// Fill in get_F, get_W, get_H, f, h as in Python, but in C
void get_F(const float *x, const float *w, float dt, float F[STATE_SIZE][STATE_SIZE]);

void get_W(const float *x, float dt, float W[STATE_SIZE][3]);

void get_H(const float *x, float H[3][STATE_SIZE]);

void predict_func(const float *x, const float *w, float dt, float *x_out);

void measure_func(const float *x, float *out) ;

// EKF methods
void EKF_init(EKF *ekf, const float *q0, const float *b0, float init_gyro_bias_err,
              float gyro_noise, float gyro_bias_noise, float accelerometer_noise);

void EKF_predict(EKF *ekf, const float *w, float dt);

void EKF_update(EKF *ekf, const float *a);