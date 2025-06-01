// #include "imu_advanced.h"
// #include "bus.h"
// #include "pin.h"
// #include "config.h"
// #include "common.h"
// #include "touch.h"
// #include "hid.h"
// #include "led.h"
// #include "vector.h"
// #include "logging.h"


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include "imu_advanced.h"
#include "config.h"
#include "common.h"
#include "pin.h"
#include "bus.h"
#include "touch.h"
#include "hid.h"
#include "led.h"
#include "vector.h"
#include "logging.h"

uint8_t IMU0 = 0;
uint8_t IMU1 = 0;
double offset_gyro_0_x;
double offset_gyro_0_y;
double offset_gyro_0_z;
double offset_gyro_1_x;
double offset_gyro_1_y;
double offset_gyro_1_z;
double offset_accel_0_x;
double offset_accel_0_y;
double offset_accel_0_z;
double offset_accel_1_x;
double offset_accel_1_y;
double offset_accel_1_z;

void imu_channel_select() {
    Config *config = config_read();
    IMU0 = config->swap_gyros ? PIN_SPI_CS1 : PIN_SPI_CS0;
    IMU1 = config->swap_gyros ? PIN_SPI_CS0 : PIN_SPI_CS1;
}

void imu_readall(uint8_t cs, IntVector data[])
{
    // Read all of gyro and accel data without further processing.
    uint8_t buf[12];
    bus_spi_read(cs, IMU_READ | IMU_OUTX_L_G, buf, 12);
    data[0].y = (((int16_t)buf[1] << 8) + (int16_t)buf[0]);
    data[0].z = (((int16_t)buf[3] << 8) + (int16_t)buf[2]);
    data[0].x = (((int16_t)buf[5] << 8) + (int16_t)buf[4]);

    data[1].x = (((int16_t)buf[ 7] << 8) + (int16_t)buf[ 6]);
    data[1].y = (((int16_t)buf[ 9] << 8) + (int16_t)buf[ 8]);
    data[1].z = (((int16_t)buf[11] << 8) + (int16_t)buf[10]);
}

void imu_init_single(uint8_t cs, uint8_t gyro_conf) {
    uint8_t id = bus_spi_read_one(cs, IMU_READ | IMU_WHO_AM_I);
    bus_spi_write(cs, IMU_CTRL1_XL, IMU_CTRL1_XL_2G);
    bus_spi_write(cs, IMU_CTRL8_XL, IMU_CTRL8_XL_LP);
    

    bus_spi_write(cs, IMU_CTRL4_C, 0b00000010); // Enable LPF1
    bus_spi_write(cs, IMU_CTRL6_C, 0b00000001); // FTYPE selection for LPF1
    bus_spi_write(cs, IMU_CTRL2_G, gyro_conf);
    uint8_t xl = bus_spi_read_one(cs, IMU_READ | IMU_CTRL1_XL);
    uint8_t g = bus_spi_read_one(cs, IMU_READ | IMU_CTRL2_G);
    info("  IMU cs=%i id=0x%02x xl=0b%08i g=0b%08i\n", cs, id, bin(xl), bin(g));
    if (id == 0x00) {
        warn("Gyro was not able to initialize\n");
        config_set_problem_gyro(true);
    }
}



void imu_power_off_single(uint8_t cs) {
    bus_spi_write(cs, IMU_CTRL1_XL, IMU_CTRL1_XL_OFF);
    bus_spi_write(cs, IMU_CTRL2_G, IMU_CTRL2_G_OFF);
    uint8_t xl = bus_spi_read_one(cs, IMU_READ | IMU_CTRL1_XL);
    uint8_t g = bus_spi_read_one(cs, IMU_READ | IMU_CTRL2_G);
    info("IMU cs=%i xl=0b%08i g=0b%08i\n", cs, bin(xl), bin(g));
}

void imu_power_off() {
    imu_power_off_single(IMU0);
    imu_power_off_single(IMU1);
}

void imu_init() {
    info("INIT: IMU\n");
    imu_channel_select();
    imu_load_calibration();
    imu_init_single(IMU0, IMU_CTRL2_G_500);
    imu_init_single(IMU1, IMU_CTRL2_G_125);
}

Vector imu_read_gyro() {
    IntVector data[2];
    imu_readall(IMU0, data);
    IntVector gyro0 = data[0];
    IntVector accel0 = data[1];
    imu_readall(IMU1, data);
    IntVector gyro1 = data[0];
    IntVector accel1 = data[1];
    double weight = max(abs(gyro1.x), abs(gyro1.y)) / 32768.0;
    double weight_0 = ramp_mid(weight, 0.2);
    double weight_1 = 1 - weight_0;
    double x = (gyro0.x-offset_gyro_0_x) * weight_0 + (gyro1.x-offset_gyro_1_x) * weight_1/4;
    double y = (gyro0.y-offset_gyro_0_y) * weight_0 + (gyro1.y-offset_gyro_1_y) * weight_1/4;
    double z = (gyro0.z-offset_gyro_0_z) * weight_0 + (gyro1.z-offset_gyro_1_z) * weight_1/4;
    return (Vector){-x/2, -y/2, -z/2};
}

Vector imu_read_accel()
{
    IntVector data[2];
    imu_readall(IMU0, data);
    IntVector gyro0 = data[0];
    IntVector accel0 = data[1];
    imu_readall(IMU1, data);
    IntVector gyro1 = data[0];
    IntVector accel1 = data[1];

    return (Vector){
        (accel0.x + accel1.x) / 2 - offset_accel_0_x,
        (accel0.y + accel1.y) / 2 - offset_accel_0_y,
        (accel0.z + accel1.z) / 2 - offset_accel_0_z
    };
}


void imu_update(FloatVector *gyro, FloatVector *accel) {
    IntVector data[2];
    imu_readall(IMU0, data);
    IntVector gyro0 = data[0];
    IntVector accel0 = data[1];
    imu_readall(IMU1, data);
    IntVector gyro1 = data[0];
    IntVector accel1 = data[1];

    float weight = max(abs(gyro1.x), abs(gyro1.y)) / 32768.0f;
    float weight_1 = (1 - ramp_mid(weight, 0.2f));
    float weight_0 = 1 - weight_1;

    float x = (gyro0.x-offset_gyro_0_x) * weight_0 *GYRO_SENS_RADPS_500 + (gyro1.x-offset_gyro_1_x) * weight_1 * GYRO_SENS_RADPS_125;
    float y = (gyro0.y-offset_gyro_0_y) * weight_0 *GYRO_SENS_RADPS_500 + (gyro1.y-offset_gyro_1_y) * weight_1 * GYRO_SENS_RADPS_125;
    float z = (gyro0.z-offset_gyro_0_z) * weight_0 *GYRO_SENS_RADPS_500 + (gyro1.z-offset_gyro_1_z) * weight_1 * GYRO_SENS_RADPS_125;

    float ax = (accel0.x + accel1.x) / 2 - offset_accel_0_x;
    float ay = (accel0.y + accel1.y) / 2 - offset_accel_0_y;
    float az = (accel0.z + accel1.z) / 2 - offset_accel_0_z;
    gyro->x = x;
    gyro->y = y;
    gyro->z = z;
    accel->x = ax;
    accel->y = ay;
    accel->z = az;
}

void imu_calibrate_single(uint8_t cs, bool mode, double* x, double* y, double* z) {
    char *mode_str = mode ? "accel" : "gyro";
    info("IMU: cs=%i calibrating %s...\n", cs, mode_str);
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    // Determine number of samples.
    uint32_t nsamples;
    if (mode==1) {
        nsamples = CFG_CALIBRATION_SAMPLES_ACCEL;
    } else {
        nsamples = CFG_CALIBRATION_SAMPLES_GYRO;
        Config *config = config_read();
        if (config->long_calibration) {
            nsamples *= CFG_CALIBRATION_LONG_FACTOR;
        }
    }
    // Sampling.
    uint32_t i = 0;
    info("| 0%%%*s100%% |\n", CFG_CALIBRATION_PROGRESS_BAR - 10, "");
    while(i < nsamples) {
        FloatVector sample;
        IntVector data[2];
        imu_readall(IMU0, data);
        IntVector gyro0 = data[0];
        IntVector accel0 = data[1];
        if (!mode) {
            sample.x = gyro0.x;
            sample.y = gyro0.y;
            sample.z = gyro0.z;
        } else {
            sample.x = accel0.x;
            sample.y = accel0.y;
            sample.z = accel0.z;
        }
        sum_x += sample.x;
        sum_y += sample.y;
        sum_z += sample.z;
        i++;
        if (!(i % (nsamples / CFG_CALIBRATION_PROGRESS_BAR))) info("=");
    }
    // Average.
    *x = sum_x / nsamples;
    *y = sum_y / nsamples;
    *z = sum_z / nsamples;
    // Assuming the resting state of the controller is having a vector of 1G
    // pointing down. (Newton's fault for inventing the gravity /jk).
    if (mode==1) *z -= BIT_14;
    info("\nIMU: cs=%i %s calibrated x=%.02f y=%.02f z=%.02f\n", cs, mode_str, *x, *y, *z);
}

void imu_load_calibration() {
    Config *config = config_read();
    offset_gyro_0_x = config->offset_gyro_0_x - (config->offset_gyro_user_x * GYRO_USER_OFFSET_FACTOR);
    offset_gyro_0_y = config->offset_gyro_0_y - (config->offset_gyro_user_y * GYRO_USER_OFFSET_FACTOR);
    offset_gyro_0_z = config->offset_gyro_0_z - (config->offset_gyro_user_z * GYRO_USER_OFFSET_FACTOR);
    offset_gyro_1_x = config->offset_gyro_1_x - (config->offset_gyro_user_x * GYRO_USER_OFFSET_FACTOR);
    offset_gyro_1_y = config->offset_gyro_1_y - (config->offset_gyro_user_y * GYRO_USER_OFFSET_FACTOR);
    offset_gyro_1_z = config->offset_gyro_1_z - (config->offset_gyro_user_z * GYRO_USER_OFFSET_FACTOR);
    offset_accel_0_x = config->offset_accel_0_x;
    offset_accel_0_y = config->offset_accel_0_y;
    offset_accel_0_z = config->offset_accel_0_z;
    offset_accel_1_x = config->offset_accel_1_x;
    offset_accel_1_y = config->offset_accel_1_y;
    offset_accel_1_z = config->offset_accel_1_z;
}

void imu_reset_calibration() {
    offset_gyro_0_x = 0;
    offset_gyro_0_y = 0;
    offset_gyro_0_z = 0;
    offset_gyro_1_x = 0;
    offset_gyro_1_y = 0;
    offset_gyro_1_z = 0;
    offset_accel_0_x = 0;
    offset_accel_0_y = 0;
    offset_accel_0_z = 0;
    offset_accel_1_x = 0;
    offset_accel_1_y = 0;
    offset_accel_1_z = 0;
}

void imu_calibrate() {
    config_set_gyro_user_offset(0, 0, 0);
    imu_reset_calibration();
    imu_calibrate_single(PIN_SPI_CS0, 0, &offset_gyro_0_x, &offset_gyro_0_y, &offset_gyro_0_z);
    imu_calibrate_single(PIN_SPI_CS1, 0, &offset_gyro_1_x, &offset_gyro_1_y, &offset_gyro_1_z);
    imu_calibrate_single(PIN_SPI_CS0, 1, &offset_accel_0_x, &offset_accel_0_y, &offset_accel_0_z);
    imu_calibrate_single(PIN_SPI_CS1, 1, &offset_accel_1_x, &offset_accel_1_y, &offset_accel_1_z);
    config_set_gyro_offset(
        offset_gyro_0_x,
        offset_gyro_0_y,
        offset_gyro_0_z,
        offset_gyro_1_x,
        offset_gyro_1_y,
        offset_gyro_1_z
    );
    config_set_accel_offset(
        offset_accel_0_x,
        offset_accel_0_y,
        offset_accel_0_z,
        offset_accel_1_x,
        offset_accel_1_y,
        offset_accel_1_z
    );
    imu_load_calibration();
}
