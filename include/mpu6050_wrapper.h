#ifndef MPU6050_WRAPPER_H
#define MPU6050_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    gpio_num_t sclGpio;
    gpio_num_t sdaGpio;
    gpio_num_t intGpio;
    uint8_t priorityTask;
    uint8_t core;
} mpu6050_init_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_wrapper_t;

typedef struct {
    float x;
    float y;
    float z;
} vector_float_wrapper_t;

typedef struct {
    float angles[3];
    float temp;
} vector_queue_t;

void mpu6050_initialize(mpu6050_init_t *config);
// int mpu6050_testConnection();
void mpu6050_recalibrate();

#ifdef __cplusplus
}
#endif

#endif // MPU6050_WRAPPER_H
