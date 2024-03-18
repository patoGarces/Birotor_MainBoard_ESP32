#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PERIOD_IMU_MS   2.5     

#define PIN_LED         2  //27 en mainBoard
#define PIN_OSCILO      47  //5
#define GPIO_LED_STATUS 48

// Pinout receptor SBUS
#define GPIO_SBUS_RX    13
#define GPIO_SBUS_TX    14
#define UART_SBUS_NUM   UART_NUM_2

// Pinout GPS
// TODO: asignar pines
#define GPIO_GPS_RX        0       
#define GPIO_GPS_TX        0
#define UART_GPS_NUM       UART_NUM_1
#define BAUDRATE_GPS_UBX    57600

// Pinout Servos
#define GPIO_SERVO_L        37
#define GPIO_SERVO_R        18
#define GPIO_MOTOR_L        35
#define GPIO_MOTOR_R        3
#define GPIO_LED_MOTOR_L    39
#define GPIO_LED_MOTOR_R    17

// Pinout MPU6050
#define GPIO_MPU_INT        9      
#define GPIO_MPU_SDA        40
#define GPIO_MPU_SCL        41

#define IMU_HANDLER_PRIORITY    configMAX_PRIORITIES - 2 //TODO: revisar esta prioridad
#define IMU_HANDLER_CORE        1

#define CENTER_ANGLE_MOUNTED    0.00  

#define OUTPUT_CHANNEL_MOT_L      0
#define OUTPUT_CHANNEL_MOT_R      1
#define OUTPUT_CHANNEL_SERVO_L    2
#define OUTPUT_CHANNEL_SERVO_R    3
#define OUTPUT_CHANNEL_LED_MOT_L  4
#define OUTPUT_CHANNEL_LED_MOT_R  5

enum {
    STATUS_ROBOT_INIT,
    STATUS_ROBOT_DISABLE,
    STATUS_ROBOT_ENABLE,
    STATUS_ROBOT_STABILIZED,
    STATUS_ROBOT_ERROR
};

enum {
    FLY_MODE_ATTI,
    FLY_MODE_STABILIZED,
    FLY_MODE_FAILSAFE
};

typedef struct {
    uint8_t motorL;
    uint8_t motorR;
    uint8_t servoL;
    uint8_t servoR;
    uint8_t dualRates;
    uint8_t motorArmed;
    uint8_t flyMode;
    uint8_t failsafe;
} drone_control_t;

typedef struct {
    uint8_t throttle;
    uint8_t aileron;
    uint8_t rudder;
    uint8_t elevator;
}rc_channels_t;

#endif
