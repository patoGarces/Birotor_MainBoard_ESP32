#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PERIOD_IMU_MS   10      

#define PIN_LED         2  //27 en mainBoard
#define PIN_OSCILO      13

#define GPIO_CAN_TX     14
#define GPIO_CAN_RX     12
#define UART_PORT_CAN   UART_NUM_1

#define IMU_HANDLER_PRIORITY    6
#define IMU_HANDLER_CORE        1

#define CENTER_ANGLE_MOUNTED    0.00  

#define OUTPUT_INDEX_MOT_L      0
#define OUTPUT_INDEX_MOT_R      1
#define OUTPUT_INDEX_SERVO_L    2
#define OUTPUT_INDEX_SERVO_R    3

typedef struct{
    uint8_t motorL;
    uint8_t motorR;
    uint8_t servoL;
    uint8_t servoR;
    uint8_t dualRates;
    uint8_t motorArmed;
}drone_control_t;

enum{
    STATUS_ROBOT_INIT,
    STATUS_ROBOT_DISABLE,
    STATUS_ROBOT_ENABLE,
    STATUS_ROBOT_STABILIZED,
    STATUS_ROBOT_ERROR
};

#endif
