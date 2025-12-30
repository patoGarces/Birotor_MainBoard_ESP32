#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PERIOD_IMU_MS   2.5     

#define COMMS_HANDLER_CORE          PRO_CPU_NUM     // core 0
#define IMU_HANDLER_CORE            APP_CPU_NUM     // core 1

#define MPU_HANDLER_PRIORITY        configMAX_PRIORITIES - 1
#define IMU_HANDLER_PRIORITY        configMAX_PRIORITIES - 2
#define ATTITUDE_HANDLER_PRIORITY   configMAX_PRIORITIES - 3
#define COMM_HANDLER_PRIORITY       configMAX_PRIORITIES - 4

#define PERIOD_PID_PRIMARY_MS       100
#define PERIOD_PID_SECONDARY_MS     40

#define PRECISION_DECIMALS_COMMS    100.00              // Precision al convertir la data cruda a float, en este caso 100 = 0.01

#define MAX_ANGLE_CONTROL           10.0
#define GAIN_PITCH_PID_OUTPUT       5.0
#define GAIN_ROLL_PID_OUTPUT       5.0

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

#define CENTER_ANGLE_MOUNTED    0.00  

#define OUTPUT_CHANNEL_MOT_L      0
#define OUTPUT_CHANNEL_MOT_R      1
#define OUTPUT_CHANNEL_SERVO_L    2
#define OUTPUT_CHANNEL_SERVO_R    3
#define OUTPUT_CHANNEL_LED_MOT_L  4
#define OUTPUT_CHANNEL_LED_MOT_R  5

#define CANT_PIDS 4 // TODO: ajustar cantidad

enum {
    PID_PITCH,
    PID_ROLL,
    PID_YAW,
};

enum {
    ANGLE_YAW,
    ANGLE_ROLL,
    ANGLE_PITCH,
};

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
} drone_control_t;

typedef struct {
    float throttle;
    float aileron;
    float rudder;
    float elevator;
    float dualRates;
} rc_channels_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float setPoint;
} pid_floats_t;

/**
 * @brief Estructura de datos enviada a la app, contiene settings locales
 */
typedef struct {
    float safetyLimits;
    pid_floats_t pids[CANT_PIDS];
} robot_local_configs_t;

/**
 * @brief Esta estructura de datos generica del robot
 */
typedef struct {
    uint8_t                 isCharging;
    // uint8_t                 isMcbConnected;
    uint16_t                batVoltage;
    uint16_t                batPercent;
    float                   tempImu;
    // float                   tempMcb;
    float                   tempMainboard;
    float                   actualPitch;
    float                   actualRoll;
    float                   actualYaw;
    // float                   collisionSensors[4];
    // int16_t                 speedTargetR;
    // int16_t                 speedTargetL;
    // int16_t                 speedMeasR;
    // int16_t                 speedMeasL;
    // float                   posInMetersR;
    // float                   posInMetersL;
    // int16_t                 currentR;
    // int16_t                 currentL;
    // float                   actualDistInCms;
    // float                   outputYawControl;
    uint8_t                 motorArmed;
    uint8_t                 flyMode;
    uint8_t                 failsafe;
    rc_channels_t           rcControl;
    drone_control_t         outputControl;
    robot_local_configs_t   localConfig;
    // uint16_t                statusCode;
} status_robot_t;

#endif
