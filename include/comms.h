#ifndef __COMMS_H__
#define __COMMS_H__

#include "stdio.h"
#include "main.h"
#include "utils.h"

#define FAILSAFE_TIMEOUT                100             // Timeout maximo sin recibir communicacion de la app, en ms * 10, ej: 10 = 100ms

#define HEADER_PACKAGE_STATUS           0xAB01          // key a enviar que indica que el paquete enviado a la app es un status
#define HEADER_PACKAGE_CONTROL          0xAB02          // key que indica que el paquete recibido de la app es de control
#define HEADER_PACKAGE_SETTINGS         0xAB03          // key que indica que el paquete recibido de la app es de configuracion
#define HEADER_PACKAGE_COMMAND          0xAB04          // key que indica que el paquete recibido de la app es un comando

#define HEADER_PACKAGE_LOCAL_CONFIG     0xAB05          // key que indica que el paquete a enviar es una setting local

enum CommandsToRobot {
    COMMAND_CALIBRATE_IMU,
    COMMAND_SAVE_LOCAL_CONFIG,
    COMMAND_GET_LOCAL_CONFIG,
    COMMAND_ARMED_ROBOT,
    COMMAND_DEARMED_ROBOT,
    COMMAND_CLEAN_WHEELS,
    COMMAND_PID_ANGLE_TEST,
    COMMAND_MOVE_DISTANCE,
    COMMAND_MOVE_ABS_YAW,
    COMMAND_MOVE_REL_YAW
};

/**
 * @brief Estructura de datos de configuracion recibida de la app
 */
 typedef struct {
    uint16_t headerPackage;
    uint16_t indexPid;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
    uint16_t safetyLimits;
} pid_settings_app_raw_t;

/**
 * @brief Estructura de datos de configuracion recibida de la app
 */
 typedef struct {
    uint16_t indexPid;
    float kp;
    float ki;
    float kd;
    float safetyLimits;
} pid_settings_comms_t;

/**
 * @brief Estructura de datos de control de velocidad
 */
typedef struct {
    uint16_t headerPackage;
    int16_t  angular_vel;               // en m/s
    int16_t  linear_vel;                // en rad/s
} velocity_command_t;

/**
 * @brief Estructura de datos de comandos recibida de la app
 */
 typedef struct {
    uint16_t    headerPackage;
    uint16_t    command;
    int16_t     value;
} command_app_raw_t;

/**
 * @brief Esta estructura de datos es la que se envia a la app android
 */
typedef struct {
    uint16_t headerPackage;
    uint16_t isCharging;
    uint16_t batVoltage;
    uint16_t imuTemp;
    uint16_t mcbTemp;
    uint16_t mainboardTemp;
    int16_t  speedMeasR;        // Velocidad rueda derecha en m/s
    int16_t  speedMeasL;      // Velocidad rueda izquierda en m/s
    int16_t  posWheelR;         // Posicion rueda derecha en metros
    int16_t  posWheelL;         // Posicion rueda izquierda en metros
    int16_t  currentR;
    int16_t  currentL;
    int16_t  pitch;
    int16_t  roll;
    int16_t  yaw;
    int16_t  collisionSensors[4];
    int16_t  posInMeters;
    int16_t  outputYawControl;
    int16_t  setPointAngle;
    int16_t  setPointPos;
    int16_t  setPointYaw;
    int16_t  setPointSpeed;
    uint16_t statusCode;
} robot_dynamic_data_t;

/**
 * @brief Estructura de datos enviada a la app, contiene settings locales
 */
typedef struct {
    uint16_t headerPackage;
    uint16_t safetyLimits;
    pid_params_raw_t pid[CANT_PIDS];
} robot_local_configs_comms_t;

void comms_start_up(void);
void sendDynamicData(robot_dynamic_data_t status);
void sendLocalConfig(robot_local_configs_t localConfig);
#endif