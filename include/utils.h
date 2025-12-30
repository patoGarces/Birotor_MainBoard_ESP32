#ifndef UTILS_H
#define UTILS_H

#include "stdio.h"
#include "main.h"
#include "PID.h"

typedef struct {
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
} pid_params_raw_t;

typedef struct {
    uint16_t            safetyLimits;
    pid_params_raw_t    pids[CANT_PIDS];
} robot_local_configs_raw_t;

pid_params_raw_t convertPidFloatsToRaw(pid_floats_t pidParams);
pid_floats_t convertPidRawToFloats(pid_params_raw_t pidRaw);
robot_local_configs_raw_t convertLocalConfigToRaw(robot_local_configs_t localConfig);
robot_local_configs_t convertLocalConfigToFloat(robot_local_configs_raw_t rawConfig);

#endif