#include "utils.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

pid_params_raw_t convertPidFloatsToRaw(pid_floats_t pidParams) {

    pid_params_raw_t pidRaw = {
        .kp = pidParams.kp * PRECISION_DECIMALS_COMMS,
        .ki = pidParams.ki * PRECISION_DECIMALS_COMMS,
        .kd = pidParams.kd * PRECISION_DECIMALS_COMMS,
    };
    return pidRaw;
}

pid_floats_t convertPidRawToFloats(pid_params_raw_t pidRaw) {

    pid_floats_t pidParams = {
        .kp = pidRaw.kp / PRECISION_DECIMALS_COMMS,
        .ki = pidRaw.ki / PRECISION_DECIMALS_COMMS,
        .kd = pidRaw.kd / PRECISION_DECIMALS_COMMS,
    };
    return pidParams;
}

/*
 * Convierto los parametros raw a float
*/
robot_local_configs_raw_t convertLocalConfigToRaw(robot_local_configs_t localConfig) {

    pid_params_raw_t pids[CANT_PIDS];
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        pids[i] = convertPidFloatsToRaw(localConfig.pids[i]);
    }

    robot_local_configs_raw_t pidRaw = {
        .safetyLimits = localConfig.safetyLimits * PRECISION_DECIMALS_COMMS,
    };
    memcpy(pidRaw.pids,pids,sizeof(pids));

    return pidRaw;
}

/*
 * Convierto a float los parametros en raw
*/
robot_local_configs_t convertLocalConfigToFloat(robot_local_configs_raw_t rawConfig) {

    pid_floats_t pids[CANT_PIDS];
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        pids[i] = convertPidRawToFloats(rawConfig.pids[i]);
    }

    robot_local_configs_t localConfigParams = {
        .safetyLimits = rawConfig.safetyLimits * PRECISION_DECIMALS_COMMS,
    };
    memcpy(localConfigParams.pids,pids,sizeof(pids));

    return localConfigParams;
}