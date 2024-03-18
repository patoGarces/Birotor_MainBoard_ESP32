#ifndef __PID_H__
#define __PID_H__

#include <stdio.h>
#include "stdbool.h"
#include "esp_err.h"

typedef struct {
	float kp;
    float ki;
    float kd;
	float initSetPoint;
	float sampleTimeInMs;
}pid_init_t;

typedef struct{
	uint8_t enablePID;
	float output;										//salida del PID, deberia estar entre [-90.00;90.00]
	float input;										//lectura del input del eje a equilibrar [-90.00;90.00]
	float lastInput;
	float iTerm;
	float kp;											//parametro P
	float ki;											//parametro I
	float kd;											//parametro D
	float setPoint;										//setPoint del PID, normalizado: [-1.00;1.00]
	float sampleTimeInSec;
} pid_control_t;

esp_err_t pidInit(pid_init_t params);
void pidSetEnable(void);
void pidSetDisable(void);
bool pidGetEnable(void);
float pidCalculate(float input);
void pidSetSetPoint(float value);
float pidGetSetPoint(void);
void pidSetConstants(float KP,float KI,float KD);

#endif
