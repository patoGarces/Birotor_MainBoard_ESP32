#include "PID.h"

#include "stdio.h"

pid_control_t pidControl1;

float normalize(float value);

esp_err_t pidInit(pid_init_t initParams) {
    if (initParams.sampleTimeInMs > 1000.00 || initParams.sampleTimeInMs < 1.00) {
        return ESP_FAIL;
    }
    pidControl1.sampleTimeInSec = (float)initParams.sampleTimeInMs / 1000;
    pidControl1.enablePID = false;
    pidSetConstants(initParams.kp, initParams.ki, initParams.kd);
    pidSetSetPoint(initParams.initSetPoint);
    return ESP_OK;
}

void pidSetEnable(void) {
    pidControl1.enablePID = true;
}

void pidSetDisable(void) {
    pidControl1.enablePID = false;
}

bool pidGetEnable(void) {
    return pidControl1.enablePID;
}

float normalize(float value) {
    return (value / 100.00);
}

float cutNormalizeLimits(float normalizeInput) {
    if (normalizeInput > 1.00) {  // recorto el termino integral maximo
        return 1.00;
    } else if (normalizeInput < -1.00) {  // recorto el termino intgral minimo
        return -1.00;
    }
    return normalizeInput;
}

/*
 * Esta funcion debe ser llamada cada un periodo fijo definido en pidControl1.sampleTime
 * @param input entra en rango [-100;100]
 * @return resultado del PID normalizado [-1.00;1.00]
 */
float pidCalculate(float input) {
    if (!pidControl1.enablePID) {
        return 0;
    }
    pidControl1.input = normalize(input);
    float error = pidControl1.setPoint - pidControl1.input;  													// Error: diferencia entre el valor seteado y el de entrada

    float kTerm = pidControl1.kp * error;                                                                        // calculo P: error multiplicado por su constante
    pidControl1.iTerm += pidControl1.sampleTimeInSec * pidControl1.ki * error;                                   // calculo I: acumulo error multiplicado por ki contemplando el tiempo transcurrido desd el anterior
    float dTerm = ((pidControl1.input - pidControl1.lastInput) * pidControl1.kd) / pidControl1.sampleTimeInSec;  // Calculo D: resto la entrada anterior a la actual

    pidControl1.iTerm = cutNormalizeLimits(pidControl1.iTerm);
    dTerm = cutNormalizeLimits(dTerm);

    /*Suma de errores*/
    pidControl1.output = kTerm + pidControl1.iTerm - dTerm;  													// opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate
    pidControl1.output = cutNormalizeLimits(pidControl1.output);

    pidControl1.lastInput = pidControl1.input;
    return pidControl1.output;
}

/*
 * Funcion para asignar un nuevo setPoint
 */
void pidSetSetPoint(float value) {
    pidControl1.setPoint = normalize(value);
}

/*
 * Funcion para obtener el setPoint actual
 */
float pidGetSetPoint(void) {
    return pidControl1.setPoint * 100.00;
}

/*
 * 	Funcion para cargar los parametros al filtro PID
 */
void pidSetConstants(float KP, float KI, float KD) {
    pidControl1.kp = KP;
    pidControl1.ki = KI;
    pidControl1.kd = KD;
    pidControl1.iTerm = 0.00;
    pidControl1.lastInput = 0.00;
}
