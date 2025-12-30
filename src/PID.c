#include "PID.h"

#include "stdio.h"

pid_control_t pidControl[CANT_PIDS];

float normalize(float value);

pid_struct_t convertPidFloatToStruct(pid_floats_t pidFloat, float period) {
    pid_struct_t pidStruct = {
        .kp = pidFloat.kp,
        .ki = pidFloat.ki,
        .kd = pidFloat.kd,
        .setPoint = pidFloat.setPoint,
        .sampleTimeInMs = period
    };
    return pidStruct;
}

void pidInit(pid_init_t initConfig) {

    for(uint8_t i=0;i<CANT_PIDS;i++) { 
        pidControl[i].sampleTimeInSec = initConfig.pids[i].sampleTimeInMs / 1000.00;
        pidControl[i].enablePID = false;
        pidSetConstants(i,initConfig.pids[i].kp, initConfig.pids[i].ki, initConfig.pids[i].kd);
        pidSetSetPoint(i,initConfig.pids[i].setPoint);

        printf("pidInit -> sampleTime %d: %dms\f",i, initConfig.pids[i].sampleTimeInMs);
    }
}

void pidSetEnable(uint8_t numPid) {
    pidControl[numPid].iTerm = 0.00;
    pidControl[numPid].lastInput = 0.00;
    pidControl[numPid].enablePID = true;
}

void pidSetDisable(uint8_t numPid) {
    pidControl[numPid].enablePID = false;
    // pidControl[numPid].iTerm = 0.00;
    // pidControl[numPid].lastInput = 0.00;
}

bool pidGetEnable(uint8_t numPid) {
    return pidControl[numPid].enablePID;
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
float pidCalculate(uint8_t numPid,float input) {
        if (!pidControl[numPid].enablePID) {
            return 0;
        }
        
        float normalizeInput = normalize(input);
        float error = pidControl[numPid].setPoint - normalizeInput;  													  // Error: diferencia entre el valor seteado y el de entrada
        
        pidControl[numPid].iTerm += pidControl[numPid].sampleTimeInSec * pidControl[numPid].ki * error;                                // calculo I: acumulo error multiplicado por ki contemplando el tiempo transcurrido desd el anterior
        float dTerm = ((normalizeInput - pidControl[numPid].lastInput) * pidControl[numPid].kd) / pidControl[numPid].sampleTimeInSec;  // Calculo D: resto la entrada anterior a la actual

        pidControl[numPid].iTerm = cutNormalizeLimits(pidControl[numPid].iTerm);
        dTerm = cutNormalizeLimits(dTerm);

        /*Suma de errores*/
        float output = (pidControl[numPid].kp * error) + pidControl[numPid].iTerm - dTerm;  													// opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate

        // printf(">pidIn:%f\n>normPid:%f\n",input,normalizeInput);

        pidControl[numPid].lastInput = normalizeInput;
        return cutNormalizeLimits(output);
}

/*
 * Funcion para asignar un nuevo setPoint
 */
void pidSetSetPoint(uint8_t numPid,float value) {
    pidControl[numPid].setPoint = normalize(value);
}

/*
 * Funcion para obtener el setPoint actual
 */
float pidGetSetPoint(uint8_t numPid) {
    return pidControl[numPid].setPoint * 100.00;
}

/*
 * 	Funcion para cargar los parametros al filtro PID
 */
void pidSetConstants(uint8_t numPid,float KP, float KI, float KD) {
    pidControl[numPid].kp = KP;
    pidControl[numPid].ki = KI;
    pidControl[numPid].kd = KD;
    pidControl[numPid].iTerm = 0.00;
    pidControl[numPid].lastInput = 0.00;
}

void pidClearTerms(uint8_t numPid) {
    pidControl[numPid].iTerm = 0.00;
    pidControl[numPid].lastInput = 0.00;
}
