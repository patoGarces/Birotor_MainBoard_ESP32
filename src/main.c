#include "stdio.h"
#include "main.h"
#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "math.h"

#include "utils.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "mpu6050_wrapper.h"
#include "AS5600.h"

/* Incluyo componentes */
#include "../components/SBUS_COMMS/include/SBUS_COMMS.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"
#include "../components/SERVO_CONTROL/include/SERVO_CONTROL.h"
#include "../components/GPS_UBX/include/GPS_UBX.h"
#include "../components/WS2812/include/WS2812.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define DEVICE_BT_NAME          "Birotor Drone"

extern QueueHandle_t mpu6050QueueHandler;                   // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t queueNewSBUS;
QueueHandle_t newPidParamsQueue;                            // Recibo nuevos parametros relacionados al pid
QueueHandle_t outputMotorQueue;                             // Envio nuevos valores de salida para el control de motores
QueueHandle_t queueReceiveControl;
QueueHandle_t queueMotorsPosition;

status_robot_t statusDrone;                                  // Estructura que contiene todos los parametros de status a enviar a la app

const rc_channels_t failsafeChannels = {
    .aileron = 50,
    .elevator = 50,
    .rudder = 50,
};

const uint8_t failsafePosChannels[6] = { 0,0,50,50,50,50 };

static inline float clamp(float v, float min, float max) {
    return v < min ? min : (v > max ? max : v);
}

void armedMotors(uint8_t armed) {
    statusDrone.motorArmed = armed;
    if (armed && statusDrone.flyMode == FLY_MODE_ATTI) {
        statusLedUpdate(STATUS_LED_ARMED_ATTI);
    }
    else if (armed && statusDrone.flyMode == FLY_MODE_STABILIZED) {
        statusLedUpdate(STATUS_LED_ARMED_STABILIZED);
    }
    else {
        statusLedUpdate(STATUS_LED_WAITING_ARM);
    }
}

void failsafeMode(uint8_t failsafe) {

    statusDrone.failsafe = failsafe;

    if (failsafe) {
        statusDrone.flyMode = FLY_MODE_FAILSAFE;
        statusLedUpdate(STATUS_LED_FAILSAFE);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,failsafePosChannels[OUTPUT_CHANNEL_SERVO_L] * 10.00);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,failsafePosChannels[OUTPUT_CHANNEL_SERVO_R] * 10.00);
        printf("FailsafeMode activate\n");
    }
    else {
        armedMotors(statusDrone.motorArmed);
        printf("FailsafeMode desactivate, armed: %d\n",statusDrone.motorArmed);
    }
}

static void imuControlHandler(void *pvParameters) {
    float newAngles[3];
    as5600_queue_data_t sensorPositionData = {0};
    uint8_t cont = 0;
    float phaseMotorR = 0.00, phaseMotorL = 0.00;

    while(1) {

        if (xQueueReceive(queueMotorsPosition, &sensorPositionData, 0)) {
            if (sensorPositionData.side == AS5600_IZQ) {
                phaseMotorL = sinf(sensorPositionData.radAngle + AS5600_MOUNT_OFFSET_TETHA_L) * SINE_RESPONSE_GAIN;
                // printf(">angle_izq: %f\n>angle_rad_izq: %f\n", phaseMotorL, sensorPositionData.radAngle);
            }

            if (sensorPositionData.side == AS5600_DER) {
                phaseMotorR = sinf(sensorPositionData.radAngle + AS5600_MOUNT_OFFSET_TETHA_R) * SINE_RESPONSE_GAIN;
                // printf(">angle_der: %f\n>angle_rad_der: %f\n", phaseMotorR, sensorPositionData.radAngle);
            }
        }

        if (xQueueReceive(mpu6050QueueHandler,&newAngles, pdMS_TO_TICKS(10))) {
            // if (pidGetEnable(PID_PITCH) && statusDrone.flyMode == FLY_MODE_STABILIZED) {

                statusDrone.actualPitch = newAngles[ANGLE_PITCH];
                statusDrone.actualRoll = newAngles[ANGLE_ROLL];
                statusDrone.actualYaw = newAngles[ANGLE_YAW];

                // printf("pitch: %f\n", statusDrone.actualPitch);

                float outputPidPitch = pidCalculate(PID_PITCH, newAngles[ANGLE_PITCH]) * GAIN_PITCH_PID_OUTPUT;
                float outputPidRoll = pidCalculate(PID_ROLL, newAngles[ANGLE_ROLL]) * GAIN_ROLL_PID_OUTPUT;

            // }
            // else if (statusDrone.flyMode == FLY_MODE_ATTI) {
            //     controlOutput = rcChannels;
            // }
            // else { // if (statusDrone.flyMode == FLY_MODE_FAILSAFE) {
            //     statusDrone.rcControl = failsafeChannels;
            // }

            float mixedServoL = -outputPidPitch + (statusDrone.rcControl.rudder * CH_RUD_GAIN);
            float mixedServoR = outputPidPitch + (statusDrone.rcControl.rudder * CH_RUD_GAIN);
            
            statusDrone.outputControl.servoL = (clamp(mixedServoL, -0.5, 0.5) + 0.5) * 100.00;
            statusDrone.outputControl.servoR = (clamp(mixedServoR, -0.5, 0.5) + 0.5) * 100.00;

            pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,statusDrone.outputControl.servoL);
            pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,statusDrone.outputControl.servoR);

            if (statusDrone.motorArmed) {
                float t = statusDrone.rcControl.throttle * 2.0f;
                t = clamp(t, 0.0f, 1.0f);

                outputPidRoll = 0.00; // TODO: borrar
                // outputPidPitch = 0.00; // TODO: borrar
                float baseL = t + outputPidRoll;
                float baseR = t - outputPidRoll;

                float anglePhaseModulated = phaseMotorR * outputPidPitch;

                float mixL = baseL * (1.0f + (phaseMotorL * outputPidPitch));
                float mixR = baseR * (1.0f + anglePhaseModulated);

                mixL = clamp(mixL, 0.0f, 1.0f);
                mixR = clamp(mixR, 0.0f, 1.0f);

                statusDrone.outputControl.motorL = VEL_MOTORS_ARMED + mixL * (100.0f - VEL_MOTORS_ARMED);
                statusDrone.outputControl.motorR = VEL_MOTORS_ARMED + mixR * (100.0f - VEL_MOTORS_ARMED);

                printf(">sinR: %f\n>t: %f\n>pitch: %f\n>mixL: %f\n>mixR: %f\n>angleModulated: %f\n", phaseMotorR, t, outputPidPitch, mixL, mixR, anglePhaseModulated);
            }
            // #ifdef TELEPLOT_OUTPUT
                // printf(">sinL: %f, mixL: %f", phaseMotorL, mixL);
            // #else 
                // pwmSetOutput(OUTPUT_CHANNEL_MOT_L, statusDrone.outputControl.motorL * 10.00);
                // pwmSetOutput(OUTPUT_CHANNEL_MOT_R, statusDrone.outputControl.motorR * 10.00);
            // #endif

            // ESP_LOGI("imuControlHandler","servoL: %d, motorL: %d",statusDrone.outputControl.servoL, statusDrone.outputControl.motorL);

            // ESP_LOGI("imuControlHandler","thr: %f, ail: %f, motorL: %d, motorR: %d",statusDrone.rcControl.throttle, statusDrone.rcControl.aileron, statusDrone.outputControl.motorL, statusDrone.outputControl.motorR);

            // ESP_LOGI("imuControlHandler","pitchAngle: %f, ail: %f, pidOut: %f", statusDrone.actualPitch, statusDrone.rcControl.elevator, outputPidPitch);
            // ESP_LOGI("imuControlHandler","thr: %f, ail: %f, elev: %f, rudd: %f",statusDrone.rcControl.throttle, statusDrone.rcControl.aileron, statusDrone.rcControl.elevator, statusDrone.rcControl.rudder);
            // ESP_LOGI("imuControlHandler","L: %d\tR: %d", statusDrone.outputControl.motorL, statusDrone.outputControl.motorR);
        }
    }
}

// static void updateParams(void *pvParameters) {      // TODO: falta convertirlo a multi pid

//     pid_settings_comms_t newPidParams;

//     while (1){
        
//         if(xQueueReceive(newPidParamsQueue,&newPidParams,pdMS_TO_TICKS(10))){
//             pidSetConstants(PID_PITCH, newPidParams.kp,newPidParams.ki,newPidParams.kd);
//             pidSetSetPoint(PID_PITCH, 0.00);
//             storageWritePidParams(newPidParams);            

//             // statusDrone.P = newPidParams.kp*100;  
//             // statusDrone.I = newPidParams.ki*100;  
//             // statusDrone.D = newPidParams.kd*100; 
//             // statusDrone.centerAngle = newPidParams.center_angle;   
//             // statusDrone.safetyLimits = newPidParams.safety_limits;              
//         }
//     }
// }

static void attitudeControl(void *pvParameters){
    uint8_t failsafeLastState = false;
    channels_control_t newControlMessage;

    while(true) {
        if (xQueueReceive(queueNewSBUS,&newControlMessage, pdMS_TO_TICKS(10))){

            if (newControlMessage.err) {
                if(!failsafeLastState) {
                    failsafeLastState = true;
                    failsafeMode(true);
                }
            }
            else{
                if (failsafeLastState) {
                    failsafeLastState = false;
                    failsafeMode(false);
                }

                switch(newControlMessage.s1) {
                    case SW_POS_1:
                        if(statusDrone.motorArmed){
                            statusDrone.motorArmed = false;
                            armedMotors(false);
                            statusDrone.outputControl.motorL = 0;
                            statusDrone.outputControl.motorR = 0;
                        }
                        statusDrone.flyMode = FLY_MODE_ATTI;
                    break;
                    case SW_POS_2:
                        if (!statusDrone.motorArmed) {
                            statusDrone.motorArmed = true;
                            armedMotors(true);
                            statusDrone.outputControl.motorL = VEL_MOTORS_ARMED;
                            statusDrone.outputControl.motorR = VEL_MOTORS_ARMED;
                        } 
                        statusDrone.flyMode = FLY_MODE_ATTI;
                    break;
                    case SW_POS_3:
                        if (!statusDrone.motorArmed) {
                            statusDrone.motorArmed = true;
                            armedMotors(true);
                            statusDrone.outputControl.motorL = VEL_MOTORS_ARMED;
                            statusDrone.outputControl.motorR = VEL_MOTORS_ARMED;
                        } 
                        statusDrone.flyMode = FLY_MODE_STABILIZED;
                    break;
                }
  
                switch(newControlMessage.s2){
                    case SW_POS_1:
                        statusDrone.rcControl.dualRates = DUAL_RATES_SOFT;
                    break;
                    case SW_POS_2:
                        statusDrone.rcControl.dualRates = DUAL_RATES_MEDIUM;
                    break;
                    case SW_POS_3:
                        statusDrone.rcControl.dualRates = DUAL_RATES_HARD;
                    break;
                }

                // Lo normalizo para que vaya de 0 a 1 en float considerando el dual rates
                statusDrone.rcControl.elevator = statusDrone.rcControl.dualRates * ((newControlMessage.elevator - 50) / 50.00);
                statusDrone.rcControl.aileron = statusDrone.rcControl.dualRates * ((newControlMessage.aileron - 50) / 50.00);
                statusDrone.rcControl.rudder = statusDrone.rcControl.dualRates * ((newControlMessage.rudder - 50) / 50.00);
                statusDrone.rcControl.throttle = (newControlMessage.throttle - 50) / 100.00;

                float targetElevator = statusDrone.rcControl.elevator * MAX_ANGLE_CONTROL;
                float targetAileron = statusDrone.rcControl.aileron * MAX_ANGLE_CONTROL;
                // float targetRudder = statusDrone.rcControl.rudder * MAX_ANGLE_CONTROL;
                pidSetSetPoint(PID_PITCH, targetElevator);
                printf(">setpoint pitch: %f\n", targetElevator);
                pidSetSetPoint(PID_ROLL, targetAileron);

                // ESP_LOGI("attitudeControl", "elevator: %f, targetElevator: %f", statusDrone.rcControl.elevator, targetElevator);
                // pidSetSetPoint(PID_YAW, (statusDrone.rcControl.elevator -50) * (statusDrone.dualRates/100.00));
            }
        }
    }
}

void genericError() {

    statusLedUpdate(STATUS_LED_ERROR);

    while(true) {
        printf("FATAL GENERIC ERROR\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void initializeTest(void) {
    uint8_t posTest;
    uint8_t limitMax = DUAL_RATES_HARD, limitMin = 100-DUAL_RATES_HARD;

    pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,50);
    pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,50);
    vTaskDelay(pdMS_TO_TICKS(200));

    for(posTest = limitMin; posTest < limitMax; posTest++){
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,posTest);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,posTest);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for(posTest = limitMax; posTest > limitMin; posTest--){
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,posTest);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,posTest);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for(posTest = 0; posTest < 2; posTest++){
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L, 500);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R, 500);
        vTaskDelay(pdMS_TO_TICKS(200));

        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,limitMin * 10.00);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,limitMax * 10.00);
        vTaskDelay(pdMS_TO_TICKS(200));

        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,limitMax * 10.00);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,limitMin * 10.00);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    pwmSetOutput(OUTPUT_CHANNEL_SERVO_L, 500);
    pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,500);
}

void app_main() {
    const char *TAG = "app_main";
    pid_settings_comms_t readParams={0};

    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    mpu6050QueueHandler = xQueueCreate(1, sizeof(vector_queue_t));
    queueNewSBUS = xQueueCreate(1,sizeof(channels_control_t));
    queueReceiveControl = xQueueCreate(1, sizeof(velocity_command_t));
    newPidParamsQueue = xQueueCreate(1, sizeof(pid_settings_comms_t));
    queueMotorsPosition = xQueueCreate(5, sizeof(as5600_queue_data_t));

    statusLedInit(GPIO_LED_STATUS);

    pwm_servo_init_t configServo = {
        .hsPwm1Gpio = GPIO_MOTOR_L,
        .hsPwm2Gpio = GPIO_MOTOR_R,
        .lsPwm1Gpio = GPIO_SERVO_L,
        .lsPwm2Gpio = GPIO_SERVO_R,
        .lsPwm3Gpio = GPIO_LED_MOTOR_L,
        .lsPwm4Gpio = GPIO_LED_MOTOR_R
    };
    pwmServoInit(configServo);

    for (uint8_t channel=0; channel<6; channel++) {
        pwmSetOutput(channel,failsafePosChannels[channel] * 10.00);
    }

    as5600_config_t config = {
        .freq = AS5600_PWM_230HZ,
        .gpioInIzq =  GPIO_AS5600_IZQ_IN,
        .gpioSdaIzq = GPIO_AS5600_IZQ_SDA,  
        .gpioSclIzq =  GPIO_AS5600_IZQ_SCL,   
        .gpioInDer = GPIO_AS5600_DER_IN,   
        .gpioSclDer = GPIO_AS5600_DER_SCL,    
        .gpioSdaDer = GPIO_AS5600_DER_SDA,  
        .queue = queueMotorsPosition,
        .core = AS5600_HANDLER_CORE,
        .priority = AS5600_HANDLER_PRIORITY
    };
    as5600Init(&config);
    
    sbusInit(UART_SBUS_NUM,GPIO_SBUS_TX,GPIO_SBUS_RX);
    storageInit();
    
    // btInit( DEVICE_BT_NAME );
    statusLedUpdate(STATUS_LED_CALIBRATION_IMU);

    mpu6050_init_t configMpu = {
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .intGpio = GPIO_MPU_INT,
        .priorityTask = IMU_HANDLER_PRIORITY,
        .core = IMU_HANDLER_CORE
    };
    
    mpu6050_initialize(&configMpu);

    statusDrone.localConfig.pids[PID_PITCH].kp = 1.0;
    statusDrone.localConfig.pids[PID_PITCH].ki = 0.0;
    statusDrone.localConfig.pids[PID_PITCH].kd = 0.0;
    statusDrone.localConfig.pids[PID_PITCH].setPoint = 0.0;

    statusDrone.localConfig.pids[PID_ROLL].kp = 1.0;
    statusDrone.localConfig.pids[PID_ROLL].ki = 0.2;
    statusDrone.localConfig.pids[PID_ROLL].kd = 0.3;
    statusDrone.localConfig.pids[PID_ROLL].setPoint = 0.0;

    statusDrone.localConfig.pids[PID_YAW].kp = 1.0;
    statusDrone.localConfig.pids[PID_YAW].ki = 0.0;
    statusDrone.localConfig.pids[PID_YAW].kd = 0.0;
    statusDrone.localConfig.pids[PID_YAW].setPoint = 0.0; // TODO: setear con el valor detectado al iniciar

    statusDrone.localConfig.safetyLimits = 45;

    ESP_LOGI(TAG, "\n------------------- local config -------------------"); 
    ESP_LOGI(TAG, "safetyLimits: %.02f",statusDrone.localConfig.safetyLimits);
    
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        ESP_LOGI(TAG,"PID %d Params: kp: %.02f\tki: %.02f\tkd: %.02f\tsetPoint: %.02f",i,statusDrone.localConfig.pids[i].kp, statusDrone.localConfig.pids[i].ki, statusDrone.localConfig.pids[i].kd, statusDrone.localConfig.pids[i].setPoint);
    }
    ESP_LOGI(TAG, "\n------------------- local config -------------------\n"); 

    drone_control_t initDroneState = { .motorL = 0, .motorR = 0, .servoL = 50, .servoR = 50 };
    statusDrone.outputControl = initDroneState;      // Estado inicial, posicion motores 0%, servos 50%, dualrates 50%, armed false,flymode ATTI, failsafe false

    pid_init_t pidConfig;
    pidConfig.pids[PID_PITCH] = convertPidFloatToStruct(statusDrone.localConfig.pids[PID_PITCH] ,PERIOD_PID_PRIMARY_MS);
    pidConfig.pids[PID_ROLL] = convertPidFloatToStruct(statusDrone.localConfig.pids[PID_ROLL] ,PERIOD_PID_SECONDARY_MS);
    pidConfig.pids[PID_YAW] = convertPidFloatToStruct(statusDrone.localConfig.pids[PID_YAW] ,PERIOD_PID_SECONDARY_MS);
    pidInit(pidConfig);

    pidSetEnable(PID_PITCH);
    pidSetEnable(PID_ROLL);

    initializeTest();

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    // xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",4096,NULL,4,NULL);

    statusLedUpdate(STATUS_LED_WAITING_ARM);

    // gpsUbxInit(UART_GPS_NUM,BAUDRATE_GPS_UBX,GPIO_GPS_RX,GPIO_GPS_TX);

    // while(1){
    //     if(btIsConnected()){
    //         cont1++;
    //         if(cont1>50){
    //             cont1 =0;
    //         }
    //         statusDrone.header = HEADER_COMMS;
    //         statusDrone.bat_voltage = 10;
    //         statusDrone.bat_percent = 55;
    //         statusDrone.batTemp = 100-cont1;
    //         statusDrone.temp_uc_control = cont1;
    //         statusDrone.temp_uc_main = 123-cont1; 
    //         // statusDrone.status_code = 0;
    //         sendStatus(statusDrone);
    //     }
    //     gpio_set_level(PIN_LED,1);
        // vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(PIN_LED,0);
        // vTaskDelay(pdMS_TO_TICKS(50));
    // }
}