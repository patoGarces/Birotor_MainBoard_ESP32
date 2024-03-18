#include "stdio.h"
#include "main.h"
#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "driver/uart.h"

#include "math.h"

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/SBUS_COMMS/include/SBUS_COMMS.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"
#include "../components/SERVO_CONTROL/include/SERVO_CONTROL.h"
#include "../components/GPS_UBX/include/GPS_UBX.h"
#include "../components/WS2812/include/WS2812.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000
#define VEL_MAX_CONTROL         5    
#define DEVICE_BT_NAME          "Birotor Drone"

#define VEL_MOTORS_ARMED        10

#define CH_AIL_GAIN             0.5 
#define CH_RUD_GAIN             1 
#define CH_ELEV_GAIN            1 

#define DUAL_RATES_SOFT         20
#define DUAL_RATES_MEDIUM       40
#define DUAL_RATES_HARD         65

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
extern QueueHandle_t queueNewSBUS;
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;
status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app
drone_control_t droneControl = {0,0,50,50,50,0,0,0};      // Estado inicial, posicion motores 0%, servos 50%, dualrates 50%, armed false,flymode ATTI, failsafe false

rc_channels_t rcChannels = {
    50,
    50,
    50,
    50
};
const rc_channels_t failsafeChannels = {
    .aileron = 50,
    .elevator = 50,
    .rudder = 50,
};
const uint8_t failsafePosChannels[6] = { 0,0,50,50,50,50 };


uint8_t cutRangeExceed(int16_t value,uint8_t min,uint8_t max){
    if( value > max){
        return max;
    }
    if( value < min ){
        return min;
    }
    return value;
}

void armedMotors(uint8_t armed) {
    droneControl.motorArmed = armed;
    if (armed && droneControl.flyMode == FLY_MODE_ATTI) {
        statusLedUpdate(STATUS_LED_ARMED_ATTI);
    }
    else if (armed && droneControl.flyMode == FLY_MODE_STABILIZED) {
        statusLedUpdate(STATUS_LED_ARMED_STABILIZED);
    }
    else {
        statusLedUpdate(STATUS_LED_WAITING_ARM);
    }
}

void failsafeMode(uint8_t failsafe) {

    droneControl.failsafe = failsafe;

    if (failsafe) {
        droneControl.flyMode = FLY_MODE_FAILSAFE;
        statusLedUpdate(STATUS_LED_FAILSAFE);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,failsafePosChannels[OUTPUT_CHANNEL_SERVO_L]);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,failsafePosChannels[OUTPUT_CHANNEL_SERVO_R]);
        printf("FailsafeMode activate\n");
    }
    else {
        armedMotors(droneControl.motorArmed);
        printf("FailsafeMode desactivate, armed: %d\n",droneControl.motorArmed);
    }
}

static void imuControlHandler(void *pvParameters){
    float newAngles[3];
    float outputPidElevator = 0.00;
    rc_channels_t controlOutput;
    uint8_t cont = 0,toggle=0;

    while(1){

        if(xQueueReceive(newAnglesQueue,&newAngles,pdMS_TO_TICKS(10))){     // TODO: ESTA TODO sincronizado a partir del MPU, ESTO ESTA MAL
            if (pidGetEnable()) {//} && droneControl.flyMode == FLY_MODE_STABILIZED) {   // TODO: descomentar

                // statusToSend.pitch = newAngles[AXIS_ANGLE_Y];
                // statusToSend.roll = newAngles[AXIS_ANGLE_X];
                // statusToSend.yaw = newAngles[AXIS_ANGLE_Z];
                pidSetSetPoint((rcChannels.elevator -50) * (droneControl.dualRates/100.00));

                float roundedAngle = roundf(newAngles[AXIS_ANGLE_X] * 100) / 100; 
                float outputPidRaw = pidCalculate(roundedAngle);
                outputPidElevator = ((outputPidRaw*-0.50) + 0.5) * 100.00; 

                cont++;
                if(cont>50) {
                    // Envio log para graficar en arduino serial plotter
                    printf("angle_x:%f,output_pid_raw:%f,output_pid:%f\n",roundedAngle,outputPidRaw,outputPidElevator);
                    cont=0;
                }

                controlOutput = rcChannels;
                controlOutput.elevator = outputPidElevator;

                // if(GRAPH_ARDUINO_PLOTTER){
                    //Envio log para graficar en arduino serial plotter
                    // printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
                // }
                // gpio_set_level(PIN_OSCILO, 0);

                // cont++;
                // if(cont>50){
                //     printf("angle:%f,set_point: %f,output_pid: %f\n",newAngles[AXIS_ANGLE_X],pidGetPointAngle(),outputPid);
                //     cont=0;
                // }
            }
            else if (droneControl.flyMode == FLY_MODE_ATTI) {
                controlOutput = rcChannels;
            }
            else { // if (droneControl.flyMode == FLY_MODE_FAILSAFE) {
                controlOutput = failsafeChannels;
            }

            int16_t mixedServoL = (int8_t)(controlOutput.elevator-50)*CH_ELEV_GAIN * (droneControl.dualRates/100.00) + (int8_t)(controlOutput.rudder-50)*CH_RUD_GAIN * (droneControl.dualRates/100.00) + 50;
            int16_t mixedServoR = -(int8_t)(controlOutput.elevator-50)*CH_ELEV_GAIN * (droneControl.dualRates/100.00) + (int8_t)(controlOutput.rudder-50)*CH_RUD_GAIN * (droneControl.dualRates/100.00) + 50;
            
            droneControl.servoL = cutRangeExceed(mixedServoL,0,100);           // TODO: inicializar dualrates por seguridad
            droneControl.servoR = cutRangeExceed(mixedServoR,0,100);

            pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,droneControl.servoL);
            pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,droneControl.servoR);

            if (droneControl.motorArmed){
                if( controlOutput.throttle >= 50){
                    int16_t mixedMotorL = (int8_t)(controlOutput.throttle-50) + (int8_t)(controlOutput.aileron-50)*CH_AIL_GAIN * (droneControl.dualRates/100.00) + 50;
                    int16_t mixedMotorR = (int8_t)(controlOutput.throttle-50) - (int8_t)(controlOutput.aileron-50)*CH_AIL_GAIN * (droneControl.dualRates/100.00) + 50;
                    mixedMotorL = cutRangeExceed(mixedMotorL,0,100);
                    mixedMotorR = cutRangeExceed(mixedMotorR,0,100);

                    droneControl.motorL = cutRangeExceed(VEL_MOTORS_ARMED + (((mixedMotorL - 50) *90.00) / 50.00),10,100);
                    droneControl.motorR = cutRangeExceed(VEL_MOTORS_ARMED + (((mixedMotorR - 50) *90.00) / 50.00),10,100);
                }
            }
            pwmSetOutput(OUTPUT_CHANNEL_MOT_L,droneControl.motorL);
            pwmSetOutput(OUTPUT_CHANNEL_MOT_R,droneControl.motorR);
            // ESP_LOGE("attitudeControl","thr: %d, servoL: %d, servoR: %d, motorL: %d, motorR: %d, dualRates: %d",newControlMessage.throttle,droneControl.servoL,droneControl.servoR,droneControl.motorL,droneControl.motorR,droneControl.dualRates);
        }
    }
}

static void updateParams(void *pvParameters){

    pid_params_t newPidParams;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));

    while (1){
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,pdMS_TO_TICKS(10))){
            newPidParams.center_angle += CENTER_ANGLE_MOUNTED;
            pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
            pidSetSetPoint(newPidParams.center_angle);
            storageWritePidParams(newPidParams);            

            statusToSend.P = newPidParams.kp*100;  
            statusToSend.I = newPidParams.ki*100;  
            statusToSend.D = newPidParams.kd*100; 
            statusToSend.centerAngle = newPidParams.center_angle;   
            statusToSend.safetyLimits = newPidParams.safety_limits;              
        }
    }
}

static void attitudeControl(void *pvParameters){
    uint8_t failsafeLastState = false;
    queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));
    channels_control_t newControlMessage;

    while(true){
        if( xQueueReceive(queueNewSBUS,&newControlMessage, pdMS_TO_TICKS(10))){

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
                        if(droneControl.motorArmed){
                            droneControl.motorArmed = false;
                            armedMotors(false);
                            droneControl.motorL = 0;
                            droneControl.motorR = 0;
                        }
                        droneControl.flyMode = FLY_MODE_ATTI;
                    break;
                    case SW_POS_2:
                        if (!droneControl.motorArmed) {
                            droneControl.motorArmed = true;
                            armedMotors(true);
                            droneControl.motorL = VEL_MOTORS_ARMED;
                            droneControl.motorR = VEL_MOTORS_ARMED;
                        } 
                        droneControl.flyMode = FLY_MODE_ATTI;
                    break;
                    case SW_POS_3:
                        if (!droneControl.motorArmed) {
                            droneControl.motorArmed = true;
                            armedMotors(true);
                            droneControl.motorL = VEL_MOTORS_ARMED;
                            droneControl.motorR = VEL_MOTORS_ARMED;
                        } 
                        droneControl.flyMode = FLY_MODE_STABILIZED;
                    break;
                }
  
                switch(newControlMessage.s2){
                    case SW_POS_1:
                        droneControl.dualRates = DUAL_RATES_SOFT;
                    break;
                    case SW_POS_2:
                        droneControl.dualRates = DUAL_RATES_MEDIUM;
                    break;
                    case SW_POS_3:
                        droneControl.dualRates = DUAL_RATES_HARD;
                    break;
                }

                rcChannels.elevator = newControlMessage.elevator;
                rcChannels.aileron = newControlMessage.aileron;
                rcChannels.rudder = newControlMessage.rudder;
                rcChannels.throttle = newControlMessage.throttle;
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

void initializeTest(void){
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
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,50);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,50);
        vTaskDelay(pdMS_TO_TICKS(200));

        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,limitMin);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,limitMax);
        vTaskDelay(pdMS_TO_TICKS(200));

        pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,limitMax);
        pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,limitMin);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    pwmSetOutput(OUTPUT_CHANNEL_SERVO_L,50);
    pwmSetOutput(OUTPUT_CHANNEL_SERVO_R,50);
}

void app_main() {
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    pwmServoInit(GPIO_MOTOR_L,GPIO_MOTOR_R,GPIO_SERVO_L,GPIO_SERVO_R,GPIO_LED_MOTOR_L,GPIO_LED_MOTOR_R);

    for(uint8_t channel=0;channel<6;channel++){
        pwmSetOutput(channel,failsafePosChannels[channel]);
    }

    statusLedInit(GPIO_LED_STATUS);
    sbusInit(UART_SBUS_NUM,GPIO_SBUS_TX,GPIO_SBUS_RX);
    storageInit();
    
    // btInit( DEVICE_BT_NAME );
    statusLedUpdate(STATUS_LED_CALIBRATION_IMU);

    mpu6050_init_t mpuConfig = {
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .intGpio = GPIO_MPU_INT,
        .sampleTimeInMs = PERIOD_IMU_MS,
        .accelSensitivity = MPU_ACCEL_SENS_2G,
        .gyroSensitivity = MPU_GYRO_SENS_250
    };
    
    if( mpuInit(mpuConfig) != ESP_OK) {
        genericError();
    }

    pid_params_t newPidParams = {
        .center_angle = 0.00,
        .kp = 2.00,
        .ki = 2.00,
        .kd = 0.2
    };
    storageWritePidParams(newPidParams);
    readParams = storageReadPidParams();
    printf("center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f\n",readParams.center_angle,readParams.kp,readParams.ki,readParams.kd,readParams.safety_limits);
    
    pid_init_t pidConfig = {
        .initSetPoint = readParams.center_angle,
        .kp = readParams.kp,
        .ki = readParams.ki,
        .kd = readParams.kd,
        .sampleTimeInMs = PERIOD_IMU_MS
    };
    ESP_ERROR_CHECK(pidInit(pidConfig));
    pidSetEnable();

    initializeTest();

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    // xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",4096,NULL,4,NULL);

    statusLedUpdate(STATUS_LED_WAITING_ARM);
    // gpsUbxInit(UART_GPS_NUM,BAUDRATE_GPS_UBX,GPIO_GPS_RX,GPIO_GPS_TX);

    while(1){
    //     if(btIsConnected()){
    //         cont1++;
    //         if(cont1>50){
    //             cont1 =0;
    //         }
    //         statusToSend.header = HEADER_COMMS;
    //         statusToSend.bat_voltage = 10;
    //         statusToSend.bat_percent = 55;
    //         statusToSend.batTemp = 100-cont1;
    //         statusToSend.temp_uc_control = cont1;
    //         statusToSend.temp_uc_main = 123-cont1; 
    //         // statusToSend.status_code = 0;
    //         sendStatus(statusToSend);
    //     }
    //     gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}