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
#define DUAL_RATES_HARD         80

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
extern QueueHandle_t queueNewSBUS;
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

drone_control_t droneControl;

uint8_t cutRangeExceed(int16_t value,uint8_t min,uint8_t max){
    if( value > max){
        return max;
    }
    if( value < min ){
        return min;
    }
    return value;
}

static void imuControlHandler(void *pvParameters){
    float newAngles[3];
    float outputPid;

    uint32_t cont = 0;

    while(1){

        if(xQueueReceive(newAnglesQueue,&newAngles,10)){

            // if( getEnablePid() ){
            //     gpio_set_level(PIN_OSCILO, 1);
            // }
            statusToSend.pitch = newAngles[AXIS_ANGLE_Y];
            statusToSend.roll = newAngles[AXIS_ANGLE_X];
            statusToSend.yaw = newAngles[AXIS_ANGLE_Z];

            outputPid = pidCalculate(newAngles[AXIS_ANGLE_Y]) *-1; 
            // outputMotors.motorL = outputPid * MAX_VELOCITY;
            // outputMotors.motorR = outputMotors.motorL;

            // statusToSend.speedL = outputMotors.motorL;
            // statusToSend.speedR = outputMotors.motorR;

            // Envio data a los motores
            // sendMotorData(outputMotors.motorR,outputMotors.motorL,0x00,0x00);            // TODO: controlar el enable

            if(!getEnablePid()){ 
                if((( newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED-1)) && ( newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED+1) )) ){ 
                    // enableMotors();
                    setEnablePid();   
                    statusToSend.status_code = STATUS_ROBOT_STABILIZED;                                                       
                }
                else{
                    // disableMotors();
                }
            }
            else{ 
                if((( newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED-statusToSend.safetyLimits)) || ( newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED+statusToSend.safetyLimits) )) ){ 
                    // disableMotors();
                    setDisablePid();
                    statusToSend.status_code = STATUS_ROBOT_DISABLE; 
                }
            }

            if(GRAPH_ARDUINO_PLOTTER){
                //Envio log para graficar en arduino serial plotter
                // printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
            }
            // gpio_set_level(PIN_OSCILO, 0);

            cont++;
            if(cont>10){
                // printf("angle:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
                cont=0;
            }
        }
    }
}

static void updateParams(void *pvParameters){

    pid_params_t newPidParams;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));

    while (1){
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,0)){
            newPidParams.center_angle += CENTER_ANGLE_MOUNTED;
            pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
            pidSetPointAngle(newPidParams.center_angle);
            storageWritePidParams(newPidParams);            

            statusToSend.P = newPidParams.kp*100;  
            statusToSend.I = newPidParams.ki*100;  
            statusToSend.D = newPidParams.kd*100; 
            statusToSend.centerAngle = newPidParams.center_angle;   
            statusToSend.safetyLimits = newPidParams.safety_limits;   // TODO: incluir para enviarlo                 
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void attitudeControl(void *pvParameters){

    queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));

    control_app_t newControlVal;
    channels_control_t newControlMessage;

    while(true){
        // if( xQueueReceive(queueReceiveControl,
        //                  &newControlVal,
        //                  ( TickType_t ) 1 ) == pdPASS ){

            // attitudeControlMotor.motorL = newControlVal.axis_x * -1 * VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            // attitudeControlMotor.motorR = newControlVal.axis_x *  VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            
            // // setVelMotors(attitudeControlMotor.motorL,attitudeControlMotor.motorR);

            // if( !attitudeControlMotor.motorL && !attitudeControlMotor.motorR){
            //     // disableMotors();
            // }
            // else{
            //     // enableMotors();
            // }

            // printf("CONTROL RECIBIDO: X: %ld, Y: %ld, motorL: %d ,motorR: %d \n",newControlVal.axis_x,newControlVal.axis_y,attitudeControlMotor.motorL,attitudeControlMotor.motorR);
        // } 

        if( xQueueReceive(queueNewSBUS,&newControlMessage, 0)){

            if (!droneControl.motorArmed && newControlMessage.s1 == SW_POS_2){
                droneControl.motorArmed = true;
                droneControl.motorL = VEL_MOTORS_ARMED;
                droneControl.motorR = VEL_MOTORS_ARMED;
            }
            else if(droneControl.motorArmed && newControlMessage.s1 != SW_POS_2){
                droneControl.motorArmed = false;
                droneControl.motorL = 0;
                droneControl.motorR = 0;
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

            int16_t mixedServoL = (int8_t)(newControlMessage.elevator-50)*CH_ELEV_GAIN * (droneControl.dualRates/100.00) + (int8_t)(newControlMessage.rudder-50)*CH_RUD_GAIN * (droneControl.dualRates/100.00) + 50;
            int16_t mixedServoR = -(int8_t)(newControlMessage.elevator-50)*CH_ELEV_GAIN * (droneControl.dualRates/100.00) + (int8_t)(newControlMessage.rudder-50)*CH_RUD_GAIN * (droneControl.dualRates/100.00) + 50;
            
            droneControl.servoL = cutRangeExceed(mixedServoL,0,100);           // TODO: inicializar dualrates por seguridad
            droneControl.servoR = cutRangeExceed(mixedServoR,0,100);

            setChannelOutput(OUTPUT_CHANNEL_SERVO_L,droneControl.servoL);
            setChannelOutput(OUTPUT_CHANNEL_SERVO_R,droneControl.servoR);

            if (droneControl.motorArmed){

                if( newControlMessage.throttle >= 50){
                    int16_t mixedMotorL = (int8_t)(newControlMessage.throttle-50) + (int8_t)(newControlMessage.aileron-50)*CH_AIL_GAIN * (droneControl.dualRates/100.00) + 50;
                    int16_t mixedMotorR = (int8_t)(newControlMessage.throttle-50) - (int8_t)(newControlMessage.aileron-50)*CH_AIL_GAIN * (droneControl.dualRates/100.00) + 50;
                    mixedMotorL = cutRangeExceed(mixedMotorL,0,100);
                    mixedMotorR = cutRangeExceed(mixedMotorR,0,100);

                    droneControl.motorL = cutRangeExceed(VEL_MOTORS_ARMED + (((mixedMotorL - 50) *90.00) / 50.00),10,100);
                    droneControl.motorR = cutRangeExceed(VEL_MOTORS_ARMED + (((mixedMotorR - 50) *90.00) / 50.00),10,100);
                }
            }

            setChannelOutput(OUTPUT_CHANNEL_MOT_L,droneControl.motorL);
            setChannelOutput(OUTPUT_CHANNEL_MOT_R,droneControl.motorR);
            ESP_LOGE("attitudeControl","thr: %d, servoL: %d, servoR: %d, motorL: %d, motorR: %d, dualRates: %d",newControlMessage.throttle,droneControl.servoL,droneControl.servoR,droneControl.motorL,droneControl.motorR,droneControl.dualRates);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void initializeTest(void){
    uint8_t posTest;

    setChannelOutput(OUTPUT_CHANNEL_SERVO_L,50);
    setChannelOutput(OUTPUT_CHANNEL_SERVO_R,50);
    vTaskDelay(pdMS_TO_TICKS(200));

    for(posTest = 0; posTest < 100; posTest++){
        setChannelOutput(OUTPUT_CHANNEL_SERVO_L,posTest);
        setChannelOutput(OUTPUT_CHANNEL_SERVO_R,posTest);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for(posTest = 100; posTest > 0; posTest--){
        setChannelOutput(OUTPUT_CHANNEL_SERVO_L,posTest);
        setChannelOutput(OUTPUT_CHANNEL_SERVO_R,posTest);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for(posTest = 0; posTest < 2; posTest++){
        setChannelOutput(OUTPUT_CHANNEL_SERVO_L,50);
        setChannelOutput(OUTPUT_CHANNEL_SERVO_R,50);
        vTaskDelay(pdMS_TO_TICKS(200));

        setChannelOutput(OUTPUT_CHANNEL_SERVO_L,0);
        setChannelOutput(OUTPUT_CHANNEL_SERVO_R,100);
        vTaskDelay(pdMS_TO_TICKS(200));

        setChannelOutput(OUTPUT_CHANNEL_SERVO_L,100);
        setChannelOutput(OUTPUT_CHANNEL_SERVO_R,0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    setChannelOutput(OUTPUT_CHANNEL_SERVO_L,50);
    setChannelOutput(OUTPUT_CHANNEL_SERVO_R,50);
}

void app_main() {
    uint8_t cont1=0;
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    storageInit();
    statusLedInit(GPIO_LED_STATUS);

    // btInit( DEVICE_BT_NAME );
    // mpu_init();
    // readParams = storageReadPidParams();
    // printf("center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f\n",readParams.center_angle,readParams.kp,readParams.ki,readParams.kd,readParams.safety_limits);
    // pidInit(readParams);

    pwmServoInit(GPIO_MOTOR_L,GPIO_MOTOR_R,GPIO_SERVO_L,GPIO_SERVO_R,GPIO_LED_MOTOR_L,GPIO_LED_MOTOR_R);
    sbusInit(UART_SBUS_NUM,GPIO_SBUS_TX,GPIO_SBUS_RX);
    setChannelOutput(2,50);

    initializeTest();

    // xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    // xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);

    // gpsUbxInit(UART_GPS_NUM,BAUDRATE_GPS_UBX,GPIO_GPS_RX,GPIO_GPS_TX);
    
    statusLedUpdate(STATUS_LED_CALIBRATION_IMU);

    // while(true) {
    //     for(uint8_t i=0;i<6;i++) {
    //         printf("StatusLed: %d\n",i);
    //         statusLedUpdate(i);
    //         vTaskDelay(pdMS_TO_TICKS(3000));
    //     }
    // }

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
    //     vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}