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

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/SBUS_COMMS/include/SBUS_COMMS.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"
#include "../components/SERVO_CONTROL/include/SERVO_CONTROL.h"


#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000
#define VEL_MAX_CONTROL         5    
#define DEVICE_BT_NAME          "Birotor Drone"

#define VEL_MOTORS_ARMED        10

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
extern QueueHandle_t queueNewSBUS;
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

drone_status_t droneStatus;

uint8_t cutRangeExceed(int16_t value){
    if( value > 100){
        return 100;
    }
    if( value < 0 ){
        return 0;
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

            int16_t outputServoL = (int8_t)(newControlMessage.elevator-50) + (int8_t)(newControlMessage.aileron-50) + 50;
            int16_t outputServoR = -(int8_t)(newControlMessage.elevator-50) + (int8_t)(newControlMessage.aileron-50) + 50;
            
            droneStatus.servoL = cutRangeExceed(outputServoL);
            droneStatus.servoR = cutRangeExceed(outputServoR);

            setChannelOutput(OUTPUT_INDEX_SERVO_L,droneStatus.servoL);
            setChannelOutput(OUTPUT_INDEX_SERVO_R,droneStatus.servoR);

            if (droneStatus.motorArmed){

                if( newControlMessage.throttle >= 50){
                    droneStatus.throttleOutput = VEL_MOTORS_ARMED + (((newControlMessage.throttle - 50) *90.00) / 50.00);

                    // ESP_LOGE("attitudeControl","throttle: %d",droneStatus.throttleOutput);

                    // droneStatus.throttleOutput = cutRangeExceed(droneStatus.throttleOutput);
                    // droneStatus.motorR = cutRangeExceed(droneStatus.throttleOutput);
                    droneStatus.motorR = droneStatus.throttleOutput;
                    droneStatus.motorL = droneStatus.throttleOutput;
                }
                // else{
                //     outputMotors = VEL_MOTORS_ARMED;
                // }
            }

            if (!droneStatus.motorArmed && newControlMessage.s1 == SW_POS_2){
                droneStatus.motorArmed = true;
                droneStatus.motorL = VEL_MOTORS_ARMED;
                droneStatus.motorR = VEL_MOTORS_ARMED;
                droneStatus.throttleOutput = VEL_MOTORS_ARMED;
            }
            else if(droneStatus.motorArmed && newControlMessage.s1 != SW_POS_2){
                droneStatus.motorArmed = false;
                droneStatus.motorL = 0;
                droneStatus.motorR = 0;
                droneStatus.throttleOutput = 0;
            }

            setChannelOutput(OUTPUT_INDEX_MOT_L,droneStatus.motorL);
            setChannelOutput(OUTPUT_INDEX_MOT_R,droneStatus.motorR);
            ESP_LOGE("attitudeControl","throttle: %d, servoL: %d, servoR: %d, motorL: %d, motorR: %d",newControlMessage.throttle,droneStatus.servoL,droneStatus.servoR,droneStatus.motorL,droneStatus.motorR);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
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

    // btInit( DEVICE_BT_NAME );
    // mpu_init();
    // readParams = storageReadPidParams();
    // printf("center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f\n",readParams.center_angle,readParams.kp,readParams.ki,readParams.kd,readParams.safety_limits);
    // pidInit(readParams);

    pwmServoInit();
    sbusInit();
    setChannelOutput(2,50);

    // xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    // xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);

    while(true){
        vTaskDelay(20);
    }

    // while(1){
        
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
    //     vTaskDelay(pdMS_TO_TICKS(50));
    // }
}