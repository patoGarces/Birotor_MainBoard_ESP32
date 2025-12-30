#include "../components/MPU6050/MPU6050_6Axis_MotionApps20.h"
// #include "../components/MPU6050/MPU6050_9Axis_MotionApps41.h"

#include "mpu6050_wrapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

QueueHandle_t mpu6050QueueHandler;

TaskHandle_t readHandler;
mpu6050_init_t MpuConfigInit;
uint8_t enableCalibrate = false;

void mpu6050Handler(void*){
	// quaternion_wrapper_t q;     		// [w, x, y, z]         quaternion container
	// vector_float_wrapper_t gravity;	// [x, y, z]            gravity vector
	Quaternion q;         				// [w, x, y, z]         quaternion container
	VectorFloat gravity;    			// [x, y, z]            gravity vector
	float ypr[3];                       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	uint16_t packetSize = 42;           // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;                 // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64];             // FIFO storage buffer
	uint8_t mpuIntStatus;               // holds actual interrupt status byte from MPU
	uint16_t contMeasure = 0;

	MPU6050 mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();

	// This need to be setup individually
	// mpu.setXGyroOffset(220);
	// mpu.setYGyroOffset(76);
	// mpu.setZGyroOffset(-85);
	// mpu.setZAccelOffset(1788);

	if (enableCalibrate) {
		// mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
		enableCalibrate = false;
	}

	mpu.setDMPEnabled(true);

	while(1){
	    mpuIntStatus = mpu.getIntStatus();
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO
	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			vector_queue_t newData;
			for (uint8_t i=0;i<3;i++) {
				newData.angles[i] = ((ypr[i] * 180) / (float)M_PI);
			};
			newData.temp = ((mpu.getTemperature() / 340.0f) + 36.53f);

			if (contMeasure < 1000) { // 2500) {						// wait to stabilize measuments
				contMeasure++;
			}
			else {
            	xQueueSend(mpu6050QueueHandler,(void *) &newData, 1);
			}
	    }

	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		// vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void mpu6050_initialize(mpu6050_init_t *config) {
    
	MpuConfigInit = *config;

    i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = config->sdaGpio;
	conf.scl_io_num = config->sclGpio;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM));

    xTaskCreatePinnedToCore(mpu6050Handler,"mpu6050_handler_wrapper",8096,NULL,config->priorityTask,&readHandler,config->core);
}

void mpu6050_recalibrate() {
	// vTaskDelete(readHandler);
	// enableCalibrate = true;
	// i2c_driver_delete(I2C_NUM_0);
	// mpu6050_initialize(&MpuConfigInit);
}

// int mpu6050_testConnection() {
//     return mpu.testConnection() ? 1 : 0;
// }

#ifdef __cplusplus
}
#endif
