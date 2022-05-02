/*====================================================================================
Filename:	MPU6050.hpp
Version	:	Draft A
Date	: 	21-March-2022
Author	:	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION

This sketch is a version of MPU6050raw.ino that has been converted into an RTOS compatible source code module for
AudioNav3.  This is based on the sketch by the great Jeff Rowberg <jeff@rowberg.net>

It includes the latest calibration software from Jeff Rowberg, generating a fresh set
of accelerometer and gyroscope offsets each time the software starts up.

It utilises the on-board Digital Motion Processor to produce the Yaw/Pitch/Roll data
from quaternions.

The Inertial Measurement Unit(IMU) is the MPU-6050 chip on a GY-521 breakout board.

Six degrees of freedom:
	3-axis gyroscope, measuring rotational velocity in degrees/s
	3-axis accelerometer
	x-y axes are in the horizontal plane, z-axis is vertical pointing down
    This is a right-handed coordinate frame (NED - North, East, Down)

Note that if yaw is tracked solely using the gyroscope then the result will be
subject to significant drift.

CONNECTIONS to ESP32
  MPU6250   Description				ESP32 			    NOTES
	Vcc		5V						5V pin
	Gnd		Ground					Gnd
	SCL		I2C data 				GPIO 21		RX for config. commands from ESP32
	SDA		I2C clock  				GPIO 22		TX for NMEA sentences to the ESP32
	XDA		I2C aux data o/p		-			Not used
	XCL		I2C aux clock o/p		-			Not used
	ADO		Address switch			5V			HIGH:addr = 0x69; LOW: addr = 0x68
	INT		HW interrupt			-			Not used currently
yaw
*************************************************************************************/
// #include guard
#ifndef MPU6050_HPP
#define MPU6050_HPP
//-----------------------------------------------------------------------------------
#include "AudioNav3.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"  // Latest library s/w from Jeff Rowberg
#include "WiFiAP.hpp"

#define IMU_DEBUG  				0   // Logging for MPU6050 IMU (Level 1 or 2)
#define PRINT_YPR				0	// Print Yaw, Pitch and Roll

#define DLPF_MODE				1  	// 1= highest b/w; 6 = lowest b/w

#define RADIANS_TO_DEGREES		57.296		// Conversion factor

uint8_t devAddr = 0x69; // 0x68 is already used by the MPU-9250 (on same I2C bus)

static TimerHandle_t MPU6050Timer = NULL;

MPU6050 mpu(devAddr);  		// Instantiate an MPU6050 object


// Set up Data Ready interrupt for the MPU6050
#define DR_INTERRUPT_PIN   15

xSemaphoreHandle mpuDataReady = 0;	// Binary semaphore handle for ISR dataReady flag

bool dmpReady = false;	// Set true if Digital Motion Processor initialised OK
uint8_t mpuIntStatus;	// Holds actual interrupt status byte from MPU
uint8_t devStatus;		// Returns status after each device operation (0 = success)
uint16_t packetSize;  	// Expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[1120];
uint16_t fifoCount;		// number of bytes currently in FIFO buffer


// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void IRAM_ATTR isrDataReady(){
	xSemaphoreGiveFromISR(mpuDataReady, NULL);
}

static const uint8_t YawQueueLength = 5;
static QueueHandle_t YawQueue = NULL;

// ===========  Function to configure and calibrate the MPU6050 IMU   ==============
void MPU6050_Setup(){
	uint8_t mpuStatus;
	debug("IMU","Setting up the the MPU6050 Inertial Measurement Unit...", true );
	// Check I2C connection to MPU6050
	mpuStatus = mpu.testConnection();  // Is MPU6050 responding correctly on I2C bus
	vTaskDelay(100 / portTICK_PERIOD_MS);
	if (IMU_DEBUG>0) {
		debug("IMU","Testing connection to MPU6050... ", false);
		Serial.println(mpuStatus ? "connection SUCCESSFUL" : "connection FAILED!!!!!");
	}

	mpu.initialize(); //Activates MPU6050; sets accel & gyro ranges to max sensitivity

	if (IMU_DEBUG>0) {// Print out confirmation of sensor sensitivity settings
		// gets should both return zero if on maximum sensitivity
		debug("IMU","Accelerometer Range = ", false);
		Serial.println(mpu.getFullScaleAccelRange() ? "ERROR" : "+/- 2g");
		debug("IMU","Gyroscope Range    = ", false);
		Serial.println(mpu.getFullScaleGyroRange() ? "ERROR" : "+/- 250 deg/s");
	}

	// Set up temperature sensor on MPU6050
	mpu.resetTemperaturePath();
	mpu.setTempSensorEnabled(true);
	mpu.setTempFIFOEnabled(true);
	if (IMU_DEBUG>0) {// Print out confirmation that temperature sensor is enabled
		debug("IMU", "Temperature FIFO enabled ", false);
		Serial.println(mpu.getTempFIFOEnabled() ? "YES" : "NO");
	}

	// Set the DIGITAL LOW PASS FILTER mode (range 1-6; 0 = OFF)
	mpu.setDLPFMode(DLPF_MODE); // ON range is 1->6;  Reduces max gyro event rate to 1KHz
	if (IMU_DEBUG>0) {// Print out confirmation of DLPF setting
		debug("IMU", "DIGITAL LOW PASS FILTER mode is set to", false);
		Serial.println(mpu.getDLPFMode());
	}

	// Load and configure the Digital Motion Processor (DMP)
	devStatus = mpu.dmpInitialize();
	if (IMU_DEBUG>0) {// Print out the current accelerometer & gyroscope offsets
		debug("IMU", "Initialise the Digital Motion Processor.  Status is", false);
		Serial.print(devStatus ?  "DMP FAILED\n" : "SUCCESS\n");  //Success = 0
	}

	// Calibrate the MPU6050, generate and store Accel and Gyro offsets
	debug("IMU", "Calibrating MPU6050 and generating Accel & Gyro offsets", false);
	if (devStatus == 0) {
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
	Serial.println();
	if (IMU_DEBUG>0) {// Print out the current accelerometer & gyroscope offsets
		debug("IMU", "Active Accel & Gryo offsets are as follows", false);
		mpu.PrintActiveOffsets();
	}

	// ENABLE the Digital Motion Processor (DMP)
	mpu.setDMPEnabled(true);
	dmpReady = mpu.getDMPEnabled();
	packetSize = mpu.dmpGetFIFOPacketSize();
	if (IMU_DEBUG>0) {// Print out dmpEnabled status (boolean)
		debug("IMU", "Enable the Digital Motion Processor.      Status is", false);
		Serial.println(dmpReady ?  "DMP READY" : "DMP IS NOT READY!!!");
	}

	// Set up for MPU-6050 DATA READY interrupt
	mpu.setIntDataReadyEnabled(true);  // Enable DataReady interrupts
	uint8_t intRegister = mpu.getIntEnabled();
	if (IMU_DEBUG>0) {// Print out interrupt enabled status
		debug("IMU", "Enable the Data Ready Interrupt. IntEnabled register value is", false);
		Serial.printf("0x%02x \n", intRegister);
	}

	mpuDataReady = xSemaphoreCreateBinary();  //Create binary semaphore for use by ISR
	pinMode(DR_INTERRUPT_PIN, INPUT);
	attachInterrupt(DR_INTERRUPT_PIN, isrDataReady, RISING);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	//Create the queue to send Yaw to the CoreTask
	YawQueue = xQueueCreate(YawQueueLength, sizeof(float));
	if(YawQueue != NULL)
			debug("IMU","YawQueue created successfully",true);
		else
			errorReport("IMU",201 ,"Failed to create YawQueue", true);
	vTaskDelay(500 / portTICK_PERIOD_MS);  // Settling time
} // end of MPU6050_Setup




#define YPR_SAMPLE_DIVIDER	10
static uint16_t yprCount = 0;

// RTOS TASK to get Yaw, Pitch & Roll periodically
void getYawPitchRoll(void *parameter) {
	// The DO ONCE section of the TASK
	MPU6050_Setup();


	// The DO FOREVER loop
	// Read the data from the MPU6050 DMP and forward the YAW to the Navigate TASK via the YawQueue
	while(1){
		if(xSemaphoreTake(mpuDataReady,9999)) {
			if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
				yprCount++;
				//Serial.println(yprCount);
				if (yprCount%YPR_SAMPLE_DIVIDER == 0){  // only forward every tenth reading
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
					float yaw = ypr[0] * RADIANS_TO_DEGREES;
					if (xQueueSend(YawQueue, (void *)&yaw , 10) != pdTRUE){
						debug("IMU","YawQueue is FULL!",true);
					}
					if (PRINT_YPR){
						debug("IMU","Y P R (deg)", false);
						Serial.printf("  %+3.2f     %+3.2f     %+3.2f\n",
								ypr[0] * RADIANS_TO_DEGREES,
								ypr[1] * RADIANS_TO_DEGREES,
								ypr[2] * RADIANS_TO_DEGREES);
					}
				}
			}
		}
	}
}


//------------------------------------------------------------------------------------
#endif
