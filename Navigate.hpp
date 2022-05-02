/*====================================================================================
Filename:	Navigate.hpp
Version : 	Draft B
Date  	:   7-April-2022
Author  : 	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION
This is the AudioNav3 module which defines the RTOS Task NAVIGATE.

NAVIGATE receives the following data via inter-task communication queues:
	 - YAW data from the Digital Motion Processor in the MPU6050 IMU
	 - GPS data from the Adafruit Ultimate GPS receiver, specifically:
	  	   SOG, COG, Fix, HDOP and nSatellites

The output from NAVIGATE will (when developed!) be a series of audio instruction
codes, sent via an inter-task queue to the Announce RTOS Task.

 *************************************************************************************/
// #include guard
#ifndef NAVIGATE_HPP
#define NAVIGATE_HPP
//------------------------------------------------------------------------------------

#include "MPU6050.hpp"  	// source of gyro data for Navigate task
#include "GPS.hpp"			// source of GPS data for Navigate task
//#include "Announce.hpp"		// sink for audio announcement requests from Navigate task
#include "WiFiAP.hpp"

#define NAV_DEBUG		1 	// 0 = OFF; 1 = Print sensor data only; 2 = Everything

#define WIFI_LOGGING	1	// 1 = Echo courseSF data to Wi-Fi client

static TimerHandle_t navTimer = NULL;  	// re-load timer to clock the Navigate process

static xSemaphoreHandle navTick = 0;	// Binary semaphore handle for navTimer


void navTimerCallback(TimerHandle_t xTimer){
	xSemaphoreGiveFromISR(navTick, NULL);
}


struct AnnouncementStruct{
	uint8_t type;
	bool prefix;
	float  value;
	bool suffix;
};

static QueueHandle_t AnnouncementQueue = NULL;
static const uint8_t AnnouncementQueueLength = 25;





void navSetup(){
	// Create and start a 100ms software timer to clock the navigation process
	navTimer = xTimerCreate(
			"Navigation Timer",
			100 / portTICK_PERIOD_MS,
			pdTRUE,			// Auto-reload = yes
			(void *)1,		// Timer ID = 1
			navTimerCallback);
	if (navTimer == NULL){
		Serial.println("xTimeCreate failed to create navTimer");
	} else { // Start timer
		// First wait so as to start the timer on a multiple of 100 (for tidiness!)
		while(xTaskGetTickCount() % 100 != 99);
		xTimerStart(navTimer, portMAX_DELAY); // Starts timer on the next tickcount
	}

	//Create a binary semaphore for use by navTimer and the Navigate task
	navTick = xSemaphoreCreateBinary();

	//Create the queue to send Announcement structs to the "Announce" TASK
	AnnouncementQueue = xQueueCreate(AnnouncementQueueLength, sizeof(AnnouncementStruct));
	if(AnnouncementQueue != NULL)
		debug("NAV","Announcement Queue created successfully",true);
	else
		errorReport("NAV",301,"Failed to create AnnouncementQueue in Navigate", true);

	vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 20s for GPS and Gyro to stabilise
}



// Function to calculate deltaYaw, taking account of wrap-around at +/- 180 degrees
float deltaYaw(float thisYaw, float lastYaw){
	float deltaYaw = thisYaw - lastYaw;;
	if (thisYaw * lastYaw < -8100) 		// large -ve number = wrap-around has occurred
		if(thisYaw < lastYaw) 			// wrap with clockwise rotation
			deltaYaw += 360;
		else							// wrap with anti-clockwise rotation
			deltaYaw -= 360;
	return(deltaYaw);
}



// Function "courseSF" combines GPS and gyro data to best effect to give as accurate
// an estimate as possible of the course over the ground, with low latency.
// courseSF = courseSensorFusion
#define ALPHA		0.98   // weighting factor for complementary filter

float courseSF(GPSdataStruct &gps, float deltaYaw){
	static float lastCourseSF = 0;
	static float newCourseSF = 0;

	if (gps.speed > 2.0){   // if speed > 1.0 knot then use the GPS data
		// First deal with wrap around at zero degrees
		if (lastCourseSF - gps.course < -180)
			lastCourseSF += 360;
		else
			if (lastCourseSF - gps.course > 180)
				gps.course += 360;
		// now use complementary filter to estimate new course
		newCourseSF = ((1.0 - ALPHA) * gps.course) + (ALPHA * (lastCourseSF + deltaYaw));
	} else {
		// at speeds of < 1 knot, just use gyro data
		newCourseSF = lastCourseSF + deltaYaw;
	}
	// Now undo any residual effects of wrap-around to give a course between 0 & 360
	if (newCourseSF < 0)
		newCourseSF += 360;  // deal with negative courses

	if (newCourseSF > 360)
		newCourseSF -= 360;

	lastCourseSF = newCourseSF;
	return(newCourseSF);
}

#define COURSE_ANNOUNCEMENT		1
#define HEADING_ANN				2
#define SPEED_ANN				3
#define GPS_QUALITY _ANN		4
#define SATELLITES_ANN			5
#define BEEP_ANN				6




// The RTOS TASK "Navigate"
void navigate(void *parameter){
	static float yaw = 0;			// yaw value from MPU6050 Digital Motion Processor
	static float lastYaw = 0;		// remember previous yaw to calculate deltaYaw
	static float dYaw = 0;			// = (yaw - lastYaw) corrected for wrap-around
	static float course = 101.1;
	static char textBuffer[50];		// form Wi-Fi logging output
	static int length = 0;			// length of the string loaded into textBuffer (excluding NULL)

	static GPSdataStruct GPSdata;	// key data from GPS receiver
	static BaseType_t rxStatus;		// return status from xQueueReceive
	static AnnouncementStruct ann2;
	static AnnouncementStruct ann1;
	static int count = 0;

	// The DO ONCE section of the TASK
	navSetup();
	ann1.type = COURSE_ANNOUNCEMENT;
	ann1.prefix = pdTRUE;
	ann1.value = 263.2;
	ann1.suffix = true;

	// Send this announcement to the "Announce" task via the AnnouncementQueue
	if (xQueueSend(AnnouncementQueue, (void *)&ann1 , 10) != pdTRUE){
		Serial.println("In TASK Navigate: The Announcement Queue is FULL!");
	}


	// The DO FOREVER loop
	while(1){
		// wait for the 100ms navTick software timer to expire
		if(xSemaphoreTake(navTick, 9999))  {
			lastYaw = yaw;  // record previous value of yaw
			// read the latest Gyro yaw data from the IMU task via the YawQueue
			rxStatus =xQueueReceive(YawQueue, (void*)&yaw, 0);
			if(NAV_DEBUG > 1 & rxStatus == pdFALSE)
				Serial.println("YawQueue receive failed (queue empty)");

			// read the latest GPSdata from the GPS task via the GPSQueue
			rxStatus = xQueueReceive(GPSQueue, (void*)&GPSdata, 0);
			if(NAV_DEBUG > 1 & rxStatus == pdFALSE)
				Serial.println("GPSQueue receive failed (queue empty)");

			/* TO BE INSERTED HERE:
			 *
			 *		1. The complementary filter to combine gyro and GPS data into a
			 *		low-latency (100ms) best estimate of COG that is robust in the
			 *		event of lulls in boat speed
			 *
			 *		2. Decision logic to trigger audio navigation 'indicators'
			 *		depending on the actual vs desired course.
			 *		(dependent on operating mode selected)
			 *
			 *		In the meantime just print the input data to serial monitor!
			 */
			dYaw = deltaYaw(yaw , lastYaw);
			course = courseSF(GPSdata, dYaw);

			if (NAV_DEBUG > 0){
				debug("NAV","SOG_COG_FIX_HDOP_SAT_YAW_deltaYAW_courseSF   ",false);
				Serial.print(GPSdata.speed);
				Serial.print(" \t");
				Serial.print(GPSdata.course);
				Serial.print(" \t");
				Serial.print(GPSdata.fix);
				Serial.print(" \t");
				Serial.print(GPSdata.HDOP);
				Serial.print(" \t");
				Serial.print(GPSdata.nSatellites);
				Serial.print(" \t");
				Serial.print(yaw);
				Serial.printf("\t %+3.2f", dYaw);
				Serial.printf("\t %05.1f \n", course);

			}

			if (WIFI_LOGGING & (count % 3 == 0)){
				// Load text buffer with desired output
				length = sprintf(textBuffer,
						"%05.1f   %2.2f     %2d\r\n",
						course , GPSdata.speed, GPSdata.nSatellites);

				for(int i =0; i < WIFI_MAX_CLIENTS; i++){
					if(telnetClients[i]) {
						for (int j = 0; j < length; j++)
							telnetClients[i]->write(textBuffer[j]);
					}
				}
			}
			count++;
		}
	}
} // End of Navigate TASK



//-----------------------------------------------------------------------------------
#endif
