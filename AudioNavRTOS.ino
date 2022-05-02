/*====================================================================================
Filename:	AudioNavRTOS.ino
Version : 	Draft C
Date  	:   1-May-2022
Author  : 	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION
This is the 'main' module for AudioNav3 which is an exploratory version of AudioNav
where task scheduling is done using RTOS.

Draft C:  Announce TASK added, fed with requests from Navigate via a Queue
*************************************************************************************/

#include "MPU6050.hpp"
#include "GPS.hpp"
#include "Navigate.hpp"
#include "Announce.hpp"
#include "WiFiAP.hpp"

#if CONFIG_FREERTOS_UNICORE 	// if processor only has one core then use Core 0!
	#define APP_CPU		0
#else
	#define APP_CPU		1		// otherwise use Core 1
#endif

#define I2C_SDA_PIN		22
#define I2C_SDL_PIN		21

void setup() {
	// Open the Serial port
	Serial.begin(115200);
	while(!Serial);			// wait for Serial to open
	Serial.println("\n");
	debug("AN3", "AudioNav3 (RTOS) begins...\n", true);
	Serial.print("Wire"); Serial.println(Wire.available());

	// Start Wire for the I2C bus; clock rate of 400kHz recommended for the ESP32
	Wire.begin();
	Wire.setClock(400000);
	errorReport("AN3",1 ,"Failed to initialise the I2C bus", true);

	// MPU6050_Setup();

	debug("AN3", "End of Main setup().  Now launching RTOS TASKS...\n", true);


//============================   LAUNCH RTOS TASKS   ================================

	// Launch '"getYawPitchRoll TASK from MPU6050 DMP
	xTaskCreatePinnedToCore(
			getYawPitchRoll,  	// The function to be called (defined in MPU6050.hpp)
			"getYawPitchRoll",  // Name of the task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			APP_CPU);     		// Run everything on Core 1 for simplicity

	vTaskDelay(5000 / portTICK_PERIOD_MS);  // Time allowed for set-up actions in TASK
											// Extra time here for IMU calibration

	// Launch "getGPS" TASK to get NMEA data from the GPS module
	xTaskCreatePinnedToCore(
			getGPS,  			// The function to be called (defined in GPS.hpp)
			"getGPS",  			// Name of the task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			APP_CPU);     		// Run everything on Core 1 for simplicity

	vTaskDelay(1000 / portTICK_PERIOD_MS);  // Time to allow DO ONCE set-up actions to complete


	// Launch the "Navigate" TASK to combine GPS & gyro data and give navigation
	xTaskCreatePinnedToCore(
			navigate,  			// Function to be called (defined in Navigation.hpp)
			"Navigate",   		// Name of the task
			2048,         		// Stack size (bytes in ESP32, words in FreeRTOS)
			NULL,         		// Parameter to pass to function
			1,            		// Task priority (0 to configMAX_PRIORITIES - 1)
			NULL,         		// Task handle
			APP_CPU);     		// Run on single specified core  (Core 1 for ESP32)

	vTaskDelay(1000 / portTICK_PERIOD_MS);  // Time to allow DO ONCE set-up actions to complete


	// Launch the "Announce" TASK that generates audio announcements as directed by
	// instructions placed in the AnnouncementQueue
	xTaskCreatePinnedToCore(
			Announce,  			// Function to be called (defined in Announce.hpp)
			"Announce",   		// Name of task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			APP_CPU);     		// Run everything on Core 1 for simplicity

	vTaskDelay(1000 / portTICK_PERIOD_MS);  // Time to allow DO ONCE set-up actions to complete


	// Launch the TASK that sets up a WiFi ACCESS POINT on the ESP32 and then executes a
	//'forever' task to scan for new Wi-Fi Clients
	xTaskCreatePinnedToCore(
			WiFiClientScan,  	// Function to be called (defined in WiFiAP.hpp)
			"WiFiClientScan",   // Name of task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			APP_CPU);     		// Run everything on Core 1 for simplicity

	vTaskDelay(1000 / portTICK_PERIOD_MS);  // Time to allow DO ONCE set-up actions to complete


	// Launch a 'forever' task to READ data from Wi-Fi Clients
	xTaskCreatePinnedToCore(
			WiFiClientRead,  	// Function to be called (defined in WiFiAP.hpp)
			"WiFiClientRead",   // Name of task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			APP_CPU);     		// Run everything on Core 1 for simplicity

	vTaskDelay(1000 / portTICK_PERIOD_MS);  // Time to allow DO ONCE set-up actions to complete
	debug("AN3", "All RTOS TASKS have now been launched\n", true);
} //End of SETUP



void loop() {
	// Do nothing
	// setup() and loop() run in their own task with Priority 1 on Core 1 on ESP32
}
