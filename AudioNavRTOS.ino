/*====================================================================================
Filename:	AudioNavRTOS.ino
Version : 	Draft A
Date  	:   17-MArch-2022
Author  : 	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION

This is a test sketch for initial experimentation to see what the structure of AudioNav
might look like using a simple subset of RTOS.

The initial application comprises setting up a Wi-fi Access Point and implementing a
simple MENU dialogue between the Wi-Fi AP and associated Telnet Client(s)

*************************************************************************************/
#include "WiFiAP.hpp"


#define CORE_1		1	// all processes will be run on Core 1 for simplicity


void setup() {
	// Open Serial port
	Serial.begin(115200);
	while(!Serial);  // wait for Serial to open
	Serial.println("AudioNavRTOS begins...");
	setupWiFiAP();		// Set-up a Wi-Fi ACCESS POINT on the ESP32

	// Launch a 'forever' task to scan for new Wi-Fi Clients
	xTaskCreatePinnedToCore(  	// Use xTaskCreate() in vanilla FreeRTOS
			WiFiClientScan,  	// Function to be called
			"WiFiClientScan",   // Name of task
			1024,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			CORE_1);     		// Run everything on Core 1 for simplicity



	// Launch a 'forever' task to READ data from Wi-Fi Clients
	xTaskCreatePinnedToCore(  	// Use xTaskCreate() in vanilla FreeRTOS
			WiFiClientRead,  	// Function to be called
			"WiFiClientRead",   // Name of task
			2048,         		// Stack size in bytes
			NULL,         		// Parameter to pass to function (not used)
			1,            		// Task priority (run ALL tasks at P1)
			NULL,         		// Task handle (only needed for SUSPEND or KILL?)
			CORE_1);     		// Run everything on Core 1 for simplicity


} //end setup()



void loop() {
	// Do nothing
	// setup() and loop() run in their own task with Priority 1 on Core 1
	// on the ESP32
}
