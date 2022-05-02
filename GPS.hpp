/*===================================================================================
Filename:	GPS.hpp
Version :	Draft B
Date	: 	7-April-2022
Author	:	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION:

RTOS version of GPS module.

Receives NMEA data (via the UART# and parses teh GPS sentences to extract key data,
which is then stored in a struct and sent via a queue to the CoreTask.

The connections between the Adafruit GPS Ultimate breakout board and the ESP32 are as
follows:

Adafruit GPS   	Description				ESP32 		NOTES

	VPPS		One pulse/s output		-			Not used currently
	Vin			5V						-			Board has 3.3V regulator for chip
	Gnd			Ground					-
	RX			Data RX (UART#2) 		GPIO 17		RX for config. commands from ESP32
	TX			Data TX (UART#2)  		GPIO 16		TX for NMEA sentences to the ESP32
	FIX			FIX flag				-			Not used currently
	UBAT								-			Not used currently
	EN			Bit clock I/P			-			Not used currently
	3.3V out	3.3V output from onboard regulator	Not used currently

Uses HardwareSerial "Serial2" on the ESP32 for GPS I/O.  UART#2 is believed to
have  a read buffer size of 256 bytes.

The GPS receiver is set to transmit two NMEA sentences each second to the ESP32.
These are as follows:
	GPSRMC  (basic navigation information)	circa 72 bytes long
	GPSGGA	(information on fix accuracy)	circa 72 bytes long

If the getGPSdata task is scheduled to run once per second, then the serial-read
buffer should handle the incoming data perfectly adequately

*************************************************************************************/
// #include guard
#ifndef GPS_HPP
#define GPS_HPP
//------------------------------------------------------------------------------------

#include <Adafruit_GPS.h>
#include "AudioNav3.h"
//#include "WiFiServer.hpp"

#define GPS_DEBUG	0	// GPS diagnostic output ON/OFF
#define NMEA_ON 	0	// Turn on echo of NMEA sentence to Serial o/p channel

Adafruit_GPS GPS(&Serial2); // connect GPS instance to ESP32 UART2


struct GPSdataStruct{
	float speed;
	float course;
	uint8_t fix;
	float HDOP;
	uint8_t nSatellites;
};

static const uint8_t GPSqueueLength = 5;
static QueueHandle_t GPSQueue = NULL;


void AdafruitGPS_Setup(){
	// Initialise the Adafruit Ultimate GPS module using the Adafruit_GPS_Library
	debug("GPS","Setting up the Adafruit Ultimate GPS module...",true);
	Serial2.begin(9600,SERIAL_8N1,16,17);  // open UART2 hardware serial port
	while(!Serial2);	// wait for Serial2 port to open
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); 	// RMC & GGA sentences only
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  	// Set GPS update rate to 1Hz

	//Create the GPSQueue to send GPSdataStruct data to the Navigate TASK
	GPSQueue = xQueueCreate(GPSqueueLength, sizeof(GPSdataStruct));
	if(GPSQueue != NULL)
		debug("GPS","GPSQueue created successfully",true);
	else
		errorReport("GPS",101 ,"Failed to create GPSQueue", true);
}





// TASK function to read and parse NMEA sentences from the GPS receiver
// and send results to the navigate Task via the GPSsendQueue
void getGPS(void * parameter){
	static GPSdataStruct GPSdata;

	// The DO ONCE section of the TASK
	AdafruitGPS_Setup();  // Set-up actions for the Adafruit Ultimate GPS receiver

	// The DO FOREVER loop
	// Read the GPS NMEA sentences and parse them to extract the required information
	char c;
	while(1){
		//Serial.println("In task parseGPS");
		c = GPS.read();  		// read one char from input buffer  (c == 0 if buffer empty)
		while (c != 0) {
			if (NMEA_ON) {
				Serial.write(c); 	// echo NMEA sentences to Serial monitor if NMEA_ON set
				//for(int i =0; i < WIFI_MAX_CLIENTS; i++){
				//	if(telnetClients[i]) telnetClients[i]->write(c);
				//}
			}
			if (GPS.newNMEAreceived()) {
				if (!GPS.parse(GPS.lastNMEA())) {  // also sets newNMEAreceived() flag to false
					Serial.println("GPS parsing failure");
				} else {
					//Serial.println("GPS parsing success");
					// Load the GPSdata struct with values to send to the Navigate task
					GPSdata.speed = audioNav.speed = GPS.speed; //GPS.speed;
					GPSdata.course  = audioNav.course = GPS.angle;
					GPSdata.fix = audioNav.GPSfix = GPS.fix;
					GPSdata.HDOP = audioNav.GPSfixAccuracy = GPS.HDOP;	//Horizontal Dilution of Precision
					GPSdata.nSatellites = audioNav.nSatellites = GPS.satellites;

					//Serial.println("Send data to Navigate via GPSQueue");
					if (xQueueSend(GPSQueue, (void *)&GPSdata , 10) != pdTRUE){
						debug("GPS","GPSQueue is FULL!",true);
					}
				}
			}
			c = GPS.read();
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}



//-------------------------------------------------------------------------------------
#endif
