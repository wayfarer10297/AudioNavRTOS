/*====================================================================================
Filename:	AudioNav3.h
Version	:	Draft C
Date	: 	01-April-2022
Author	:	Roger Thompson
=====================================================================================*/


/*************************************************************************************
DESCRIPTION

Global pre-processor definitions and variable declarations for the AudioNav3 software.

AudioNAv3 is the RTOS trial version

**************************************************************************************/
// #include guard
#ifndef AUDIONAV3_H
#define AUDIONAV3_H
//-------------------------------------------------------------------------------------



#define MAG_DEBUG	0	// Magnetometer diagnostic output ON/OFF
#define ANN_DEBUG	0	// Announcer diagnostic output (levels 0, 1 & 2)
#define MEN_DEBUG	0	// Menu diagnostic output ON/OFF

//#include <ESP32Time.h>	// for Real Time Clock

//ESP32Time rtc;			// instantiation of ESP32Time object (Real Time Clock)

// Data structure holding the KEY STATE VARIABLES used in the operation of AudioNav
// These values are updated on a Scheduled basis by the associated functions/methods
struct AudioNav {
	// Real Time Clock
	bool setRTCfromGPS = false;		// flag to trigger periodic reset the RTC using GPS
	//int RTCupdateInterval = 10;	// interval (seconds) between RTC updates
	// Compass
	bool compassActive = false;		// compass detected and initialised?
	float heading = 0;				// compass heading in degrees
	// GPS
	bool GPSfix = false;			// Have a fix?  True/False
	int nSatellites = 0;			// the number of satellites currently in use
	float GPSfixAccuracy = 100;		// This is the Horizontal Dilution of Precision
	float course = 0.0;				// Course Over the Ground i degrees (from GPS)
	float speed = 0.0;				// Speed Over the Ground in knots (from GPS)
	// Announcer
	bool audioOn = false;			// set to FALSE if WiFi transmission of data in use
	int announcementInterval = 20;	// Periodic announcement interval in seconds
	int announcementsQueued;		// No. of announcements currently queued
	// Menu
	int menuStatus = 0;				// 0 = INACTIVE;
	int mode = 2;					// mode 2 is announcement of Heading, COG & SOG
};  // end of struct AudioNav

AudioNav audioNav;   // instantiate the KEY VARIABLES struct


// Function to print out debug messages in a standard format
void debug(const char * prefix, const char * diagnostic_info, bool newline){
	Serial.printf("%07d ", xTaskGetTickCount());	// Print timestamp in Ticks (ms)
	Serial.printf("$%s:  ", prefix);  				// Print diagnostic prefix
	Serial.print(diagnostic_info);
	// if !newline then additional data to be appended, so  print colon,space
	Serial.print(newline ? "\n" : ": ");
}


void errorReport(const char * prefix, int errorCode, const char * description, bool newline){
	Serial.printf("%07d ", xTaskGetTickCount());	// Print timestamp in Ticks (ms)
	Serial.printf("$%s:  ", prefix);				// Print diagnostic prefix
	Serial.printf("ERROR CODE %03d:", errorCode);			// Print ERROR CODE number
	Serial.print(description);
	// if !newline then additional data to be appended, so just print a space
	if (newline) Serial.print("\n"); else Serial.print(" ");
}


//-------------------------------------------------------------------------------------
#endif
