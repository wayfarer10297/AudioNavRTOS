/*====================================================================================
Filename:	Announce.hpp
Version	:	Draft A
Author	:	Roger Thompson
Date	: 	30-April-2022
=====================================================================================*/


/**************************************************************************************
DESCRIPTION

This is the Audio Announcement module for the RTOS version of AudioNav (AudioNav3).

Audio output is achieved using the DF Player Mini MP3 player, which has an on-board
micro-SD card holding thee .mp3 sound files to be played.

The library routines retrieve the
required files from SPIFFS, convert them into PWM and then send this to the MAX98357A
class-D audio amplifier chip, using the I2S protocol.

The connections between the DF Player Mini and the ESP32 are as follows:

	DF Player									ESP32
 	----------------------------------			-------------------------------------
 	Ground      								Ground
    Vcc     Supply voltage IN (3.3V - 5V))		5V output
    RX		Data RX	(commands from ESPP32)		GPIO 17		(UART#1 TX)
	TX		Data TX (acks/info to ESP32)		GPIO 16		(UART#1 RX)
	BUSY	LOW = mp3 currently playing			GPIO 4		(hardware interrupt)

**************************************************************************************/
// Include guard
#ifndef ANNOUNCE_HPP
#define ANNOUNCE_HPP
//------------------------------------------------------------------------------------

#include <WiFi.h>
//#include <HardwareSerial.h>
#include <DFPlayerMini_Fast.h>
#include "Announce.h"    		// details of vocabulary and associated mp3 files
#include "Navigate.hpp"


# define START_BYTE 		0x7E	// Each command starts with a '$' sign (0x7E)
# define VERSION 			0xFF
# define CMD_LENGTH 		0x06	// No. of bytes following (excluding Checksum)
# define END_BYTE 			0xEF
# define NO_ACKNOWLEDGE 	0x01 	// No acknowledgement required after command sent
# define ACKNOWLEDGE		0x01	// Acknowledgement is required after command sent

# define ACTIVATED 			LOW

#define SERIAL1_RX_PIN      01		// Pin for UART#1 RX
#define SERIAL1_TX_PIN		02		// Pin for UART#1 TX

#define BUSY_INTERRUPT_PIN	4	// LOW = Busy (i.e. playing mp3); HIGH = Idle



DFPlayerMini_Fast  MP3Player;	// Create DF Player object
//HardwareSerial Serial1(1);		// Serial port for the DF Player (using UART#1)

xSemaphoreHandle mp3PlayerReady = 0;// Binary semaphore handle for isrPlayerReady flag



struct MP3Track{
	byte folderNumber;
	byte fileNumber;
};



void printPlayerDetails();

void playTrack(byte, byte);

void executeCommand(byte, byte, byte);






// Interrupt Service Routine to detect busy/idle status of the MP3 Player
volatile bool playerReady = true;

void IRAM_ATTR isrPlayerReady(){
	xSemaphoreGiveFromISR(mp3PlayerReady, NULL);
	playerReady = true;
	Serial.println("Player Ready interrupt!");
}



void DFPlayerSetup(){
	debug("ANN","Setting up the MP3 Player...",true);
/*	// Start Serial1 (UART#1) for command TX/RX between ESP32 and the DF PLayer Mini
	//Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
	while(!Serial1);	// wait for Serial1 port to open
	//Start the MP3 player
	//MP3Player.begin(Serial1, false);  // false = no debug output
	executeCommand(0x3F, 0, 0);  	// Initialise the DF Player
	MP3Player.volume(20); 			// Set volume level to 20 initially (medium)
	printPlayerDetails();			// Print a summary of key MP3 Player settings
	// Set up PlayerReady Interrupt Service Routine
	pinMode(BUSY_INTERRUPT_PIN, INPUT_PULLUP);
	*/
	attachInterrupt(BUSY_INTERRUPT_PIN, isrPlayerReady, RISING);
	//vTaskDelay(200 / portTICK_PERIOD_MS);
	debug("ANN","MP3 Player setup completed",true);
	//xSemaphoreGiveFromISR(mp3PlayerReady, NULL);
}



// The RTOS TASK "Announce"
// This task services the announcementQueue and plays the requested mp3 files
void Announce (void * parameter) {
	static AnnouncementStruct ann;	// specification of the required announcement
	static BaseType_t qRxStatus;	// status returned by xQueueReceive

	// The DO ONCE section of the TASK
	DFPlayerSetup();

	// The DO FOREVER loop
	while(1){
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		// if there is something in the announcementQueue, then announce it
		if(xQueueReceive(AnnouncementQueue, (void*)&ann, 0)){
			Serial.print("Announcement received in Queue.  ann.value = ");
			Serial.println(ann.value);
			//errorReport("GPS",101 ,"Failed to create GPSQueue", true);
			//while(!xSemaphoreTake(mp3PlayerReady,9999));  // wait for DF Player IDLE
			playTrack(10,0);
		}
	}
}


// Function to execute the required command with the specified parameters
void executeCommand(byte command, byte parameter1, byte parameter2){
	// Calculate the checksum (2 bytes)
	word checksum = -(VERSION + CMD_LENGTH + command + NO_ACKNOWLEDGE + parameter1 + parameter2);
	// Build the command line
	byte Command_line[10] =
	{	START_BYTE,
			VERSION,
			CMD_LENGTH,
		command,
		NO_ACKNOWLEDGE,
		parameter1,
		parameter2,
		highByte(checksum),
		lowByte(checksum),
		END_BYTE
	};
	//Send the command line to the DF Player module
	for (byte k=0; k<10; k++)
	{
		//Serial.print(Command_line[k]);Serial.print(" , ");
		//Serial1.write(Command_line[k]);
	}
	Serial.println("");
	delay(100);  // Delay here is to prevent immediate overwrite by another command
}




void playTrack(byte folder, byte file){
	Serial.print("\nPlay folder ");
	Serial.print(folder);
	Serial.print(", track  ");
	Serial.println(file);
	playerReady = false;  				// Flag the player as BUSY (reversed by ISR)
	executeCommand(0x0F,folder,file);  	// Play the specified track
}



// Function to print out the current settings for the DF Mini PLayer
void printPlayerDetails(){
	Serial.println("DF Mini PLayer configurations is as follows:");
	Serial.println("============================================");
		delay(200);
		Serial.print("Volume is set to:");
		Serial.println(MP3Player.currentVolume());  //read current volume
		delay(200);

		Serial.print("Number of folders:");
		Serial.println(MP3Player.numFolders());
		delay(1000);

		Serial.print("\tNumber of tracks in folders 1 (Numbers): ");
		Serial.println(MP3Player.numTracksInFolder(1));
		delay(1000);

		Serial.print("\tNumber of tracks in folders 2 (Numbers_point): ");
		Serial.println(MP3Player.numTracksInFolder(2));

		Serial.print("\tNumber of tracks in folders 5 (GPS): ");
		Serial.println(MP3Player.numTracksInFolder(5));

		Serial.print("\tNumber of tracks in folders 8 (Tones): ");
		Serial.println(MP3Player.numTracksInFolder(8));

		Serial.print("\tNumber of tracks in folders 9 (Error messages): ");
		Serial.println(MP3Player.numTracksInFolder(9));

		Serial.print("\tNumber of tracks in folders 10 (System): ");
		Serial.println(MP3Player.numTracksInFolder(10));
		delay(1000);
		Serial.println("============================================");
}



























/*

// Pointers to objects (on the HEAP!) used by the ESP8266 library
AudioFileSourceSPIFFS *file;	// ptr to AudioFileSourceSPIFFS instance
AudioGeneratorMP3 *mp3;			// ptr to AudioGeneratorMP3 instance
AudioFileSourceID3 *id3;		// ptr to AudioFileSourceID3 instance
AudioOutputI2S *out;			// ptr to AudioOutputI2S instance


class Announcer {
	// Cyclic queue to hold pending announcements
	const char* announcementQueue[ANNOUNCEMENT_BUFFER_LENGTH];
	int headQueueIndex = 0; 			// head of the announcement queue
	int tailQueueIndex = 0;				// tail of the announcement queue
	int announcementQueueLength = 0; 	// queue is empty
	int sQindex = 0;					// counter to suppress repeat DEBUG messages

public:

	// Method to initialise the audio Announcer and play the start-up announcement
	void begin(const char * mp3Filename){
		out = new AudioOutputI2S(0,0,32,0);
		out -> SetPinout(33, 27, 32); 	// specify ESP32 pins used by the MAX98357A
		mp3 = new AudioGeneratorMP3();
		file = new AudioFileSourceSPIFFS(mp3Filename);
		id3  = new AudioFileSourceID3(file);
		// begin the announcement
		mp3 -> begin(id3, out);
		while (mp3->isRunning()) {
			if (!mp3->loop()) {
				mp3->stop();
				//delete(file);
				//delete(id3);
				//delete(mp3);
			}
		}
	}


	// method to add a new message to the queue
	void cueAnnouncement(const char* message)
	{
		if (audioNav.audioOn){
			announcementQueue[tailQueueIndex] = message;

			announcementQueueLength++;
			tailQueueIndex = ++tailQueueIndex%ANNOUNCEMENT_BUFFER_LENGTH;
			if(ANN_DEBUG>=1){
				debug("ANN","cueAnnouncement. Message queue length = ", false);
				Serial.println(announcementQueueLength);
			}
			if (announcementQueueLength > ANNOUNCEMENT_BUFFER_LENGTH)
			{
				Serial.println("ERROR CODE 300 - Announcement Queue overrun");
			}
		}
		else
		{
			// transmit data over wifi?
		}
	}





	// method to add PERIODIC navigation announcements to the queue (Mode dependent)
	// Modes are:  1:H only;  2: H+C+S;  3: C+S;  4: C only.
	void cuePeriodicAnnouncements()
	{
		if (audioNav.mode == 1 | audioNav.mode == 2)
		{
			cueAnnouncement(HEADING);
			announceNumber(audioNav.heading,3,0,true);
		}
		if ((audioNav.GPSfix) && (audioNav.mode == 2 | audioNav.mode == 4))
		{
			if (audioNav.speed >0.5){
				cueAnnouncement(COURSE);
				announceNumber(audioNav.course,3,0,true);
			}
		}
		if ((audioNav.GPSfix) && (audioNav.mode == 2 | audioNav.mode ==3))
			if(audioNav.speed > 0.5) {
				cueAnnouncement(SPEED);
				announceNumber(audioNav.speed,1,1,false);
				cueAnnouncement(KNOTS);
			} else {
				cueAnnouncement(CURRENTLY_STATIONARY);
			}
		if(ANN_DEBUG>=2)	Serial.println("$ANN - End of cuePeriodicAnnouncements");
	} // End of cuePeriodicAnnouncements






	void announceNumber(float number, int intDigits, int fracDigits, bool leadingZeros){
		float intPart, fracPart;
		int numDigits;
		if (fracDigits == 0) number += 0.5;  // for correct number rounding to integer
		fracPart = modf(number,&intPart);
		if (intPart == 0) numDigits = 0; else numDigits = int(log10(intPart)) + 1;

		if (audioNav.audioOn){
			// announce numbers of integer part
			for (int i = intDigits; i >= 1; i--){
				int digit;
				int divisor;
				divisor =  pow(10, i-1);
				digit = intPart/divisor;
				if ((i == 1)  &  (fracDigits > 0)){
					cueAnnouncement(speak[digit+50]);  		// add announcement of this digit+POINT to the queue
				}
				else
					cueAnnouncement(speak[digit]); 			// add announcement of this digit to the queue
				intPart = intPart - digit * divisor; 		//slice leading digit off of number
			}
			// announce integers of fractional part
			for (int i = 1; i <= fracDigits; i++){
				int digit;
				digit = int(fracPart * 10);
				cueAnnouncement(speak[digit]);
				fracPart = fracPart * 10  - digit;
			}
		}
		else
		{
		}

	}



	void announceGPSfixAccuracy(){
		if (audioNav.audioOn){

			if (audioNav.GPSfix == true ) {
				cueAnnouncement(SATELLITES_ACQUIRED);
				cueAnnouncement(speak[audioNav.nSatellites]);
				if (ANN_DEBUG) {
				debug("ANN","cueAnnouncement(Satellites acquired: )", false);
				Serial.println(audioNav.nSatellites);
			}
			}
			else	{
				cueAnnouncement(GPS_ACQUIRING);
				if (ANN_DEBUG) 	debug("ANN","cueAnnouncement(GPS_ACQUIRING)", true);
				return;
			}

			if (ANN_DEBUG) {
				debug("ANN", "GPS Fix Accuracy (HDOP) =  ", false);
				Serial.println(audioNav.GPSfixAccuracy);}
			if (audioNav.GPSfixAccuracy <= 1) 	cueAnnouncement(FIX_QUALITY_EXCELLENT);
			else
				if (audioNav.GPSfixAccuracy <= 5) 	cueAnnouncement(FIX_QUALITY_GOOD);
				else
					if (audioNav.GPSfixAccuracy <= 10) 	cueAnnouncement(FIX_QUALITY_MODERATE);
					else
						if (audioNav.GPSfixAccuracy <= 20) 	cueAnnouncement(FIX_QUALITY_FAIR);
						else
							cueAnnouncement(FIX_QUALITY_POOR);
		}

	}


	// method to SERVICE the announcement queue
	void serviceQueue()	{
		//audioNav.announcementsQueued = announcementQueueLength; // for diagnostics (see GPS.hpp)
		if(ANN_DEBUG>=1 && (sQindex == 0)){
			Serial.print("$ANN - serviceQueue: Number of announcements waiting = ");
			Serial.println(announcementQueueLength);
			sQindex++; //index suppresses repeat msgs if queue length unchanged
		}

		if (mp3->isRunning())
		{
			if (!mp3->loop()) 	mp3->stop();
		}
		else
		{
			if(ANN_DEBUG>=2) Serial.println("MP3 done ****");
			// if there is a message waiting AND the mp3 object from the previous  announcement has finished transmission
			if (announcementQueueLength > 0)
			{
				const char* mp3Filename;
				mp3Filename = announcementQueue[headQueueIndex];
				if(ANN_DEBUG>0){
					Serial.print("MakeAnnouncement:");
					Serial.println(mp3Filename);
				}
				delete(file);
				//delete(id3);
				//delete(mp3);
				// create new objects for this announcement
				file = new AudioFileSourceSPIFFS(mp3Filename);
				//id3 = new AudioFileSourceID3(file);
				//mp3 = new AudioGeneratorMP3();
				// play announcement
				mp3 -> begin(id3, out);
				headQueueIndex = ++headQueueIndex%ANNOUNCEMENT_BUFFER_LENGTH;  // announcement buffer is cyclic; increment index modulo ANNOUNCEMENT_BUFFER_LENGTH
				announcementQueueLength--;
				sQindex = 0;
			}
		}
	}

}; // End of class Announcer

*/

//-------------------------------------------------------------------------------------
#endif


