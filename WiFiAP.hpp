/*===================================================================================
Filename:	WiFiAP.hpp
Version :	Draft A
Date	: 	12-March-2022
Author	:	Roger Thompson
====================================================================================*/

/*************************************************************************************
DESCRIPTION:

This test sketch sets out the basic elements of creating a WiFi Access Point using an
ESP32 for the purposes of establishing a WiFi-based Telnet link to one or more WiFi
clients.
*************************************************************************************/
#ifndef WIFIAP_HPP
#define WIFIAP_HPP
//-----------------------------------------------------------------------------------

#include <WiFi.h>


#define 	WIFI_SSID 		"Telnet_AP"
#define 	WIFI_PASSWORD 	""
#define 	WIFI_TIMEOUT	10000   // timeout after 10s if no Wi-Fi connection made
#define		WIFI_MAX_CLIENTS 5


WiFiServer telnetServer(23);  		// Port number for the Telnet service
WiFiClient **telnetClients = {NULL};


void setupWiFiAP(){
	telnetClients = new WiFiClient*[WIFI_MAX_CLIENTS];
	for (int i=0; i<WIFI_MAX_CLIENTS; i++) telnetClients[i] = NULL;

	WiFi.softAP(WIFI_SSID);
	Serial.println("\n\nWiFi Access Point established");
	telnetServer.begin();
	Serial.println("\nTelnet server started");
	Serial.print("IP address:  ");
	Serial.println(WiFi.softAPIP());
}


// RTOS TASK function:
// ONCE ONLY:  Sets up WiFi ACCESS POINT
// DO FOREVER: Scans once per second to look for new Wi-Fi Clients wishing to connect to the AP
void WiFiClientScan(void * parameter)
{
	setupWiFiAP();  // Set-up a Wi-Fi ACCESS POINT on the ESP32

	while(1){ // do forever
		 // Any new clients?  If not, tempClient is set to NULL
		WiFiClient tempClient = telnetServer.available();
		if (tempClient) {
			// New Wi-Fi client found
			for (int i=0; i < WIFI_MAX_CLIENTS; i++) {
				if (telnetClients[i] == NULL) {
					WiFiClient* client = new WiFiClient(tempClient);
					telnetClients[i] = client;
					break;
				}
			}
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);  // re-scan after one second
	}
}	// end of WiFiClientRead



// RTOS TASK function:
// DO FOREVER: Cycles round Wi-Fi Clients reading characters from their respective
// input buffers, and responding as appropriate based on the incoming data
void WiFiClientRead(void * parameter)
{
	// Menu!
	char option1[] = "1  Change Audio Volume\n";
	char option2[] = "2  Enable/Disable HEADING\n";
	char option3[] = "3  Enable/Disable SOG\n";
	char option4[] = "4  Enable/Disable COG\n";
	char blankLine[] = " \n";

	while(1){ // do forever
		// Cycle round each Client and READ a char from the input buffer if there is one
		for(int client = 0; client < WIFI_MAX_CLIENTS; client++){
			if(telnetClients[client]!= NULL){
				int readResult = telnetClients[client] -> read();
				if (readResult >= 0) {	// -1 is returned if there is nothing to read
					switch (char ch = readResult) {
					case 'M':  // Print the MAIN menu
					case 'm':
						for (int i = 0; i< strlen(option1); i++)	telnetClients[client] -> write(option1[i]);
						for (int i = 0; i< strlen(option2); i++)	telnetClients[client] -> write(option2[i]);
						for (int i = 0; i< strlen(option3); i++)	telnetClients[client] -> write(option3[i]);
						for (int i = 0; i< strlen(option4); i++)	telnetClients[client] -> write(option4[i]);
						for (int i = 0; i< strlen(blankLine); i++)	telnetClients[client] -> write(blankLine[i]);
						break;
					case '0':
						Serial.println(ch);
						break;
					case '1':
						Serial.println(ch);
						break;
					case '2':
						Serial.println(ch);
						break;
					case '\r':
						//Serial.println("Carriage return");
						break;
					case '\n':
						//Serial.println("Line feed");
						break;
					default:
						// for any other characters
						Serial.print("Invalid character entered: ");
						Serial.println(ch);
						break;
					}
				}
			}
		}
		//Serial.println("READ from connected Wi-Fi client(s) completed");
		vTaskDelay(100 / portTICK_PERIOD_MS);  // re-check read buffer after 100ms
	}
}	// end of WiFiClientRead


//------------------------------------------------------------------------------------
#endif
