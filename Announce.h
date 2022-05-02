/*====================================================================================
FilenameE:	Announce.h
Version	:	Draft A
Author	:	Roger Thompson
Date	: 	26/4/22
=====================================================================================*/


/**************************************************************************************
DESCRIPTION

Data associated with .mp3 audio files for the Announce TAs in AudioNAv3.

This is the RTOS version of AudioNAv, which uses the DF MIni Player.

**************************************************************************************/
#ifndef ANNOUNCE_H
#define ANNOUNCE_H
//------------------------------------------------------------------------------------
#define SPEED 					speak[20]
#define KNOTS					speak[21]
#define COURSE					speak[22]
#define DEGREES					speak[23]
#define POINT					speak[24]
#define HEADING					speak[25]
#define AUDIONAV_STARTING 		speak[30]
#define GPS_FIX_OBTAINED 		speak[31]
#define SATELLITES_ACQUIRED		speak[32]
#define FIX_QUALITY_EXCELLENT	speak[34]
#define FIX_QUALITY_GOOD		speak[35]
#define FIX_QUALITY_MODERATE	speak[36]
#define FIX_QUALITY_FAIR		speak[37]
#define FIX_QUALITY_POOR		speak[38]
#define NAVIGATION_BEGINS		speak[40]
#define CURRENTLY_STATIONARY	speak[41]
#define GPS_ACQUIRING		    speak[42]
#define	ERROR_CODE				speak[90]


// Array of pointers to strings holding .mp3 file names for voice announcements
const char* speak[100];

void load_speak_array(){
	// Numbers 0-15


	speak[0]  = "/zero.mp3";
	speak[1]  = "/one.mp3";
	speak[2]  = "/two.mp3";
	speak[3]  = "/three.mp3";
	speak[4]  = "/four.mp3";
	speak[5]  = "/five.mp3";
	speak[6]  = "/six.mp3";
	speak[7]  = "/seven.mp3";
	speak[8]  = "/eight.mp3";
	speak[9]  = "/nine.mp3";
	speak[10] = "/ten.mp3";
	speak[11] = "/eleven.mp3";
	speak[12] = "/twelve.mp3";
	speak[13] = "/thirteen.mp3";
	speak[14] = "/fourteen.mp3";
	speak[15] = "/fifteen.mp3";

	// Keywords
	speak[20] = "/keywords/speed.mp3";
	speak[21] = "/keywords/knots.mp3";
	speak[22] = "/keywords/course.mp3";
	speak[23] = "/keywords/degrees.mp3";
	speak[24] = "/keywords/point.mp3";
	speak[25] = "/keywords/heading.mp3";

	// Various announcements
	speak[30] = "/msg/AudioNav_Starting.mp3";
	speak[31] = "/msg/GPS_fix_obtained.mp3";
	speak[32] = "/msg/satellites_acquired.mp3";
	speak[34] = "/msg/fix_quality_excellent.mp3";
	speak[35] = "/msg/fix_quality_good.mp3";
	speak[36] = "/msg/fix_quality_moderate.mp3";
	speak[37] = "/msg/fix_quality_fair.mp3";
	speak[38] = "/msg/fix_quality_poor.mp3";
	speak[40] =	"/msg/Navigation_begins.mp3";
	speak[41] = "/msg/currently_stationary.mp3";
	speak[42] = "/msg/gps_acquiring.mp3";
	// Numbers-point
	speak[50]  = "/zero_point.mp3";
	speak[51]  = "/one_point.mp3";
	speak[52]  = "/two_point.mp3";
	speak[53]  = "/three_point.mp3";
	speak[54]  = "/four_point.mp3";
	speak[55]  = "/five_point.mp3";
	speak[56]  = "/six_point.mp3";
	speak[57]  = "/seven_point.mp3";
	speak[58]  = "/eight_point.mp3";
	speak[59]  = "/nine_point.mp3";

	// Pauses
	speak[81] = "/pause1.mp3";
	speak[82] = "/pause2.mp3";
	speak[85] = "/pause5.mp3";

	// Error messages
	speak[90] = "/msg/error_code.mp3";
}










//------------------------------------------------------------------------------------
#endif
