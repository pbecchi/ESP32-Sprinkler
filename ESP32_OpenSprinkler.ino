/* ====================================================

Version:	OpenSprinkler_Arduino 2.1.6

Date:		January 2016

Repository:

License:	Creative Commons Attribution-ShareAlike 3.0 license

About:		ESP32_OpenSprinkler is a fork of pbecchi OpenSprinkler ESP8266 code thats derived from Ray Wang Opensprinkler
          Main scope is to use different MCU ESP32 and to use alternative hardware:

				- Your choice of ethernet:
					- Wiznet W5100 Ethernet with onboard SD Card or
					- Enc28j60 ethernet with external SD card
				- Discrete IO outputs instead of using a shift register 

			
	
	
			Refer to the start of 'pins.h' for options to substitute different hardware and turn functions on or off.

			As always - FULL CREDIT to Ray for all his hard work to build and maintain the Open Sprinkler project!

/* ====================================================
Original Opensprinkler code commences below here
==================================================== */

/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
* Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
*
* Main loop wrapper for Arduino
* Feb 2015 @ OpenSprinkler.com
*
* This file is part of the OpenSprinkler Firmware
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see
* <http://www.gnu.org/licenses/>.
*/
#ifndef ESP32
#include <EEPROM.h>
#endif
#include <Arduino.h>
#include "Config.h"
#include "Defines.h"
//necessary with deep search
#include <Time/TimeLib.h>

#ifdef OPENSPRINKLER_ARDUINO

	#include <Wire.h>
#if defined(ESP8266) || defined(ESP32)

//#include "SPIFFSdFat.h"
//#include "PCF8574\PCF8574.h"
#include "PCF8574Mio.h"
#ifndef ESP32
#include <ESP8266mDNS.h>
#endif
#ifndef SDFAT
#ifndef ESP32
#include <FS.h>
#endif
#else 
#include <SD.h>
#include <SdFat.h>
#endif
#endif
	#include <Time.h>
#ifndef DS1307RTC
	#include <DS1307RTC.h>
#else
	#include "RTClib-master\RTClib.h"
#endif

#if defined (LCDI2C)
#if !defined(LCD_SSD1306) 
    #include <LiquidCrystal_I2C.h>
#endif
#else
	#include <LiquidCrystal.h>
#endif

	#ifdef OPENSPRINKLER_ARDUINO_W5100
#if !defined(ESP8266) && !defined(ESP32)

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>


#else

#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#ifndef ESP32
 #include <ESP8266WiFi.h>
 #include <WiFiClientSecure.h>
 #include <DNSServer.h>
 #include <ESP8266WebServer.h>
 #include <WiFiManager.h> 
 #ifdef OTA_UPLOAD
  #include <ArduinoOTA.h>
 #endif
#else
 #include <WiFi.h>
#endif
#endif
		#include "EtherCardW5100.h"
	#else
		#include <EtherCard.h>
	#endif

	#ifdef OPENSPRINKLER_ARDUINO_AUTOREBOOT
		 #include <TimeAlarms.h>
	#endif // OPENSPRINKLER_ARDUINO_AUTOREBOOT

	#ifdef OPENSPRINKLER_ARDUINO_FREEMEM
		  #include <MemoryFree.h>
	#endif // OPENSPRINKLER_ARDUINO_FREEMEM
#endif

#ifndef OPENSPRINKLER_ARDUINO_WDT	// Only needed if not using WDT (to ensure the WDT is disbled)
#if !defined(ESP8266) && !defined(ESP32)
#include <avr/wdt.h>
#endif
#endif
byte DB = 0xFF;     ////________________debug print everywhere_________________
#if defined(ESP8266) || defined(ESP32)
extern DS1307RTC RTC;
#endif
#include "OpenSprinkler.h"
extern PCF8574 PCF[10];

extern OpenSprinkler os;

void do_setup();
void do_loop();

void setup()
{
	delay(2000);
#ifdef EEPROM_ESP
	EEPROM.begin(NVM_SIZE);
	
#endif

#ifdef OPENSPRINKLER_ARDUINO_AUTOREBOOT // Added for Auto Reboot   
   Alarm.alarmRepeat ( REBOOT_HR, REBOOT_MIN, REBOOT_SEC, reboot );
#endif // OPENSPRINKLER_ARDUINO_AUTOREBOOT
   do_setup();
#ifdef OTA_UPLOAD
   // Port defaults to 8266
   // ArduinoOTA.setPort(8266);

   // Hostname defaults to esp8266-[ChipID]
   // ArduinoOTA.setHostname("myesp8266");

   // No authentication by default
   // ArduinoOTA.setPassword((const char *)"123");

   ArduinoOTA.onStart([]() {
	   Serial.println("Start");
   });
   ArduinoOTA.onEnd([]() {
	   write_message("firmware reloaded!");
	   Serial.println("\nEnd");
   });
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
	   Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
   });
   ArduinoOTA.onError([](ota_error_t error) {
	   Serial.printf("Error[%u]: ", error);
	   write_message("firmware upload error!");
	   if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
	   else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
	   else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
	   else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
	   else if (error == OTA_END_ERROR) Serial.println("End Failed");
   });
   ArduinoOTA.begin();
   DB = 0;//_______________________________________________all debug is off enter DB via USB 
#endif
}

void loop()
{
	DEBUG_COMMAND
#ifdef OTA_UPLOAD
		ArduinoOTA.handle();
#endif
   do_loop();
}

#ifdef OPENSPRINKLER_ARDUINO_AUTOREBOOT // Added for Auto Reboot
void ( *resetPointer ) ( void ) = 0;			// AVR software reset function
void reboot()
{
    resetPointer();
}
#endif // OPENSPRINKLER_ARDUINO_AUTOREBOOT

