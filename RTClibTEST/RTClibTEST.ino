/*
 Name:		RTClibTEST.ino
 Created:	30/07/2017 15:15:30
 Author:	pbecc
*/
/*
* TimeRTC.pde
* Example code illustrating Time library with Real Time Clock.
* This example is identical to the example provided with the Time Library,
* only the #include statement has been changed to include the DS3232RTC
* library.
*/
#include <Arduino.h>
#include "i2crtc.h"
#include <Time.h>
#include <Wire.h>      //http://arduino.cc/en/Reference/Wire
//I2CRTC RTC = I2CRTC();
extern I2CRTC RTC;
void setup(void)
{
	Serial.begin(115200);
	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet)
		Serial.println("Unable to sync with the RTC");
	else
		Serial.println("RTC has set the system time");
}

void loop(void)
{
	digitalClockDisplay();
	delay(1000);
}

void digitalClockDisplay(void)
{
	// digital clock display of the time
	Serial.print(hour());
	printDigits(minute());
	printDigits(second());
	Serial.print(' ');
	Serial.print(day());
	Serial.print(' ');
	Serial.print(month());
	Serial.print(' ');
	Serial.print(year());
	Serial.println();
}

void printDigits(int digits)
{
	// utility function for digital clock display: 
	// prints preceding colon and leading 0
	Serial.print(':');
	if (digits < 10)
		Serial.print('0');
	Serial.print(digits);
}
