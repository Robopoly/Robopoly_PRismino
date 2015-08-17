/***************************************************************************************
 *
 * Title:       PRismino library v1.5
 * File:        prismino.h
 * Date:        2015-04-05
 * Author:      Karl Kangur, David Perrenoud
 * Website:     https://github.com/Robopoly/Robopoly_PRismino
 *
 ***************************************************************************************/
#include <Arduino.h>

// prevent errors due to multiple includes
#ifndef _prismino_h
#define _prismino_h

// uncomment this line to use the slow decay mode with the H-bridge
//#define SLOWDECAY

// servomotor pins
#define S1 6 // PD7
#define S2 5 // PC6

// buzzer pin
#define BZR 8 // PB4
#define BUZZER 8 // PB4, for backwards compatibility

// dip switch pins
#define DIP1 0 // PD2
#define DIP2 1 // PD3
#define DIP3 3 // PD1
#define DIP4 2 // PD0

#define dipSwitch(pin, func, mode) attachInterrupt(pin, func, mode);

// button pin
#define BTN 7 // PE6
#define BUTTON 7 // PE6, for backwards compatibility

// LED pin
#define LED 13 // PC7

// potentiometer pin
#define POT A0 // PF7

// macro for shortcut and backwards compatibility
#define play(frequency, duration) tone(BZR, frequency, duration)

void setupSetSpeed(void);

void setSpeed(int input_speed_1, int input_speed_2);

void buttonCallback(void (*callback)(void) = NULL);

class Stepper
{
public:
	Stepper(void);
	void setPosition(int16_t);
	int16_t getPosition(void);
	void moveSteps(int16_t, uint16_t);
	uint8_t isBusy(void);
};

// number of available slots
#define TIMEDFUNCTIONS 4

int8_t setTimer(void (*callback)(void), uint16_t, uint8_t = 0);
void unsetTimer(uint8_t);

#endif
