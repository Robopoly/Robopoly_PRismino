/***************************************************************************************
 *
 * Title:       PRismino library v1.1
 * File:        prismino.h
 * Date:        2014-02-08
 * Author:      Karl Kangur
 * Website:     https://github.com/Robopoly/prismino-library
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
#define BUZZER 8 // PB4

// dip switch pins
#define DIP1 0 // PD2
#define DIP2 1 // PD3
#define DIP3 2 // PD1
#define DIP4 3 // PD0

// button pin
#define BUTTON 7 // PE6

// LED pin
#define LED 13 // PC7

// potentiometer pin
#define POT A0 // PF7

// macro for shortcut and backwards compatibility
#define play(frequency, duration) tone(BUZZER, frequency, duration)

void setSpeed(int8_t, int8_t);

void dipSwitch(uint8_t, void (*callback)(void) = NULL, uint8_t = CHANGE);
void buttonCallback(void (*callback)(void) = NULL);

// number of available slots
#define TIMEDFUNCTIONS 4

int8_t setTimer(void (*callback)(void), uint16_t, uint8_t = 0);
void unsetTimer(uint8_t);

#endif
