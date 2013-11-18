/***************************************************************************************
 *
 * Title:       PRismino library v1.1
 * File:        prismino.h
 * Date:        2013-11-16
 * Author:      Karl Kangur
 * Website:     https://github.com/Robopoly/prismino-library
 *
 ***************************************************************************************/
#include <Arduino.h>

// prevent errors due to multiple includes
#ifndef _prismino
#define _prismino

// uncomment this line to use the slow decay mode with the H-bridge
//#define SLOWDECAY

// servo pins
#define SERVO1 6 // PORTD7
#define SERVO2 5 // PORTC6

// buzzer pin
#define BUZZER 8 // PORTB4

// dip switch pins
#define DIP1 0 // PORTD2
#define DIP2 1 // PORTD3
#define DIP3 2 // PORTD1
#define DIP4 3 // PORTD0

// button pin
#define BUTTON 7 // PORTE6

// LED pin
#define LED 13 // PORTC7

// potentiometer pin
#define POT A0 // PORTF7

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