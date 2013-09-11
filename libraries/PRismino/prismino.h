/***************************************************************************************
 *
 * Title:       PRismino library v1.0
 * File:        prismino.h
 * Date:        2013-09-01
 * Author:      Karl Kangur
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
#define DIP1 2 // PORTD0
#define DIP2 3 // PORTD1
#define DIP3 1 // PORTD3
#define DIP4 0 // PORTD2

// button pin
#define BUTTON 7 // PORTE6

// LED pin
#define LED 13 // PORTC7

// potentiometer pin
#define POT A0 // PORTF7

void play(uint16_t, uint16_t);

void setSpeed(int8_t, int8_t);

// define a function with no arguments pointer type
typedef void (*func_t)(void);

void dipSwitch(uint8_t, func_t = NULL, uint8_t = INT_EDGE);
void buttonCallback(func_t = NULL);

// number of available slots
#define TIMEDFUNCTIONS 4

int8_t setTimer(func_t, uint16_t, uint8_t = 0);
void unsetTimer(uint8_t);

#endif