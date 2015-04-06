/***************************************************************************************
 *
 * Title:       Linear Camera library v0.1 for the PRsimino
 * File:        LinearCamera.h
 * Date:        2014-03-14
 * Author:      Marco Pagnamenta, Karl Kangur
 * Website:     https://github.com/Robopoly/prismino-library
 *
 ***************************************************************************************/
#ifndef _LINEARCAMERA_H
#define _LINEARCAMERA_H

#include <Arduino.h>

class LinearCamera
{
    public:
    
    // number of pixels on the camera
    static const uint8_t PIXELS = 102;
    
    private:
    
    // two-wire default communication speed in Hz
    static const unsigned long TWISPD = 100000;
    
    uint8_t address;
    uint8_t data[PIXELS];
    
    // the pixels are on addresses 0 to 101
    static const uint8_t ADDR_DATA = 0;
    // on 102 there is the peak region from 0 to 23
    static const uint8_t ADDR_PEAK = 102;
    
    void twiStart(void);
    void twiStop(void);
    void twiWrite(uint8_t);
    uint8_t twiReadAck(void);
    uint8_t twiReadNack(void);
    //uint8_t twiGetStatus(void);
    //uint8_t write(uint8_t data);
    
    public:
    
    LinearCamera(uint8_t);
    uint8_t* getPixels(void);
    int8_t getPeak(void);
    uint8_t getAverage(void);
    uint8_t getMin(void);
    uint8_t getMax(void);
    void setSpeed(unsigned long);
    void setExpTime(uint16_t);
};

#endif
