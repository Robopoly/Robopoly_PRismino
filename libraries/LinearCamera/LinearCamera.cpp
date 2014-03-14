/***************************************************************************************
 *
 * Title:       Linear Camera library v0.1 for the PRsimino
 * File:        LinearCamera.cpp
 * Date:        2014-03-14
 * Author:      Marco Pagnamenta, Karl Kangur
 * Website:     https://github.com/Robopoly/prismino-library
 *
 ***************************************************************************************/
#include "LinearCamera.h"
#include <util/twi.h>

void LinearCamera::twiStart(void)
{
    // twi start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
}

void LinearCamera::twiStop(void)
{
    // twi stop signal
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void LinearCamera::twiWrite(uint8_t data)
{
    // transmit 8 bits to the slave
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    // wait for ack/nack
    while(!(TWCR & (1 << TWINT)));
}

uint8_t LinearCamera::twiReadAck(void)
{
    // read with an acknowlege signaling to send another byte
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while(!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t LinearCamera::twiReadNack(void)
{
    // read without an acknowlege signaling to stop data transmission
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
    return TWDR;
}

/*uint8_t LinearCamera::twiGetStatus(void)
{
    uint8_t status;
    status = TWSR & 0xF8;
    return status;
}

uint8_t LinearCamera::write(uint8_t data)
{
    twiStart();
    // check for start condition status
    if(twiGetStatus() != TW_START)
    {
        return 0;
    }
    twiWrite((this->address << 1) | TW_WRITE);
    // check for slave acknowledge after call
    if(twiGetStatus() != TW_MT_SLA_ACK)
    {
        return 0;
    }
    twiWrite(data);
    // check for slave acknowledge after successful transmission
    if(twiGetStatus() != TW_MT_DATA_ACK)
    {
        return 0;
    }
    // send stop signal to slave
    twiStop();
}*/

LinearCamera::LinearCamera(uint8_t address)
{
    // enable internal pull-ups
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    
    this->setSpeed(TWISPD);
    
    // enable TWI
    TWCR = (1 << TWEN);
    
    // save current linear camera address
    this->address = address;
}

// sets the communication speed, argument in Hz
void LinearCamera::setSpeed(unsigned long twiSpeed)
{
    // set the TWI communication speed with 2 parameters
    unsigned long speed, bestSpeed = 0;
    
    // by default the speed will be set to lowest possible if no solution is found
    uint8_t bestBitRateRegister = 0xff;
    uint8_t bestBitRatePrescaler = 0b11;
    
    // there are 4 available prescalers for TWI: 1, 4, 16 and 64
    for(uint8_t bitRatePrescaler = 0; bitRatePrescaler < 4; bitRatePrescaler++)
    {
        // the bit rate generator is an 8-bit register
        for(uint8_t bitRateRegister = 0; bitRateRegister < 0xff; bitRateRegister++)
        {
            // SCL frequency: F_CPU / (16 + 2 * TWBR * 4^TWPS)
            // see datasheet p. 230
            speed = F_CPU / (16 + 2 * bitRateRegister * (1 << (bitRatePrescaler * 2)));
            //      |                 |                 |
            //      |                 |                 +- 4^TWSR
            //      |                 +------------------- TWBR
            //      +------------------------------------- 16MHz
            
            // find the best solution by iterating through all possibilities
            if(abs(twiSpeed - speed) < abs(twiSpeed - bestSpeed))
            {
                bestSpeed = speed;
                bestBitRateRegister = bitRateRegister;
                bestBitRatePrescaler = bitRatePrescaler;
                
                // exact speed found, stop searching
                if(twiSpeed - speed == 0)
                {
                    break;
                }
            }
        }
    }
    
    // set bit rate register
    TWBR = bestBitRateRegister;
    // reset status register, set bit rate prescaler (TWPS)
    TWSR = (bestBitRatePrescaler << TWPS0);
}

uint8_t* LinearCamera::getPixels()
{
    // request data from slave
    twiStart();
    
    // call slave to write the reading address
    twiWrite((this->address << 1) | TW_WRITE);
    
    // check for identification acknowledge from slave
    if((TWSR & 0xF8) != TW_MT_SLA_ACK)
    {
        // error, did not receieve and acknowledge after being called
        return 0;
    }
    
    // linear camera data is in registers 0 to 101
    twiWrite(ADDR_DATA);
    
    // check for data transfer acknowledge from slave
    if((TWSR & 0xF8) != TW_MT_DATA_ACK)
    {
        // error, did not receieve and acknowledge after data transfer
        return 0;
    }
    
    // read the data
    // send a second start condition
    twiStart();
    
    // one could check for the restart condition here
    
    twiWrite((this->address << 1) | TW_READ);
    // start reading from read table register 0
    for(uint8_t i = 0; i < this->PIXELS - 2; i++)
    {
        // read next byte to data array
        this->data[i] = twiReadAck();
    }
    // read last pixel data without an acknowledge to stop transmission
    this->data[this->PIXELS - 1] = twiReadNack();
    
    twiStop();
    
    // the method caller gets the data pointer back
    return data;
}

int8_t LinearCamera::getPeak()
{
    // this returns the peak position between 0 and 102, 51 being the center
    uint8_t max = 0, region = 0;
    for(uint8_t i = 0; i < this->PIXELS; i++)
    {
        if(this->data[i] > max)
        {
            max = this->data[i];
            region = i;
        }
    }
    return region;
    
    // the linear camera evaluates the peak internally, but
    // it divides the result by 4 to get an 4-pixel average
    /*int8_t peak = -1;
    
    twiStart();
    twiWrite((this->address << 1) | TW_WRITE);
    twiWrite(ADDR_PEAK);
    twiStart();
    twiWrite((this->address << 1) | TW_READ);
    peak = twiReadNack();
    twiStop();
    
    return peak;*/
}

void LinearCamera::setExpTime(uint16_t time)
{
    // set exposition time
    twiStart();
    twiWrite((this->address << 1) | TW_WRITE);
    twiWrite(6);
    // the exposition time is over 2 bytes
    twiWrite(time & 0xFF);
    twiWrite(time >> 8);
    twiStop();
}

uint8_t LinearCamera::getMin(void)
{
    uint16_t min = 0xFF;
    for(uint8_t i = 0; i < this->PIXELS; i++)
    {
        if(this->data[i] < min)
        {
            min = this->data[i];
        }
    }
    // return the average value
    return min;
}

uint8_t LinearCamera::getMax(void)
{
    uint16_t max = 0;
    for(uint8_t i = 0; i < this->PIXELS; i++)
    {
        if(this->data[i] > max)
        {
            max = this->data[i];
        }
    }
    // return the average value
    return max;
}

uint8_t LinearCamera::getAverage(void)
{
    // 102 * 255 = 26010 < 2^16 = 65536
    uint16_t total = 0;
    for(uint8_t i = 0; i < this->PIXELS; i++)
    {
        // sum all values
        total += this->data[i];
    }
    // return the average value
    return total / this->PIXELS;
}
