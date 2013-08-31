/***************************************************************************************
 *
 * Title:       PRismino library v1.0
 * File:        prismino.cpp
 * Date:        2013-09-01
 * Author:      Karl Kangur
 *
 ***************************************************************************************/
#include <prismino.h>

// ### SOUND FUNCTIONS

static volatile uint32_t playTime = 0;

// set frequency and playing time in milliseconds
void play(uint16_t frequency, uint16_t time)
{
  // plays sounds using the timer0, timer0 is also used for millis() functions!
  
  // save timer configuration to restore at the end
  uint8_t temp_tccr0a = TCCR0A;
  uint8_t temp_tccr0b = TCCR0B;
  
  // set mode to PWM, Phase Correct, TOP defined by OCR0A, stop timer
  TCCR0A = (1 << WGM00);
  TCCR0B = (1 << WGM02);
  
  // set buzzer pin mode to output
  DDRB |= (1 << 4);
  
  // timer0 prescalers
  const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  
  // scan through prescalars to find the best fit (ocr <= 255)
  uint8_t prescalerIndex;
  uint16_t ocr;
  for(prescalerIndex = 0; prescalerIndex < 5; prescalerIndex++)
  {
    // phase correct PWM means it will count up and then down again
    // thus the frequency must be divided by 2 (bit shift)
    ocr = (F_CPU >> 2) / frequency / prescalers[prescalerIndex];
    if(ocr <= 255)
    {
      // the prescaler and oscillator combination has been found
      break;
    }
  }
  
  // set the frequency
  OCR0A = ocr;
  
  // reset timer0 counter
  TCNT0 = 0;
  
  // evaluate play time (commutation) counter, casting is necessary
  playTime = (uint32_t) 2 * frequency * time / 1000;
  
  // enalble compare interrupt vector
  uint8_t temp_timsk0 = TIMSK0;
  TIMSK0 = (1 << OCIE0A);
  
  // set the prescaler (enable timer)
  TCCR0B |= (prescalerIndex + 1);
  
  // enable global interrupts
  asm("sei");
  
  // wait here until sound is played
  while(playTime);
  
  // set pin value to 0
  PORTB &= ~(1 << 4);
  // set buzzer pin to input mode
  DDRB &= ~(1 << 4);
  
  // restore timer0 interrupt vectors, needed for millis()
  TIMSK0 = temp_timsk0;
  // reset timer0 for Arduino millis() function
  TCCR0A = temp_tccr0a;
  TCCR0B = temp_tccr0b;
}

ISR(TIMER0_COMPA_vect)
{
  // toggle buzzer pin
  PORTB ^= (1 << 4);
  playTime--;
  if(!playTime)
  {
    // disable this compare interrupt vector
    TIMSK0 &= ~(1 << OCIE0A);
  }
}

// ### PWM/MOTOR FUNCTIONS

static volatile int8_t speedLeft, speedRight;

// sets pwm for h-bridge
void setSpeed(int8_t _speedLeft, int8_t _speedRight)
{
  // h-bridge uses timer/counter 4 (10-bit), channels A and B
  // stop timer, set port operations to normal and waveform generation mode to Fast PWM
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  // set fast PWM mode
  TCCR4D = 0;
  TCCR4E = 0;
  
  // do not allow higher or lower values than 100 or -100
  if(_speedLeft < 0)
  {
    speedLeft = _speedLeft < -100 ? -100 : _speedLeft;
  }
  else
  {
    speedLeft = _speedLeft > 100 ? 100 : _speedLeft;
  }
  
  if(_speedRight < 0)
  {
    speedRight = _speedRight < -100 ? -100 : _speedRight;
  }
  else
  {
    speedRight = _speedRight > 100 ? 100 : _speedRight;
  }
  
  // set compare interrupt 
  uint16_t temp;
  
  // timer 4 uses a shared temporary register, write to it and then to the 8-bit register to save the 10-bit value
  temp = (long) 1023 * (speedLeft > 0 ? speedLeft : -speedLeft) / 100;
  
  TC4H = temp >> 8;
  OCR4A = temp & 0xff;
  
  temp = (long) 1023 * (speedRight > 0 ? speedRight : -speedRight) / 100;
  
  TC4H = temp >> 8;
  OCR4B = temp & 0xff;
  
  // reset timer
  TC4H = 0;
  TCNT4 = 0;
  
  // set counter top value (0x3ff = 1023) which gives 16MHz/1024 = 15.625kHz
  TC4H = 3;
  OCR4C = 0xff;
  
  // set h-bridge control ports to output
  DDRB |= (1 << 7) | (1 << 6) | (1 << 5);
  DDRD |= (1 << 6);
  
  #ifdef SLOWDECAY
  // set all values to 1 for slow decay mode (shorts the motor winding)
  PORTB &= ~((1 << 7) | (1 << 6) | (1 << 5));
  PORTD &= ~(1 << 6);
  #else
  // set all values to 0 for fast decay mode (return current goes back to source)
  PORTB |= (1 << 7) | (1 << 6) | (1 << 5);
  PORTD |= (1 << 6);
  #endif
  
  // enable interrupt vectors
  TIMSK4 = (1 << OCIE4A) | (1 << OCIE4B) | (1 << TOIE4);
  // enable overflow and compare interrupts for A and B channels
  TIFR4 = (1 << OCF4A) | (1 << OCF4B) | (1 << TOV4);
  
  // set prescaler to 1 (enable timer)
  TCCR4B = (1 << CS40);
  
  asm("sei");
}

ISR(TIMER4_COMPA_vect)
{
  if(speedLeft > 0)
  {
    #ifdef SLOWDECAY
    PORTD &= ~(1 << 6);
    #else
    PORTB |= (1 << 7);
    #endif
  }
  else if(speedLeft < 0)
  {
    #ifdef SLOWDECAY
    PORTB &= ~(1 << 7);
    #else
    PORTD |= (1 << 6);
    #endif
  }
}

ISR(TIMER4_COMPB_vect)
{
  if(speedRight > 0)
  {
    #ifdef SLOWDECAY
    PORTB &= ~(1 << 6);
    #else
    PORTB |= (1 << 5);
    #endif
  }
  else if(speedRight < 0)
  {
    #ifdef SLOWDECAY
    PORTB &= ~(1 << 5);
    #else
    PORTB |= (1 << 6);
    #endif
  }
}

ISR(TIMER4_OVF_vect)
{
  if(speedLeft > 0)
  {
    #ifdef SLOWDECAY
    PORTD |= (1 << 6);
    #else
    PORTB &= ~(1 << 7);
    #endif
  }
  else if(speedLeft < 0)
  {
    #ifdef SLOWDECAY
    PORTB |= (1 << 7);
    #else
    PORTD &= ~(1 << 6);
    #endif
  }
  
  if(speedRight > 0)
  {
    #ifdef SLOWDECAY
    PORTB |= (1 << 6);
    #else
    PORTB &= ~(1 << 5);
    #endif
  }
  else if(speedRight < 0)
  {
    #ifdef SLOWDECAY
    PORTB |= (1 << 5);
    #else
    PORTB &= ~(1 << 6);
    #endif
  }
}

// ### DIP SWITCH FUNCTIONS

// save pointers to functions to call when a switch is toggeled
static volatile func_t functionPointers[5];

void dipSwitch(uint8_t id, func_t callback, uint8_t mode)
{
  uint8_t extIntBit, intVectBit, intModeBit;
  switch(id)
  {
  case 0:
  extIntBit = INT0;
  intVectBit = INTF0;
  intModeBit = ISC00;
  break;
  case 1:
  extIntBit = INT1;
  intVectBit = INTF1;
  intModeBit = ISC10;
  break;
  case 2:
  extIntBit = INT2;
  intVectBit = INTF2;
  intModeBit = ISC20;
  break;
  case 3:
  extIntBit = INT3;
  intVectBit = INTF3;
  intModeBit = ISC30;
  break;
  }

  if(callback != NULL)
  {
    // enable external interrupts for DIP switch
    EIMSK |= (1 << extIntBit);
    // enable interrupt vector
    EIFR |= (1 << intVectBit);
    // set interrupt call mode (falling, rising, both, low-level)
    EICRA |= (mode << intModeBit);
    // set callback function
    functionPointers[id] = callback;
  }
  else
  {
    // disable the interrupt if the function is NULL
    EIMSK &= ~(1 << extIntBit);
    EIFR &= ~(1 << intVectBit);
    functionPointers[id] = NULL;
  }
}

ISR(INT0_vect)
{
  // call function when the corresponding interrupt is detected
  if(functionPointers[0])
  {
    functionPointers[0]();
  }
}

ISR(INT1_vect)
{
  if(functionPointers[1])
  {
    functionPointers[1]();
  }
}

ISR(INT2_vect)
{
  if(functionPointers[2])
  {
    functionPointers[2]();
  }
}

ISR(INT3_vect)
{
  if(functionPointers[3])
  {
    functionPointers[3]();
  }
}

// set up callback function when the shield's button is clicked
void buttonCallback(func_t callback)
{
  if(callback != NULL)
  {
    EIMSK |= (1 << INT6);
    EIFR |= (1 << INTF6);
    EICRB |= (INT_FALL << ISC60);
    functionPointers[4] = callback;
  }
  else
  {
    EIMSK &= ~(1 << INT6);
    EIFR &= ~(1 << INTF6);
    functionPointers[4] = NULL;
  }  
}

ISR(INT6_vect)
{
  if(functionPointers[4])
  {
    functionPointers[4]();
  }
}

// ### TIMER FUNCTIONS

struct timedFunction
{
  volatile func_t funcPtr;
  volatile uint16_t interval;
  volatile uint16_t counter;
  volatile uint8_t callNumber;
};

// available timed functions slots is defined in the header
timedFunction timedFunctionArray[TIMEDFUNCTIONS];
uint8_t timedFunctionSetup = 0;

// calls functions automatically based on a set interval
// returns the id of the subroutine
int8_t setTimer(func_t callback, uint16_t interval, uint8_t callNumber)
{
  uint8_t i = 0;
 
   // this needs to be called only once
  if(!timedFunctionSetup)
  {
    // set up the timer/counter: CTC, top is OCR1C and 256 prescalser
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    TCNT1 = 0;
    
    // 16MHz/256/6250 = 10Hz
    OCR1A = 6250;
    TIMSK1 = (1 << OCIE1C);
    TIFR1 = (1 << OCF1C);
    
    // just in case enable global interrupt flag
    asm("sei");
    
    // reset all interrupts
    while(i++ < TIMEDFUNCTIONS)
    {
      timedFunctionArray[i].interval = 0;
    }
    
    // upon new calls do not execute the setup part
    timedFunctionSetup = 1;
  }
  
  // register the timed function
  i = 0;
  while(i++ < TIMEDFUNCTIONS)
  {
    // find a free slot
    if(!timedFunctionArray[i].interval)
    {
      timedFunctionArray[i] = (timedFunction){callback, interval, 0, callNumber};
      return i;
    }
  }
  
  // the timed function slots were full and the new timed function couldn't be set
  return -1;
}

// timer 1 compare vector C
ISR(TIMER1_COMPC_vect)
{
  for(uint8_t i = 0; i < TIMEDFUNCTIONS; i++)
  {
    if(timedFunctionArray[i].interval > 0)
    {
      if(timedFunctionArray[i].interval == timedFunctionArray[i].counter)
      {
        // call the function
        timedFunctionArray[i].funcPtr();
        timedFunctionArray[i].counter = 0;
        
        // the requested number of call times has been reached
        if(timedFunctionArray[i].callNumber == 1)
        {
          // reset this timer function
          timedFunctionArray[i].funcPtr = NULL;
          timedFunctionArray[i].interval = 0;
          timedFunctionArray[i].counter = 0;
          timedFunctionArray[i].callNumber = 0;
          
          // do not pass the counter increment and continue to the next timed function
          continue;
        }
        else if(timedFunctionArray[i].callNumber > 0)
        {
          // decrement call number
          timedFunctionArray[i].callNumber--;
        }
      }
      timedFunctionArray[i].counter++;
    }
  }
}

// remove regularly called function
void unsetTimer(uint8_t id)
{
  timedFunctionArray[id] = (timedFunction){NULL, 0, 0, 0};
}
