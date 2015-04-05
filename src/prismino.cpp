#include <prismino.h>

// ### PWM/MOTOR FUNCTIONS

static volatile int8_t speedLeft, speedRight;

// structure that holds data about stepper motor position and state
struct stepperData
{
  int16_t position:     16;
  int16_t final:        16;
  uint8_t direction:     1;
  volatile uint8_t busy: 1;
} stepper;

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
  // stepper interrupt
  if(stepper.busy)
  {
    // update current position
    stepper.position += stepper.direction ? 1 : -1;

    // switch to next state
    switch(stepper.position & 0b11)
    {
      case 0:
        PORTB &= ~(1 << 7);
        PORTD |= (1 << 6);

        PORTB |= (1 << 6);
        PORTB |= (1 << 5);
        break;
      case 1:
        PORTB |= (1 << 7);
        PORTD |= (1 << 6);

        PORTB &= ~(1 << 6);
        PORTB |= (1 << 5);
        break;
      case 2:
        PORTB |= (1 << 7);
        PORTD &= ~(1 << 6);

        PORTB |= (1 << 6);
        PORTB |= (1 << 5);
        break;
      case 3:
        PORTB |= (1 << 7);
        PORTD |= (1 << 6);

        PORTB |= (1 << 6);
        PORTB &= ~(1 << 5);
        break;
    }

    // goal position reached, disable the timer
    if(stepper.position == stepper.final)
    {
      // stop the timer
      TCCR4B = 0;
      stepper.busy = 0;
    }

    // return from the interrupt as the DC motor control is not used
    return;
  }

  // DC motor interrupts
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

// set up callback function when the shield's button is clicked on pin 7
void buttonCallback(void (*callback)(void))
{
  if(callback != NULL)
  {
    // enable internal pull-up
    pinMode(BTN, INPUT);
    digitalWrite(BTN, HIGH);
    attachInterrupt(4, callback, FALLING);
  }
  else
  {
    // disable internal pull-up
    digitalWrite(BTN, LOW);
    detachInterrupt(4);
  }  
}

// ### TIMER FUNCTIONS

struct timedFunction
{
  void (*callback)(void);
  volatile uint16_t interval;
  volatile uint16_t counter;
  volatile uint8_t callNumber;
};

// available timed functions slots is defined in the header
timedFunction timedFunctionArray[TIMEDFUNCTIONS];
uint8_t timedFunctionSetup = 0;

// calls functions automatically based on a set interval
// returns the id of the subroutine
int8_t setTimer(void (*callback)(void), uint16_t interval, uint8_t callNumber)
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
    for(i = 0; i++ < TIMEDFUNCTIONS; i++)
    {
      timedFunctionArray[i].interval = 0;
    }
    
    // upon new calls do not execute the setup part
    timedFunctionSetup = 1;
  }
  
  // register the timed function
  for(i = 0; i++ < TIMEDFUNCTIONS; i++)
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
        timedFunctionArray[i].callback();
        timedFunctionArray[i].counter = 0;
        
        // the requested number of call times has been reached
        if(timedFunctionArray[i].callNumber == 1)
        {
          // reset this timer function
          timedFunctionArray[i].callback = NULL;
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

// stepper motor methods
Stepper::Stepper()
{
  // initialise stepper motor position variable
  stepper.position = 0;
}

void Stepper::setPosition(int16_t _position)
{
  stepper.position = _position;
}

int16_t Stepper::getPosition()
{
  return stepper.position;
}

uint8_t Stepper::isBusy()
{
  return stepper.busy;
}

void Stepper::moveSteps(int16_t steps, uint16_t frequency)
{
  stepper.busy = 1;

  // stop timer, set port operations to normal and waveform generation mode phase and frequency correct PWM mode
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  // set phase and frequency correct PWM mode
  TCCR4D = (1 << WGM40);
  TCCR4E = 0;
  
  // reset timer
  TC4H = 0;
  TCNT4 = 0;
  
  // set h-bridge control ports to output
  DDRB |= (1 << 7) | (1 << 6) | (1 << 5);
  DDRD |= (1 << 6);
  
  // enable overflow interrupt vector
  TIMSK4 = (1 << TOIE4);
  // enable overflow interrupt
  TIFR4 = (1 << TOV4);

  // define final position to reach
  stepper.final = stepper.position + steps;

  // define rotating direction
  stepper.direction = steps > 0 ? 1 : 0;

  // select a prescaler, timer 4 prescalers are of power of 2 from 1 (1 << 0) to 16384 (1 << 14)
  uint8_t i;
  // timer/counter 4 is a 10-bit counter
  uint16_t top;
  for(i = 1; i < 14; i++)
  {
    // frequency = 16MHz / (2 * prescaler * top) => top = 16MHz / (2 * prescaler * frequency)
    top = (F_CPU >> i) / frequency;
    if(top < 1024)
    {
      break;
    }
  }
  
  // set counter top value
  TC4H = top >> 8;
  OCR4C = top & 0xff;
  
  // the 4 LSB define the prescaler, add 1 because 0 means the timer is off
  TCCR4B = i + 1;
  
  asm("sei");
}
