#include <prismino.h>

// ### PWM/MOTOR FUNCTIONS

static volatile int8_t speedLeft, speedRight;

static volatile int speed_1, speed_2;

// structure that holds data about stepper motor position and state
struct stepperData
{
  int16_t position:     16;
  int16_t final:        16;
  uint8_t direction:     1;
  volatile uint8_t busy: 1;
} stepper;

void setupSetSpeed(void)
{
  // Set outuput compare registers to 0
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 0;
  
  // Fast-PWM 10-bit => TOP = 0x3FF => WGM13:0 = 7
  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << WGM13);
  
  // Inverted mode
  TCCR1A |= (1 << COM1A1) | (1 << COM1C1);
  TCCR1A &= ~(1 << COM1A0) & ~(1 << COM1C0);
  
  // Set prescaler to 1
  TCCR1B &= ~(1 << CS12) & ~(1 << CS11);
  TCCR1B |= (1 << CS10);
  
  // Disable output compare interrupts
  TIMSK1 &= ~(1 << ICIE1) & ~(1 << OCIE1C) & ~(1 << OCIE1B) & ~(1 << OCIE1A);
  
  // Set H-bridge pins to LOW
  PORTB &= ~(1 << 7) & ~(1 << 6) & ~(1 << 5);
  PORTD &= ~(1 << 6);
  
  // Set H-bridge pins to OUTPUT
  DDRB |= (1 << 7) | (1 << 6) | (1 << 5);
  DDRD |= (1 << 6);
}

// sets pwm for h-bridge
void setSpeed(int input_speed_1, int input_speed_2)
{
  // Prevent values from being out of the range [-100;100]
  if(input_speed_1 > 0)
  {
    speed_1 = input_speed_1 > 100 ? 100 : input_speed_1;
    // Mapping from 4 to 53 for fastdecay mode
    speed_1 = ( (100 - 53)*(speed_1 - 4) )/(100-4) + 53;
    speed_1 = speed_1 < 0 ? 0 : speed_1;
  }
  else
    speed_1 = input_speed_1 < -100 ? -100 : input_speed_1;
    
  if(input_speed_2 > 0)
  {
    speed_2 = input_speed_2 > 100 ? 100 : input_speed_2;
    // Mapping from 4 to 53 for fastdecay mode
    speed_2 = ( (100 - 53)*(speed_2 - 4) )/(100-4) + 53;
    speed_2 = speed_2 < 0 ? 0 : speed_2;
  }
  else
    speed_2 = input_speed_2 < -100 ? -100 : input_speed_2;
  
  // Set Output Compare Register for Motor 1 with OCR1C and PIN 12 (PD6)
  if(speed_1 > 0)
  {
    OCR1C = ( (int) ( ((double)speed_1*1023)/100) ) ;
    PORTD &= ~(1 << 6);  // set PIN 12 on LOW
  }
  else if(speed_1 < 0)
  {
    OCR1C = 1023 + ((int) ( ((double) speed_1*1023)/100 ) );
    PORTD |= (1 << 6);  // set PIN 12 on HIGH
  }
  else
  {
    OCR1C = 0;
    PORTD &= ~(1 << 6);  // set PIN 12 on LOW
  }
  
  // Set Output Compare Register for Motor 2 with OCR1A and PIN 10 (PB6)
  if(speed_2 > 0)
  {
    OCR1A = ( (int) ( ((double)speed_2*1023)/100) ) ;
    PORTB &= ~(1 << 6);  // set PIN 10 on LOW
  }
  else if(speed_2 < 0)
  {
    OCR1A = 1023 + ((int) ( ((double) speed_2*1023)/100 ) );
    PORTB |= (1 << 6);  // set PIN 10 on HIGH
  }
  else
  {
    OCR1A = 0;
    PORTB &= ~(1 << 6);  // set PIN 10 on LOW
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
