/***************************************************************************************
 *
 * Title:       Analog read IR
 * Description: Read an infrared sensor from an analog input port.
 * If the analog value read is < 512, we turn the LED on, else it's turned off.
 * Connect the sensor output to A1.
 * Connect the sensor power rack ENABLE to 4.
 *
 ***************************************************************************************/
#include <prismino.h>

#define IR_EN 4

// the analog value is between 0 and 1023 so it needs a 16-bit variable
// can use "unsigned int" or "uint16_t" for short
unsigned int ir_value;

void setup()
{
  // set pin output mode (sources current)
  pinMode(LED, OUTPUT);
  pinMode(IR_EN, OUTPUT);
}

void loop()
{
  // turn on the emitter and read the IR sensor
  digitalWrite(IR_EN, HIGH); 
  delayMicroseconds(100);
  ir_value = analogRead(A1);
  digitalWrite(IR_EN, LOW); // turn off IR, no need to keep it on all the time

  
  if(ir_value < 512)
  {
    // a low value means something (an obstacle) reflects emitted IR to the receiever
    digitalWrite(LED, HIGH);
  }
  else
  {
    // no obstable is detected, turn LED off
    digitalWrite(LED, LOW);
  }
  delay(100);
}

