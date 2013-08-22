/***************************************************************************************
 *
 * Title:       DIP Switch
 * Description: This sketch shows how to use the DIP switch with interrupts.
 *
 ***************************************************************************************/
#include <prismino.h>

// callback functions for DIP switch state changes
void test1()
{
  Serial.println("Switch 1");
}

void test2()
{
  Serial.println("Switch 2");
}

void test3()
{
  Serial.println("Switch 3");
}

void test4()
{
  Serial.println("Switch 4");
}

void setup()
{
  // set LED pin (13) as output
  pinMode(LED, OUTPUT);
  
  // set switch callback, switch number (1 to 4), function to be called when the switch changes states and the callback mode
  // INT_FALLING triggers on a falling edge (1->0 transisiton)
  // INT_RISING triggers on a rising edge (0->1 transisiton)
  // INT_EDGE triggers on either a falling or a rising edge (default)
  // INT_LOW triggers on a low level (0), will continue to trigger if the level is low
  dipSwitch(DIP1, test1, INT_EDGE);
  dipSwitch(DIP2, test2, INT_FALL);
  dipSwitch(DIP3, test3, INT_RISE);
  dipSwitch(DIP4, test4, INT_EDGE);
  
  // remove the interrupt callback on switch 4
  dipSwitch(DIP4, NULL);
  
  Serial.begin(9600);
}

void loop()
{
  // toggle LED every second
  delay(1000);
  digitalWrite(LED, !digitalRead(LED));
}
