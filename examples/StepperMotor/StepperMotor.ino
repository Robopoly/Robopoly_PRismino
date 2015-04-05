/***************************************************************************************
 *
 * Title:         StepperMotor
 * Description:   Example sketch to show how to use the Stepper class
 * Documentation: https://github.com/Robopoly/prismino-library#stepper-motor
 *
 ***************************************************************************************/
#include <prismino.h>

// the stepper motor must be bipolar and the number of steps depends on model
Stepper myStepper;

void setup()
{ 
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  
  // reset stepper position to 0 (not actually needed)
  myStepper.setPosition(0);
}

void loop()
{
  // move 100 steps clockwise with a speed of 100Hz/step
  myStepper.moveSteps(100, 100);
  
  // wait for the task to be completed
  while(myStepper.isBusy());
  
  // turn counter clockwise 100 steps at 50Hz
  myStepper.moveSteps(-50, 50);
  
  // while the task is not yet completed toggle the LED
  while(myStepper.isBusy())
  {
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
  }
  
  // turn off the LED
  digitalWrite(LED, LOW);
  
  // report the position of the stepper at the end of the cycle
  Serial.println(myStepper.getPosition());
}
