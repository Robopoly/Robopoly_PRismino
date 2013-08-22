/***************************************************************************************
 *
 * Title:       Sound
 * Description: Use the buzzer to make sounds.
 *
 ***************************************************************************************/
#include <prismino.h>

uint16_t frequency;

void setup()
{

}

void loop()
{
  // play a frequency from 10kHz to 400Hz with steps of 5Hz
  for(frequency = 10000; frequency > 400; frequency -= 5)
  {
    // set the frequency and the playing time in milliseconds
    play(frequency, 10);
  }
}

