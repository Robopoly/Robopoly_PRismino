/***************************************************************************************
 *
 * Title:       Button callback
 * Description: This sketch shows how to use the Robopoly shield's button with a callback.
 *
 ***************************************************************************************/
#include <prismino.h>

// called when the button is clicked
void button()
{
  // toggle LED
  digitalWrite(LED, !digitalRead(LED));
}

void setup()
{
  // set up the button callback
  buttonCallback(button);
}

void loop()
{
  // this is free for your program
}
