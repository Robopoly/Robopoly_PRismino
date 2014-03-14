/***************************************************************************************
 *
 * Title:       Linear camera data fetching
 * Description: Sketch to show how to get the linear camera raw data.
 *
 ***************************************************************************************/
#include <prismino.h>
#include <LinearCamera.h>

// new instance of the camera, it works over i2c and the default address is "5"
LinearCamera lc = LinearCamera(5);

uint8_t *lcDataPtr;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // must be called to fetch data from the linear camera
  lcDataPtr = lc.getPixels();
  
  // send all 102 pixels to serial
  for(uint8_t i = 0; i < LinearCamera::PIXELS; i++)
  {
    Serial.print(*(lcDataPtr + i));
  }
  Serial.print("\n");
  
  delay(500);
}

