/***************************************************************************************
 *
 * Title:       Linear camera peak intensity detection
 * Description: Sketch to show how to get the position of the peak intensity from the
 *              linear camera.
 *
 ***************************************************************************************/
#include <prismino.h>
#include <LinearCamera.h>

// new instance of the camera, it works over i2c and the default address is "5"
LinearCamera lc = LinearCamera(5);

int8_t lcPeak = 0;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // must be called to fetch data from the linear camera
  lc.getPixels();
  
  // this will process the local linear camera data and find the peak position between 0 and 101
  // if the value is 51 it means the peak is straight ahead
  lcPeak = lc.getPeak();
  Serial.println(lcPeak);
  
  delay(100);
}

